"""
메인 실행 파일 및 시나리오 분류 파이프라인

이 모듈은 시나리오 분류기 시스템의 메인 실행 파일로, 전체 파이프라인을
구현합니다. 주요 컴포넌트:
- ScenarioLabeler: 4단계 Rule-based 시나리오 분류기
- CustomScenarioVisualizer: 시나리오 시각화 엔진
- ScenarioExporter: JSON 출력 모듈
- process_log_file: 메인 파이프라인 함수

처리 파이프라인:
    1. 맵 데이터 로딩 (MapManager)
    2. JSON 로그 로딩 및 파싱 (JsonLogLoader)
    3. 시나리오 윈도우 추출 (101-epoch 윈도우)
    4. 시나리오 분류 (ScenarioLabeler)
    5. JSON 출력 (ScenarioExporter)
    6. 시각화 (CustomScenarioVisualizer, 선택적)

참조:
    - doc/01_architecture.md: 시스템 아키텍처 및 데이터 흐름
    - doc/03_scenario_labeler.md: 시나리오 분류 알고리즘 상세
    - doc/06_visualization.md: 시각화 시스템 설명
"""

import os
import sys
import json
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend to avoid GUI issues
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import List, Dict, Any, Set, Optional
from dataclasses import dataclass, asdict
from DataTypes import ScenarioLabel, ScenarioWindow, LabeledScenario
import DefaultParams

from MapManager import MapManager
from JsonLogLoader import JsonLogLoader

from nuplan.common.actor_state.state_representation import Point2D
from nuplan.common.actor_state.tracked_objects import TrackedObjects
from nuplan.common.maps.maps_datatypes import SemanticMapLayer
from nuplan.common.actor_state.ego_state import EgoState
from nuplan.common.actor_state.tracked_objects import TrackedObject
from nuplan.common.actor_state.tracked_objects_types import TrackedObjectType



class ScenarioLabeler:
    """
    Rule-based 4단계 시나리오 분류 파이프라인.
    
    ScenarioLabeler는 101-epoch 시나리오 윈도우를 입력받아 다양한 주행 시나리오를
    자동으로 라벨링하는 핵심 컴포넌트입니다. 4단계 분류 파이프라인을 통해
    명시적 상태, 행동, 상호작용, 동역학 특성을 분석합니다.
    
    분류 단계:
        1. State-based Classification (명시적 상태 분류)
           - 속도 프로파일 (저속/중속/고속)
           - 정지 상태 감지
           - 근접 객체 분류
        
        2. Behavior-based Classification (궤적 기반 행동 분류)
           - 회전 감지 (좌회전/우회전, 고속/저속)
           - 차선변경 감지 (좌측/우측)
        
        3. Interaction-based Classification (상호작용 기반 분류)
           - 선행차 추종 (일반/느린 선행차/선행차 없음)
           - 다중 객체 판단 (다중 차량/보행자)
        
        4. Dynamics-based Classification (동역학 분류)
           - Jerk 계산 (급가속/급감속)
           - 측면 가속도 계산
    
    신뢰도 할당:
        각 라벨에는 0.0~1.0 범위의 신뢰도가 할당됩니다.
        - State-based: 0.95~0.99 (매우 높음)
        - Map-based: 0.90~0.95
        - Behavior-based: 0.80~0.85
        - Interaction-based: 0.80~0.90
        - Dynamics-based: 0.95
    
    사용 예시:
        labeler = ScenarioLabeler(map_manager=map_manager, hz=20.0)
        labeled_window = labeler.classify(window)
        # window.labels에 ScenarioLabel 리스트가 채워짐
    
    참조:
        - doc/03_scenario_labeler.md: 분류 알고리즘 상세 설명
        - doc/02_data_structures.md: ScenarioLabel 및 ScenarioWindow 구조
    """
    
    # Configuration constants
    SPEED_LOW_THRESHOLD = 2.78  # m/s (10 km/h)
    SPEED_MEDIUM_THRESHOLD = 11.11  # m/s (40 km/h)
    STATIONARY_SPEED_THRESHOLD = 0.1  # m/s
    STATIONARY_DURATION = 0.5  # seconds
    
    STOPLINE_DISTANCE_THRESHOLD = 2.0  # meters
    INTERSECTION_DISTANCE_THRESHOLD = 5.0
    
    HIGH_SPEED_VEHICLE_THRESHOLD = 20.0  # m/s
    CONSTRUCTION_ZONE_DISTANCE = 20.0  # meters
    TRAFFIC_CONE_DISTANCE = 10.0
    
    TURN_HEADING_THRESHOLD = 15.0  # degrees
    HIGH_SPEED_TURN_THRESHOLD = 8.0  # m/s
    
    LEAD_VEHICLE_DISTANCE = 20.0  # meters
    SLOW_LEAD_SPEED_DIFF = 2.0  # m/s
    
    JERK_THRESHOLD = 10.0  # m/s^3
    LATERAL_ACCELERATION_THRESHOLD = 2.5  # m/s^2
    
    MULTIPLE_VEHICLES_THRESHOLD = 10
    MULTIPLE_PEDESTRIANS_THRESHOLD = 3
    
    LONG_VEHICLE_LENGTH = 8.0  # meters
    
    def __init__(self, map_manager=None, hz: float = 20.0):
        """
        입력:
            map_manager: 맵 데이터 관리자 instance
            hz: 데이터 샘플링 주파수 (기본값: 20.0 Hz)
        """
        self.map_manager = map_manager  # 맵 데이터 접근용
        self.hz = hz                    # 샘플링 주파수
        self.dt = 1.0 / hz              # 시간 간격 (초)
        
        # 라벨 신뢰도 수준 설정
        self.confidence_levels = {
            # High confidence (state-based)
            "low_magnitude_speed": 0.99,
            "medium_magnitude_speed": 0.99,
            "high_magnitude_speed": 0.99,
            "stationary": 0.98,
            "stationary_in_traffic": 0.95,
            
            # Map-based
            "on_stopline_traffic_light": 0.95,
            "on_stopline_stop_sign": 0.95,
            "on_stopline_crosswalk": 0.90,
            "on_intersection": 0.95,
            "on_traffic_light_intersection": 0.95,
            "on_all_way_stop_intersection": 0.90,
            
            # Proximity-based
            "near_high_speed_vehicle": 0.85,
            "near_construction_zone_sign": 0.90,
            "near_trafficcone_on_driveable": 0.85,
            "near_barrier_on_driveable": 0.85,
            "near_long_vehicle": 0.90,
            "near_multiple_vehicles": 0.95,
            "near_multiple_pedestrians": 0.95,
            
            # Behavior-based
            "starting_left_turn": 0.85,
            "starting_right_turn": 0.85,
            "starting_high_speed_turn": 0.80,
            "starting_low_speed_turn": 0.80,
            "changing_lane": 0.80,
            "changing_lane_to_left": 0.80,
            "changing_lane_to_right": 0.80,
            
            # Interaction-based
            "following_lane_with_lead": 0.85,
            "following_lane_with_slow_lead": 0.80,
            "following_lane_without_lead": 0.90,
            "behind_long_vehicle": 0.85,
            "behind_bike": 0.85,
            
            # Dynamics
            "high_magnitude_jerk": 0.95,
            "high_lateral_acceleration": 0.95,
        }
        
        # Label categories
        self.label_categories = {
            "speed_profile": ["low_magnitude_speed", "medium_magnitude_speed", "high_magnitude_speed"],
            "stationary": ["stationary", "stationary_in_traffic"],
            "turning": ["starting_left_turn", "starting_right_turn", "starting_high_speed_turn", "starting_low_speed_turn"],
            "lane_change": ["changing_lane", "changing_lane_to_left", "changing_lane_to_right"],
            "following": ["following_lane_with_lead", "following_lane_with_slow_lead", "following_lane_without_lead"],
            "proximity": ["near_high_speed_vehicle", "near_long_vehicle", "near_multiple_vehicles",
                         "near_construction_zone_sign", "near_trafficcone_on_driveable", "near_barrier_on_driveable", 
                         "behind_long_vehicle", "behind_bike", "near_multiple_pedestrians"],
            "dynamics": ["high_magnitude_jerk", "high_lateral_acceleration"],
        }
    
    def classify(self, window: ScenarioWindow) -> ScenarioWindow:
        """
        4단계 분류 파이프라인 수행
        입력: window - 101-epoch 시나리오 윈도우
        반환값: 라벨링된 시나리오 윈도우
        """
        labels = set()
        
        # 1단계: 명시적 상태 분류 (속도, 위치 기반)
        labels.update(self._classify_explicit_states(window))
        
        # 2단계: 궤적 기반 행동 분류 (차선변경, 회전 등)
        labels.update(self._classify_behaviors(window))
        
        # 3단계: 상호작용 기반 분류 (다른 차량/보행자와의 관계)
        labels.update(self._classify_interactions(window))
        
        # 4단계: 동역학 분류 (가속도, jerk 등)
        labels.update(self._classify_dynamics(window))
        
        # Convert to ScenarioLabel objects
        window.labels = [
            ScenarioLabel(
                label=label,
                confidence=self.confidence_levels.get(label, 0.5),
                category=self._get_label_category(label)
            )
            for label in sorted(labels)
        ]
        
        return window
    
    def _classify_explicit_states(self, window: ScenarioWindow) -> Set[str]:
        """Stage 1: State-based classification"""
        labels = set()
        
        ego = window.ego_current
        agents = window.agents_current
        
        # Speed magnitude
        speed = self._get_speed(ego)
        if speed < self.SPEED_LOW_THRESHOLD:
            labels.add("low_magnitude_speed")
        elif speed < self.SPEED_MEDIUM_THRESHOLD:
            labels.add("medium_magnitude_speed")
        else:
            labels.add("high_magnitude_speed")
        
        # Stationary detection
        if self._is_stationary(window.ego_history[-10:] + [ego]):
            labels.add("stationary")
            
            nearby_vehicles = self._count_nearby_vehicles(ego, agents, radius=30.0)
            if nearby_vehicles > 5:
                labels.add("stationary_in_traffic")
        
        # Proximity to objects
        labels.update(self._classify_proximity(ego, agents))
        
        return labels
    
    def _classify_behaviors(self, window: ScenarioWindow) -> Set[str]:
        """Stage 2: Behavior-based classification"""
        labels = set()
        
        ego_history = window.ego_history
        ego_current = window.ego_current
        ego_future = window.ego_future
        
        # Turning detection
        turn_labels = self._detect_turning(ego_history[-20:], ego_current, ego_future[:20])
        labels.update(turn_labels)
        
        # Lane change detection
        lane_change_labels = self._detect_lane_change(ego_history, ego_current, ego_future)
        labels.update(lane_change_labels)
        
        return labels
    
    def _classify_interactions(self, window: ScenarioWindow) -> Set[str]:
        """Stage 3: Interaction-based classification"""
        labels = set()
        
        ego = window.ego_current
        agents = window.agents_current
        
        # Lead vehicle interaction
        lead_vehicle = self._get_lead_vehicle(ego, agents)
        
        if lead_vehicle:
            labels.add("following_lane_with_lead")
            
            ego_speed = self._get_speed(ego)
            lead_speed = self._get_agent_speed(lead_vehicle)
            
            if lead_speed < ego_speed - self.SLOW_LEAD_SPEED_DIFF:
                labels.add("following_lane_with_slow_lead")
            
            if self._is_long_vehicle(lead_vehicle):
                labels.add("behind_long_vehicle")
            elif lead_vehicle.tracked_object_type == TrackedObjectType.BICYCLE:
                labels.add("behind_bike")
        else:
            labels.add("following_lane_without_lead")
        
        # Multiple agents
        vehicles = [a for a in agents if a.tracked_object_type == TrackedObjectType.VEHICLE]
        pedestrians = [a for a in agents if a.tracked_object_type == TrackedObjectType.PEDESTRIAN]
        
        if len(vehicles) > self.MULTIPLE_VEHICLES_THRESHOLD:
            labels.add("near_multiple_vehicles")
        
        if len(pedestrians) > self.MULTIPLE_PEDESTRIANS_THRESHOLD:
            labels.add("near_multiple_pedestrians")
        
        return labels
    
    def _classify_dynamics(self, window: ScenarioWindow) -> Set[str]:
        """Stage 4: Dynamics classification"""
        labels = set()
        
        if len(window.ego_history) >= 2:
            recent_states = window.ego_history[-2:] + [window.ego_current]
            jerk = self._compute_jerk(recent_states)
            
            if abs(jerk) > self.JERK_THRESHOLD:
                labels.add("high_magnitude_jerk")
        
        if len(window.ego_history) >= 1:
            recent_states = window.ego_history[-1:] + [window.ego_current]
            lateral_acc = self._compute_lateral_acceleration(recent_states)
            
            if abs(lateral_acc) > self.LATERAL_ACCELERATION_THRESHOLD:
                labels.add("high_lateral_acceleration")
        
        return labels
    
    # Helper functions
    def _get_speed(self, ego: EgoState) -> float:
        vx = ego.dynamic_car_state.rear_axle_velocity_2d.x
        vy = ego.dynamic_car_state.rear_axle_velocity_2d.y
        return math.sqrt(vx**2 + vy**2)
    
    def _get_agent_speed(self, agent: TrackedObject) -> float:
        if hasattr(agent, 'velocity'):
            return math.sqrt(agent.velocity.x**2 + agent.velocity.y**2)
        return 0.0
    
    def _is_stationary(self, ego_states: List[EgoState]) -> bool:
        if len(ego_states) < int(self.STATIONARY_DURATION * self.hz):
            return False
        speeds = [self._get_speed(state) for state in ego_states]
        return all(speed < self.STATIONARY_SPEED_THRESHOLD for speed in speeds)
    
    def _count_nearby_vehicles(self, ego: EgoState, agents: List[TrackedObject], radius: float) -> int:
        ego_pos = np.array([ego.rear_axle.x, ego.rear_axle.y])
        count = 0
        for agent in agents:
            if agent.tracked_object_type == TrackedObjectType.VEHICLE:
                agent_pos = np.array([agent.center.x, agent.center.y])
                if np.linalg.norm(ego_pos - agent_pos) < radius:
                    count += 1
        return count
    
    def _classify_proximity(self, ego: EgoState, agents: List[TrackedObject]) -> Set[str]:
        labels = set()
        ego_pos = np.array([ego.rear_axle.x, ego.rear_axle.y])
        
        for agent in agents:
            agent_pos = np.array([agent.center.x, agent.center.y])
            distance = np.linalg.norm(ego_pos - agent_pos)
            
            if agent.tracked_object_type == TrackedObjectType.VEHICLE:
                speed = self._get_agent_speed(agent)
                if speed > self.HIGH_SPEED_VEHICLE_THRESHOLD and distance < 50.0:
                    labels.add("near_high_speed_vehicle")
                
                if self._is_long_vehicle(agent) and distance < 30.0:
                    labels.add("near_long_vehicle")
            
            elif agent.tracked_object_type == TrackedObjectType.CZONE_SIGN:
                if distance < self.CONSTRUCTION_ZONE_DISTANCE:
                    labels.add("near_construction_zone_sign")
            
            elif agent.tracked_object_type == TrackedObjectType.TRAFFIC_CONE:
                if distance < self.TRAFFIC_CONE_DISTANCE:
                    labels.add("near_trafficcone_on_driveable")
            
            elif agent.tracked_object_type == TrackedObjectType.BARRIER:
                if distance < self.TRAFFIC_CONE_DISTANCE:
                    labels.add("near_barrier_on_driveable")
        
        return labels
    
    def _is_long_vehicle(self, agent: TrackedObject) -> bool:
        return agent.box.length > self.LONG_VEHICLE_LENGTH
    
    def _detect_turning(self, ego_history: List[EgoState], ego_current: EgoState, 
                       ego_future: List[EgoState]) -> Set[str]:
        labels = set()
        
        if len(ego_history) < 5 or len(ego_future) < 5:
            return labels
        
        past_heading = ego_history[0].rear_axle.heading
        future_heading = ego_future[-1].rear_axle.heading if ego_future else ego_current.rear_axle.heading
        
        heading_change_rad = self._normalize_angle(future_heading - past_heading)
        heading_change_deg = math.degrees(heading_change_rad)
        
        if abs(heading_change_deg) > self.TURN_HEADING_THRESHOLD:
            if heading_change_deg > 0:
                labels.add("starting_left_turn")
            else:
                labels.add("starting_right_turn")
            
            speed = self._get_speed(ego_current)
            if speed > self.HIGH_SPEED_TURN_THRESHOLD:
                labels.add("starting_high_speed_turn")
            else:
                labels.add("starting_low_speed_turn")
        
        return labels
    
    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _detect_lane_change(self, ego_history: List[EgoState], ego_current: EgoState,
                           ego_future: List[EgoState]) -> Set[str]:
        labels = set()
        
        if len(ego_history) < 10 or len(ego_future) < 10:
            return labels
        
        past_positions = np.array([[s.rear_axle.x, s.rear_axle.y] for s in ego_history[-10:]])
        future_positions = np.array([[s.rear_axle.x, s.rear_axle.y] for s in ego_future[:10]])
        current_pos = np.array([ego_current.rear_axle.x, ego_current.rear_axle.y])
        
        future_vector = future_positions[-1] - current_pos
        past_heading = ego_history[-10].rear_axle.heading
        perpendicular = np.array([-math.sin(past_heading), math.cos(past_heading)])
        lateral_displacement = np.dot(future_vector, perpendicular)
        
        if abs(lateral_displacement) > 1.5:
            labels.add("changing_lane")
            
            if lateral_displacement > 0:
                labels.add("changing_lane_to_left")
            else:
                labels.add("changing_lane_to_right")
        
        return labels
    
    def _get_lead_vehicle(self, ego: EgoState, agents: List[TrackedObject]) -> Optional[TrackedObject]:
        ego_pos = np.array([ego.rear_axle.x, ego.rear_axle.y])
        ego_heading = ego.rear_axle.heading
        ego_forward = np.array([math.cos(ego_heading), math.sin(ego_heading)])
        
        lead_vehicle = None
        min_distance = float('inf')
        
        for agent in agents:
            if agent.tracked_object_type != TrackedObjectType.VEHICLE:
                continue
            
            agent_pos = np.array([agent.center.x, agent.center.y])
            relative_pos = agent_pos - ego_pos
            distance = np.linalg.norm(relative_pos)
            
            forward_distance = np.dot(relative_pos, ego_forward)
            
            if forward_distance > 0 and distance < self.LEAD_VEHICLE_DISTANCE:
                lateral_distance = abs(np.cross(relative_pos, ego_forward))
                
                if lateral_distance < 2.0 and distance < min_distance:
                    min_distance = distance
                    lead_vehicle = agent
        
        return lead_vehicle
    
    def _compute_jerk(self, ego_states: List[EgoState]) -> float:
        if len(ego_states) < 3:
            return 0.0
        
        accels = []
        for state in ego_states:
            ax = state.dynamic_car_state.rear_axle_acceleration_2d.x
            ay = state.dynamic_car_state.rear_axle_acceleration_2d.y
            accel_mag = math.sqrt(ax**2 + ay**2)
            accels.append(accel_mag)
        
        jerk = (accels[-1] - accels[0]) / ((len(accels) - 1) * self.dt)
        return jerk
    
    def _compute_lateral_acceleration(self, ego_states: List[EgoState]) -> float:
        if len(ego_states) < 2:
            return 0.0
        
        state = ego_states[-1]
        speed = self._get_speed(state)
        
        if len(ego_states) >= 2:
            heading_rate = (ego_states[-1].rear_axle.heading - 
                          ego_states[-2].rear_axle.heading) / self.dt
            lateral_acc = speed * heading_rate
            return lateral_acc
        
        return 0.0
    
    def _get_label_category(self, label: str) -> str:
        for category, labels in self.label_categories.items():
            if label in labels:
                return category
        return "other"


class CustomScenarioVisualizer:
    
    def __init__(self, map_manager=None):
        self.map_manager = map_manager  # 맵 데이터 instance
        
        # Visualization settings
        self.bounds = 60
        self.offset = 20
        self.figsize = (10, 10)
        
        # Color mappings
        self.agent_colors = {
            TrackedObjectType.VEHICLE: "#001eff",
            TrackedObjectType.PEDESTRIAN: "#9500ff", 
            TrackedObjectType.BICYCLE: "#ff0059",
            TrackedObjectType.EGO: "#ff7f0e"
        }
        
        # Vehicle parameters
        from nuplan.common.actor_state.vehicle_parameters import get_pacifica_parameters
        self.ego_params = get_pacifica_parameters()
        
        # Coordinate transformation variables
        self.origin = None
        self.angle = None
        self.rot_mat = None
        
        print("   CustomScenarioVisualizer initialized successfully")
    
    def visualize(self, window: ScenarioWindow, save_path: Optional[str] = None) -> Optional[np.ndarray]:
        """Create visualization for a labeled scenario"""
        try:
            # Create figure
            fig, ax = plt.subplots(figsize=self.figsize)
            
            # Setup coordinate system (ego-centric)
            self._setup_coordinate_system(window.ego_current)
            
            # Render components in order (background to foreground)
            self._render_map(ax)
            self._render_trajectories(ax, window)
            self._render_agents(ax, window.agents_current)
            self._render_ego_vehicle(ax, window.ego_current)
            self._render_scenario_labels(ax, window.labels)
            
            # Configure axes
            self._configure_axes(ax)
            
            # Convert to image
            img = self._convert_to_image(fig)
            
            # Save if path provided
            if img is not None and save_path:
                self._save_image(img, save_path)
            
            plt.close(fig)
            return img
            
        except Exception as e:
            import traceback
            print(f"Warning: Custom visualization failed for scenario {window.center_idx}")
            print(f"   Error: {e}")
            print(f"   Traceback: {traceback.format_exc()}")
            return None
    
    def _setup_coordinate_system(self, ego_state: EgoState):
        """Setup ego-centric coordinate transformation"""
        self.origin = ego_state.rear_axle.array
        self.angle = ego_state.rear_axle.heading
        self.rot_mat = np.array([
            [np.cos(self.angle), -np.sin(self.angle)],
            [np.sin(self.angle), np.cos(self.angle)]
        ], dtype=np.float64)
    
    def _transform_coordinates(self, points):
        """Transform world coordinates to ego-centric coordinates"""
        if points.ndim == 1:
            points = points.reshape(1, -1)
        return np.matmul(points - self.origin, self.rot_mat)
    
    def _render_map(self, ax):
        """Render map elements (lanes, crosswalks)"""
        if self.map_manager is None:
            return
            
        try:
            # Get map elements around ego
            query_point = Point2D(self.origin[0], self.origin[1])
            road_elements = [SemanticMapLayer.LANE, SemanticMapLayer.LANE_CONNECTOR]
            
            road_objects = self.map_manager.get_proximal_map_objects(
                query_point, self.bounds + self.offset, road_elements
            )
            
            # Render lanes
            all_lanes = (road_objects[SemanticMapLayer.LANE] + 
                        road_objects[SemanticMapLayer.LANE_CONNECTOR])
            
            for lane in all_lanes:
                # Lane polygon
                polygon_coords = np.array(lane.polygon.exterior.xy).T
                transformed_coords = self._transform_coordinates(polygon_coords)
                
                lane_patch = patches.Polygon(
                    transformed_coords, 
                    color="lightgray", 
                    alpha=0.4, 
                    ec=None, 
                    zorder=0
                )
                ax.add_patch(lane_patch)
                
                # Lane centerline
                centerline = np.array([[s.x, s.y] for s in lane.baseline_path.discrete_path])
                transformed_centerline = self._transform_coordinates(centerline)
                
                ax.plot(
                    transformed_centerline[:, 0], 
                    transformed_centerline[:, 1],
                    color="gray", 
                    alpha=0.5, 
                    linestyle="--", 
                    linewidth=1, 
                    zorder=1
                )
            
            # Render crosswalks
            crosswalks = self.map_manager.get_proximal_map_objects(
                query_point, self.bounds + self.offset, [SemanticMapLayer.CROSSWALK]
            )
            
            for crosswalk in crosswalks[SemanticMapLayer.CROSSWALK]:
                polygon_coords = np.array(crosswalk.polygon.exterior.coords.xy).T
                transformed_coords = self._transform_coordinates(polygon_coords)
                
                crosswalk_patch = patches.Polygon(
                    transformed_coords,
                    color="gray",
                    alpha=0.4,
                    ec=None,
                    zorder=3,
                    hatch="///"
                )
                ax.add_patch(crosswalk_patch)
                
        except Exception as e:
            print(f"   Warning: Map rendering failed: {e}")
    
    def _render_ego_vehicle(self, ax, ego_state: EgoState):
        """Render ego vehicle"""
        try:
            # Vehicle footprint
            footprint_coords = np.array(ego_state.car_footprint.geometry.exterior.xy).T
            transformed_footprint = self._transform_coordinates(footprint_coords)
            
            ego_patch = patches.Polygon(
                transformed_footprint,
                ec=self.agent_colors[TrackedObjectType.EGO],
                fill=False,
                linewidth=2,
                zorder=10
            )
            ax.add_patch(ego_patch)
            
            # Direction indicator (front of vehicle)
            length_indicator = self.ego_params.length * 0.75
            ax.plot(
                [1.69, 1.69 + length_indicator],
                [0, 0],
                color=self.agent_colors[TrackedObjectType.EGO],
                linewidth=2,
                zorder=11
            )
            
        except Exception as e:
            print(f"   Warning: Ego vehicle rendering failed: {e}")
    
    def _render_agents(self, ax, agents: List[TrackedObject]):
        """Render other agents (vehicles, pedestrians, etc.)"""
        for agent in agents:
            try:
                # Transform agent position
                center = self._transform_coordinates(agent.center.array.reshape(1, -1))[0]
                angle = agent.center.heading - self.angle
                
                # Agent box
                agent_coords = np.array(agent.box.geometry.exterior.xy).T
                transformed_coords = self._transform_coordinates(agent_coords)
                
                color = self.agent_colors.get(agent.tracked_object_type, "black")
                
                agent_patch = patches.Polygon(
                    transformed_coords,
                    ec=color,
                    fill=False,
                    alpha=1.0,
                    linewidth=1.5,
                    zorder=4
                )
                ax.add_patch(agent_patch)
                
                # Direction arrow (only if moving fast enough)
                if hasattr(agent, 'velocity') and agent.velocity is not None:
                    velocity_magnitude = np.linalg.norm(agent.velocity.array)
                    if velocity_magnitude > 0.3:  # Only show if moving > 0.3 m/s
                        direction = np.array([np.cos(angle), np.sin(angle)]) * agent.box.length / 2
                        arrow_start = center
                        arrow_end = center + direction
                        
                        ax.plot(
                            [arrow_start[0], arrow_end[0]], 
                            [arrow_start[1], arrow_end[1]],
                            color=color, 
                            linewidth=1, 
                            zorder=4
                        )
                        
            except Exception as e:
                print(f"   Warning: Agent rendering failed: {e}")
    
    def _render_trajectories(self, ax, window: ScenarioWindow):
        """Render ego and agent trajectories (history and future)"""
        try:
            # Ego History trajectory
            if window.ego_history:
                history_points = np.array([state.rear_axle.array for state in window.ego_history])
                transformed_history = self._transform_coordinates(history_points)
                
                ax.plot(
                    transformed_history[:, 0], 
                    transformed_history[:, 1],
                    color=self.agent_colors[TrackedObjectType.EGO],
                    alpha=0.5,
                    linewidth=2,
                    zorder=6,
                    label='Ego History'
                )
            
            # Ego Future trajectory  
            if window.ego_future:
                future_points = np.array([state.rear_axle.array for state in window.ego_future])
                transformed_future = self._transform_coordinates(future_points)
                
                ax.plot(
                    transformed_future[:, 0], 
                    transformed_future[:, 1],
                    color="blue",
                    alpha=0.5,
                    linewidth=2,
                    zorder=6,
                    linestyle=":",
                    label='Ego Future'
                )
            
            # Extract agent trajectories from history, current, and future
            agent_trajectories = self._extract_agent_trajectories(window)
            
            # Render agent trajectories
            for agent_id, trajectory_data in agent_trajectories.items():
                if not trajectory_data:
                    continue
                    
                agent_type = trajectory_data.get('type', TrackedObjectType.VEHICLE)
                history_positions = trajectory_data.get('history', [])
                future_positions = trajectory_data.get('future', [])
                
                # Only render trajectories that are long enough and close enough
                min_trajectory_length = 3
                max_distance_for_trajectory = 50.0  # meters
                
                # Check if agent is close enough to ego
                current_pos = trajectory_data.get('current_position')
                if current_pos is None:
                    continue
                    
                distance_to_ego = np.linalg.norm(current_pos)
                # if distance_to_ego > max_distance_for_trajectory:
                #     continue
                
                # Get agent color (lighter for trajectories)
                base_color = self.agent_colors.get(agent_type, "gray")
                
                # Render agent history trajectory
                if len(history_positions) >= min_trajectory_length:
                    history_array = np.array(history_positions)
                    transformed_agent_history = self._transform_coordinates(history_array)
                    
                    ax.plot(
                        transformed_agent_history[:, 0],
                        transformed_agent_history[:, 1],
                        color=base_color,
                        alpha=0.3,
                        linewidth=1,
                        zorder=5,
                        linestyle='-'
                    )
                
                # Render agent future trajectory
                if len(future_positions) >= min_trajectory_length:
                    future_array = np.array(future_positions)
                    transformed_agent_future = self._transform_coordinates(future_array)
                    
                    ax.plot(
                        transformed_agent_future[:, 0],
                        transformed_agent_future[:, 1],
                        color=base_color,
                        alpha=0.25,
                        linewidth=1,
                        zorder=5,
                        linestyle='--'
                    )
                
        except Exception as e:
            print(f"   Warning: Trajectory rendering failed: {e}")
    
    def _extract_agent_trajectories(self, window: ScenarioWindow) -> Dict[str, Dict]:
        """Extract agent trajectories from window data"""
        agent_trajectories = {}
        
        try:
            # Build agent trajectory database
            # First, collect all agents from current timestep
            for agent in window.agents_current:
                agent_id = agent.track_token
                agent_trajectories[agent_id] = {
                    'type': agent.tracked_object_type,
                    'current_position': np.array([agent.center.x, agent.center.y]),
                    'history': [],
                    'future': []
                }
            
            # Collect history positions for each agent
            for time_step, agents_at_time in enumerate(window.agents_history):
                for agent in agents_at_time:
                    agent_id = agent.track_token
                    if agent_id in agent_trajectories:
                        position = np.array([agent.center.x, agent.center.y])
                        agent_trajectories[agent_id]['history'].append(position)
            
            # Collect future positions for each agent
            for time_step, agents_at_time in enumerate(window.agents_future):
                for agent in agents_at_time:
                    agent_id = agent.track_token
                    if agent_id in agent_trajectories:
                        position = np.array([agent.center.x, agent.center.y])
                        agent_trajectories[agent_id]['future'].append(position)
            
            return agent_trajectories
            
        except Exception as e:
            print(f"   Warning: Agent trajectory extraction failed: {e}")
            return {}
    
    def _render_scenario_labels(self, ax, labels: List[ScenarioLabel]):
        """Render scenario labels as text overlay"""
        if not labels:
            return
            
        try:
            # Group labels by category
            categories = {}
            for label in labels:
                cat = label.category
                if cat not in categories:
                    categories[cat] = []
                categories[cat].append(label)
            
            # Create text content
            text_lines = []
            for category, cat_labels in sorted(categories.items()):
                text_lines.append(f"{category.upper()}:")
                for label in cat_labels:
                    text_lines.append(f"  • {label.label} ({label.confidence:.2f})")
                text_lines.append("")  # Empty line
            
            full_text = "\n".join(text_lines).strip()
            
            if full_text:
                ax.text(
                    0.02, 0.98,
                    full_text,
                    transform=ax.transAxes,
                    fontsize=9,
                    verticalalignment='top',
                    horizontalalignment='left',
                    bbox=dict(
                        boxstyle='round,pad=0.5',
                        facecolor='white',
                        alpha=0.8,
                        edgecolor='black',
                        linewidth=1
                    ),
                    zorder=1000
                )
                
        except Exception as e:
            print(f"   Warning: Scenario labels rendering failed: {e}")
    
    def _configure_axes(self, ax):
        """Configure matplotlib axes"""
        ax.set_xlim(xmin=-self.bounds + self.offset, xmax=self.bounds + self.offset)
        ax.set_ylim(ymin=-self.bounds, ymax=self.bounds)
        ax.set_aspect('equal', adjustable='box')
        ax.axis("off")
        plt.tight_layout(pad=0)
    
    def _convert_to_image(self, fig) -> Optional[np.ndarray]:
        """Convert matplotlib figure to numpy array"""
        try:
            fig.canvas.draw()
            width, height = fig.get_size_inches() * fig.get_dpi()
            
            # Handle different matplotlib versions
            try:
                buf = fig.canvas.buffer_rgba()
                img = np.asarray(buf).reshape(int(height), int(width), 4)
                img = img[:, :, :3]  # RGBA to RGB
            except AttributeError:
                try:
                    img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(
                        int(height), int(width), 3
                    )
                except AttributeError:
                    fig.canvas.draw()
                    buf = fig.canvas.tostring_rgb()
                    img = np.frombuffer(buf, dtype=np.uint8).reshape(
                        int(height), int(width), 3
                    )
            
            return img
            
        except Exception as e:
            print(f"   Warning: Image conversion failed: {e}")
            return None
    
    def _save_image(self, img: np.ndarray, save_path: str):
        """Save image to file"""
        try:
            import cv2
            
            # Ensure output directory exists
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            
            # Convert RGB to BGR for OpenCV
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            success = cv2.imwrite(save_path, img_bgr)
            
            if not success:
                print(f"   Warning: cv2.imwrite failed for {save_path}")
                return False
                
            # Verify file was created
            if not os.path.exists(save_path):
                print(f"   Warning: File not created at {save_path}")
                return False
                
            return True
            
        except Exception as e:
            print(f"   Warning: Image saving failed: {e}")
            return False


class ScenarioExporter:
    """Export labeled scenarios to JSON"""
    
    @staticmethod
    def export_scenario(window: ScenarioWindow, output_path: str):
        """Export single scenario to JSON"""
        ego = window.ego_current
        speed = math.sqrt(
            ego.dynamic_car_state.rear_axle_velocity_2d.x**2 +
            ego.dynamic_car_state.rear_axle_velocity_2d.y**2
        )
        
        agents = window.agents_current
        num_vehicles = sum(1 for a in agents if a.tracked_object_type == TrackedObjectType.VEHICLE)
        num_pedestrians = sum(1 for a in agents if a.tracked_object_type == TrackedObjectType.PEDESTRIAN)
        
        labeled_scenario = LabeledScenario(
            scenario_id=f"scenario_{window.center_idx:06d}",
            center_idx=window.center_idx,
            center_timestamp=window.center_timestamp,
            ego_position={
                'x': float(ego.rear_axle.x),
                'y': float(ego.rear_axle.y),
                'heading': float(ego.rear_axle.heading)
            },
            ego_velocity={
                'vx': float(ego.dynamic_car_state.rear_axle_velocity_2d.x),
                'vy': float(ego.dynamic_car_state.rear_axle_velocity_2d.y),
                'magnitude': float(speed)
            },
            labels=[label.label for label in window.labels],
            label_details=[
                {
                    'label': label.label,
                    'confidence': label.confidence,
                    'category': label.category
                }
                for label in window.labels
            ],
            num_agents=len(agents),
            num_vehicles=num_vehicles,
            num_pedestrians=num_pedestrians,
            confidence_mean=float(np.mean([label.confidence for label in window.labels])) if window.labels else 0.0,
            categories=list(set(label.category for label in window.labels))
        )
        
        scenario_dict = asdict(labeled_scenario)
        
        # Add observation_data - complete 101-epoch data
        observation_data = {
            'ego_history': [
                {
                    'timestamp': ego_state.timestamp_us if hasattr(ego_state, 'timestamp_us') else window.center_timestamp,
                    'position': {
                        'x': float(ego_state.rear_axle.x),
                        'y': float(ego_state.rear_axle.y),
                        'heading': float(ego_state.rear_axle.heading)
                    },
                    'velocity': {
                        'vx': float(ego_state.dynamic_car_state.rear_axle_velocity_2d.x),
                        'vy': float(ego_state.dynamic_car_state.rear_axle_velocity_2d.y)
                    },
                    'acceleration': {
                        'ax': float(ego_state.dynamic_car_state.rear_axle_acceleration_2d.x),
                        'ay': float(ego_state.dynamic_car_state.rear_axle_acceleration_2d.y)
                    }
                }
                for ego_state in window.ego_history
            ],
            'ego_current': {
                'timestamp': window.ego_current.timestamp_us if hasattr(window.ego_current, 'timestamp_us') else window.center_timestamp,
                'position': {
                    'x': float(window.ego_current.rear_axle.x),
                    'y': float(window.ego_current.rear_axle.y),
                    'heading': float(window.ego_current.rear_axle.heading)
                },
                'velocity': {
                    'vx': float(window.ego_current.dynamic_car_state.rear_axle_velocity_2d.x),
                    'vy': float(window.ego_current.dynamic_car_state.rear_axle_velocity_2d.y)
                },
                'acceleration': {
                    'ax': float(window.ego_current.dynamic_car_state.rear_axle_acceleration_2d.x),
                    'ay': float(window.ego_current.dynamic_car_state.rear_axle_acceleration_2d.y)
                }
            },
            'ego_future': [
                {
                    'timestamp': ego_state.timestamp_us if hasattr(ego_state, 'timestamp_us') else window.center_timestamp,
                    'position': {
                        'x': float(ego_state.rear_axle.x),
                        'y': float(ego_state.rear_axle.y),
                        'heading': float(ego_state.rear_axle.heading)
                    },
                    'velocity': {
                        'vx': float(ego_state.dynamic_car_state.rear_axle_velocity_2d.x),
                        'vy': float(ego_state.dynamic_car_state.rear_axle_velocity_2d.y)
                    },
                    'acceleration': {
                        'ax': float(ego_state.dynamic_car_state.rear_axle_acceleration_2d.x),
                        'ay': float(ego_state.dynamic_car_state.rear_axle_acceleration_2d.y)
                    }
                }
                for ego_state in window.ego_future
            ],
            'agents_history': [
                [
                    {
                        'id': agent.track_token,
                        'type': agent.tracked_object_type.name,
                        'position': {
                            'x': float(agent.center.x),
                            'y': float(agent.center.y),
                            'heading': float(agent.center.heading)
                        },
                        'velocity': {
                            'vx': float(agent.velocity.x) if hasattr(agent, 'velocity') and agent.velocity else 0.0,
                            'vy': float(agent.velocity.y) if hasattr(agent, 'velocity') and agent.velocity else 0.0
                        },
                        'box': {
                            'length': float(agent.box.length),
                            'width': float(agent.box.width),
                            'height': float(agent.box.height)
                        }
                    }
                    for agent in agents_at_time
                ]
                for agents_at_time in window.agents_history
            ],
            'agents_current': [
                {
                    'id': agent.track_token,
                    'type': agent.tracked_object_type.name,
                    'position': {
                        'x': float(agent.center.x),
                        'y': float(agent.center.y),
                        'heading': float(agent.center.heading)
                    },
                    'velocity': {
                        'vx': float(agent.velocity.x) if hasattr(agent, 'velocity') and agent.velocity else 0.0,
                        'vy': float(agent.velocity.y) if hasattr(agent, 'velocity') and agent.velocity else 0.0
                    },
                    'box': {
                        'length': float(agent.box.length),
                        'width': float(agent.box.width),
                        'height': float(agent.box.height)
                    }
                }
                for agent in window.agents_current
            ],
            'agents_future': [
                [
                    {
                        'id': agent.track_token,
                        'type': agent.tracked_object_type.name,
                        'position': {
                            'x': float(agent.center.x),
                            'y': float(agent.center.y),
                            'heading': float(agent.center.heading)
                        },
                        'velocity': {
                            'vx': float(agent.velocity.x) if hasattr(agent, 'velocity') and agent.velocity else 0.0,
                            'vy': float(agent.velocity.y) if hasattr(agent, 'velocity') and agent.velocity else 0.0
                        },
                        'box': {
                            'length': float(agent.box.length),
                            'width': float(agent.box.width),
                            'height': float(agent.box.height)
                        }
                    }
                    for agent in agents_at_time
                ]
                for agents_at_time in window.agents_future
            ],
            'traffic_light_status': window.traffic_light_status
        }
        
        scenario_dict['observation_data'] = observation_data
        
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(scenario_dict, f, indent=2, ensure_ascii=False)
    
    @staticmethod
    def export_batch(windows: List[ScenarioWindow], output_dir: str):
        """Export multiple scenarios to JSON files"""
        os.makedirs(output_dir, exist_ok=True)
        
        for window in windows:
            filename = f"scenario_{window.center_idx:06d}.json"
            output_path = os.path.join(output_dir, filename)
            ScenarioExporter.export_scenario(window, output_path)
        
        # Summary file
        summary = {
            'total_scenarios': len(windows),
            'scenarios': [
                {
                    'scenario_id': f"scenario_{w.center_idx:06d}",
                    'center_idx': w.center_idx,
                    'timestamp': w.center_timestamp,
                    'num_labels': len(w.labels),
                    'labels': [label.label for label in w.labels]
                }
                for w in windows
            ]
        }
        
        summary_path = os.path.join(output_dir, 'scenarios_summary.json')
        with open(summary_path, 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2)
        
        print(f"\nExported {len(windows)} scenarios to: {output_dir}")


def process_log_file(log_file_path: str, 
                    map_manager=None,
                    output_dir_json: str = "./labeled_scenarios",
                    output_dir_viz: str = "./imgs",
                    visualize: bool = True,
                    max_scenarios: int = -1,
                    step_size: int = 1):
    """
    JSON 로그 파일을 처리하는 메인 파이프라인 함수.
    
    전체 시나리오 분류 파이프라인을 실행합니다:
    1. 맵 데이터 로딩 (MapManager)
    2. JSON 로그 로딩 및 파싱 (JsonLogLoader)
    3. 시나리오 윈도우 추출 (101-epoch 윈도우)
    4. 시나리오 분류 (ScenarioLabeler)
    5. JSON 출력 (ScenarioExporter)
    6. 시각화 (CustomScenarioVisualizer, 선택적)
    
    Args:
        log_file_path (str): JSON 로그 파일 경로
            DefaultParams.LOG_FILE_NAME에서 지정된 경로를 사용합니다.
        
        map_manager (Optional[MapManager]): 맵 매니저 인스턴스
            None인 경우 자동으로 생성됩니다.
            DefaultParams.MAP_FILE_PATH를 사용하여 맵 데이터베이스를 로드합니다.
        
        output_dir_json (str): JSON 출력 디렉토리 (기본값: "./labeled_scenarios")
            각 시나리오는 개별 JSON 파일로 저장되며, scenarios_summary.json도 생성됩니다.
        
        output_dir_viz (str): 시각화 이미지 출력 디렉토리 (기본값: "./imgs")
            visualize=True인 경우에만 사용됩니다.
        
        visualize (bool): 시각화 이미지 생성 여부 (기본값: True)
            True: 각 시나리오에 대해 PNG 이미지 생성
            False: JSON만 생성 (처리 속도 향상)
        
        max_scenarios (int): 처리할 최대 시나리오 수 (기본값: -1)
            -1: 전체 시나리오 처리
            양수: 해당 개수만 처리 (테스트용)
        
        step_size (int): 시나리오 윈도우 생성 스텝 크기 (기본값: 1)
            1: 모든 가능한 윈도우 생성 (가장 조밀한 샘플링)
            큰 값: 해당 간격으로 샘플링 (예: 100 = 100 프레임마다 윈도우 생성)
    
    처리 과정:
        1. 맵 데이터 로딩
           - MapManager 초기화
           - SQLite 맵 데이터베이스 로드
           - STRtree 공간 인덱스 구축
        
        2. JSON 로그 로딩 및 파싱
           - JsonLogLoader를 사용하여 JSON 파일 로드
           - 각 엔트리를 LogEntry로 변환
           - 좌표계 변환 (로컬 → UTM)
        
        3. 시나리오 윈도우 추출
           - 101-epoch 윈도우 생성 (과거 40 + 현재 1 + 미래 60)
           - step_size에 따라 샘플링
           - 유효한 범위 내에서만 윈도우 생성
        
        4. 시나리오 분류
           - ScenarioLabeler를 사용하여 각 윈도우 분류
           - 4단계 분류 파이프라인 실행
           - 라벨 및 신뢰도 할당
        
        5. JSON 출력
           - 각 시나리오를 개별 JSON 파일로 저장
           - scenarios_summary.json 생성
        
        6. 시각화 (선택적)
           - CustomScenarioVisualizer를 사용하여 PNG 이미지 생성
           - Ego 중심 좌표계로 시각화
    
    출력:
        - JSON 파일: output_dir_json/scenario_XXXXXX.json
        - 요약 파일: output_dir_json/scenarios_summary.json
        - 이미지 파일 (visualize=True): output_dir_viz/scenario_XXXXXX.png
    
    참조:
        - doc/01_architecture.md: 전체 파이프라인 설명
        - doc/03_scenario_labeler.md: 분류 알고리즘 상세
        - doc/06_visualization.md: 시각화 시스템 설명
    
    입력 파라미터:
        log_file_path: JSON 로그 파일 경로
        map_manager: 맵 매니저 객체
        output_dir_json: JSON 결과 출력 디렉토리
        output_dir_viz: 시각화 이미지 출력 디렉토리
        visualize: 시각화 생성 여부
        max_scenarios: 처리할 최대 시나리오 수 (-1 = 전체)
        step_size: 시나리오 생성 스텝 크기 (1 = 전체, other = 샘플링)
    """

    # Add path for imports
    sys.path.insert(0, os.path.dirname(log_file_path))
    
    # 지도 초기화
    map_file_path = os.path.join(os.getcwd(), DefaultParams.MAP_FILE_PATH)
    
    print(f"지도 파일 경로: {map_file_path}")
    
    # 지도 파일이 없는 경우 exit
    if not os.path.exists(map_file_path):
        print("Map file not found")
        sys.exit(0)
            
    
    # 맵 매니저 초기화
    map_manager = MapManager(map_file_path)
    
    # 모든 레이어 초기화
    map_manager.initialize_all_layers()

    print(f"\n1. Loading log file: {log_file_path}")
    log_loader = JsonLogLoader(log_file_path, [DefaultParams.MAP_ORIGIN_X, DefaultParams.MAP_ORIGIN_Y])
    
    parsed_entries = log_loader.get_parsed_entries(map_manager)
    ego_states = log_loader.get_ego_states(parsed_entries)
    dynamic_agents = log_loader.get_dynamic_agents(parsed_entries)
    
    print(f"   Total entries: {len(parsed_entries)}")
    
    # Window parameters
    history_epochs = 40
    future_epochs = 60
    
    print(f"\n2. Extracting 101-epoch windows")
    print(f"   History: {history_epochs}, Current: 1, Future: {future_epochs}")
    
    # Extract windows
    windows = []
    start_idx = history_epochs
    end_idx = len(parsed_entries) - future_epochs
    
    if start_idx >= end_idx:
        print(f"ERROR: Insufficient data. Need at least {history_epochs + 1 + future_epochs} entries.")
        return
    
    # Calculate valid scenarios considering step_size
    total_valid_range = end_idx - start_idx
    valid_count = (total_valid_range + step_size - 1) // step_size  # Ceiling division
    
    if max_scenarios > 0:
        valid_count = min(valid_count, max_scenarios)
    
    print(f"   Valid scenarios (step_size={step_size}): {valid_count}")
    print(f"   Scenario indices: {start_idx}, {start_idx + step_size}, {start_idx + 2*step_size}, ...")
    
    for i in range(valid_count):
        center_idx = start_idx + i * step_size
        
        # Check if center_idx is still within valid range
        if center_idx >= end_idx:
            break
            
        window_start = center_idx - history_epochs
        window_end = center_idx + future_epochs + 1
        
        window = ScenarioWindow(
            center_idx=center_idx,
            center_timestamp=parsed_entries[center_idx].timestamp_us,
            ego_history=ego_states[window_start:center_idx],
            ego_current=ego_states[center_idx],
            ego_future=ego_states[center_idx+1:window_end],
            agents_history=dynamic_agents[window_start:center_idx],
            agents_current=dynamic_agents[center_idx],
            agents_future=dynamic_agents[center_idx+1:window_end],
        )
        
        windows.append(window)
    
    # Classify
    print(f"\n3. Classifying scenarios")
    labeler = ScenarioLabeler(map_manager=map_manager, hz=20.0)
    
    for i, window in enumerate(windows):
        if (i + 1) % 100 == 0:
            print(f"   Processed {i+1}/{len(windows)}...")
        labeler.classify(window)
    
    print(f"   Classification complete!")
    
    # Export JSON
    print(f"\n4. Exporting to JSON")
    ScenarioExporter.export_batch(windows, output_dir_json)
    
    # Visualize
    if visualize:
        print(f"\n5. Creating visualizations")
        os.makedirs(output_dir_viz, exist_ok=True)
        
        print("   Initializing CustomScenarioVisualizer...")
        visualizer = CustomScenarioVisualizer(map_manager=map_manager)
        
        # Generate visualizations for ALL scenarios
        success_count = 0
        for i, window in enumerate(windows):
            if (i + 1) % 100 == 0:  # Progress updates every 100 scenarios
                print(f"   Processed {i+1}/{len(windows)} visualizations... (Success: {success_count})")
            
            viz_path = os.path.join(output_dir_viz, f"scenario_{window.center_idx:06d}.png")
            img = visualizer.visualize(window, save_path=viz_path)
            
            if img is not None:
                success_count += 1
            
        print(f"   Visualization complete: {success_count}/{len(windows)} successful")
        if success_count == 0:
            print("   ERROR: No visualizations were generated! Check the error messages above.")
    
    # Statistics
    print(f"\n6. Statistics")
    print("="*80)
    
    label_counts = {}
    for window in windows:
        for label in window.labels:
            label_counts[label.label] = label_counts.get(label.label, 0) + 1
    
    print(f"\nTop 20 Most Frequent Labels:")
    for label, count in sorted(label_counts.items(), key=lambda x: x[1], reverse=True)[:20]:
        percentage = 100 * count / len(windows)
        print(f"   {label:50s}: {count:4d} ({percentage:5.1f}%)")
    
    avg_labels = np.mean([len(window.labels) for window in windows])
    print(f"\nAverage labels per scenario: {avg_labels:.2f}")
    
    print("\n" + "="*80)
    print(f"Results saved:")
    print(f"  - JSON: {output_dir_json}")
    if visualize:
        print(f"  - Images: {output_dir_viz}")
    print("="*80)


#==============================================================================
# Entry Point
#==============================================================================

if __name__ == "__main__":
    
    print(f"설정:")
    print(f"  - 로그 파일: {DefaultParams.LOG_FILE_NAME}")
    print(f"  - JSON 출력: {DefaultParams.OUTPUT_DIR_SCENARIO_FILE}")
    print(f"  - 이미지 출력: {DefaultParams.OUTPUT_DIR_IMAGE_FILE}")
    print(f"  - 최대 시나리오 수: {DefaultParams.MAX_SCENARIOS if DefaultParams.MAX_SCENARIOS > 0 else '전체'}")
    print(f"  - 시각화: {'활성화' if DefaultParams.VISUALIZE else '비활성화'}")
    print(f"  - 스텝 크기: {DefaultParams.STEP_SIZE} ({'전체 시나리오' if DefaultParams.STEP_SIZE == 1 else f'{DefaultParams.STEP_SIZE}개씩 건너뛰기'})")
    print()
    
    process_log_file(
        log_file_path=DefaultParams.LOG_FILE_NAME,
        map_manager=None,
        output_dir_json=DefaultParams.OUTPUT_DIR_SCENARIO_FILE,
        output_dir_viz=DefaultParams.OUTPUT_DIR_IMAGE_FILE,
        visualize=DefaultParams.VISUALIZE,
        max_scenarios=DefaultParams.MAX_SCENARIOS,
        step_size=DefaultParams.STEP_SIZE
    )