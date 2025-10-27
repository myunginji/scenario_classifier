# JSON 처리 및 수치 계산 라이브러리
import json
import numpy as np
from typing import List, Dict, Any, Optional, Union

# 프로젝트 내부 타입 정의
from ObjectTypes import LogEntry
from TrafficLightDataTypes import TrafficLightStatusData, TrafficLightStatusType

# NuPlan 프레임워크의 차량 상태 및 객체 관련 타입들
from nuplan.common.actor_state.ego_state import EgoState
from nuplan.common.actor_state.agent import Agent
from nuplan.common.actor_state.state_representation import StateSE2, StateVector2D, TimePoint
from nuplan.common.actor_state.vehicle_parameters import VehicleParameters, get_pacifica_parameters
from nuplan.common.actor_state.oriented_box import OrientedBox
from nuplan.common.actor_state.scene_object import SceneObjectMetadata
from nuplan.common.actor_state.static_object import StaticObject
from nuplan.common.actor_state.tracked_objects import TrackedObject
from nuplan.common.actor_state.tracked_objects_types import AGENT_TYPES, TrackedObjectType
from nuplan.common.utils.helpers import get_unique_incremental_track_id


class JsonLogLoader:
    """
    JSON 로그 파일 로더
    - JSON 형식의 주행 로그 데이터 로딩
    """
    
    def __init__(self, log_file_path: str, mapOrigin):
        """
        로더 초기화
        입력:
            log_file_path: JSON 로그 파일 경로
            mapOrigin: 맵 원점 좌표 [x, y]
        """
        self.log_file_path = log_file_path    # 로그 파일 경로
        self.data = None                      # 로드된 JSON 데이터
        self.MAP_ORIGIN_X = float(mapOrigin[0])  # 맵 원점 X 좌표
        self.MAP_ORIGIN_Y = float(mapOrigin[1])  # 맵 원점 Y 좌표
        
        # 카테고리 번호를 TrackedObjectType으로 매핑하는 딕셔너리
        self.category_to_tracked_object_type = {
            0: TrackedObjectType.VEHICLE,      # 차량
            1: TrackedObjectType.PEDESTRIAN,   # 보행자
            2: TrackedObjectType.BICYCLE,      # 자전거
            3: TrackedObjectType.TRAFFIC_CONE, # 콘
            4: TrackedObjectType.BARRIER,      # 
            5: TrackedObjectType.CZONE_SIGN,   # 공사구역 표지판
            6: TrackedObjectType.GENERIC_OBJECT, # 기타
        }
    
    def load(self) -> List[Dict[str, Any]]:
        """
        JSON 로그 파일에서 데이터를 로드
        반환값: 로드된 JSON 데이터 리스트
        """
        try:
            with open(self.log_file_path, 'r', encoding='utf-8') as file:
                self.data = json.load(file)
            return self.data
        except FileNotFoundError:
            print(f"Error: Log file not found at {self.log_file_path}")
            return []
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON format in {self.log_file_path}: {e}")
            return []
    
    def parse_ego_state(self, ego_data: Dict[str, Any], timestamp_us: int) -> EgoState:
        """
        ego 상태를 EgoState 객체로 변환
        입력:
            ego_data: 자차 상태 원시 데이터 dict
            timestamp_us: 타임스탬프 (microsec)
        반환값: EgoState 객체
        """
        
        return EgoState.build_from_rear_axle(
            rear_axle_pose=StateSE2(
                ego_data['x'] + self.MAP_ORIGIN_X,    # 맵 원점 기준으로 X 좌표 보정
                ego_data['y'] + self.MAP_ORIGIN_Y,    # 맵 원점 기준으로 Y 좌표 보정
                # 헤딩 각도를 ENU에서 유클리드 좌표계로 변환
                np.deg2rad((90 - np.rad2deg(float(ego_data['heading']))) % 360.0)
            ),
            rear_axle_velocity_2d=StateVector2D(        # 후축 기준 속도 벡터
                x=ego_data.get('v_x', 0.0),
                y=ego_data.get('v_y', 0.0)
            ),
            rear_axle_acceleration_2d=StateVector2D(    # 후축 기준 가속도 벡터
                x=ego_data.get('acc_x', 0.0),
                y=ego_data.get('acc_y', 0.0)
            ),
            tire_steering_angle=ego_data.get('steer_angle', 0.0),  # 타이어 조향각
            time_point=TimePoint(time_us=int(timestamp_us)),       # 시간 정보
            vehicle_parameters=get_pacifica_parameters(),          # 차량 param (Pacifica 기본값)
            is_in_auto_mode=True                                   # 자율주행 모드
        )
        
        
    def parse_traffic_light(self, ego_data: Dict[str, Any], map_api):
        """
        ego 위치 기반의 신호등 상태 정보
        입력:
            ego_data: ego 데이터 dict
            map_api: 맵 API 객체
        반환값: (신호등 상태, 차선 ID) tuple
        """
        
        x = ego_data.get('x', 0.0) + self.MAP_ORIGIN_X      # 맵 좌표계로 변환된 X 좌표
        y = ego_data.get('y', 0.0) + self.MAP_ORIGIN_Y      # 맵 좌표계로 변환된 Y 좌표
        tl_status = ego_data.get('vehicle_traffic_light', TrafficLightStatusType.UNKNOWN)  # 신호등 상태
        if tl_status == 255:  # 255 = UNKNOWN
            tl_status = TrafficLightStatusType.UNKNOWN
        ego_lane_id = int(map_api.get_nearest_lane((x, y))[0])  # ego와 가장 가까운 차선 ID
        
        return tl_status, ego_lane_id

    def parse_dynamic_agent(self, agent_data: Dict[str, Any], timestamp_us: Optional[int] = None) -> TrackedObject:
        """
        동적 에이전트 데이터를 TrackedObject로 변환
        입력:
            agent_data: 에이전트 raw 데이터 dict
            timestamp_us: 타임스탬프 (microsec)
        반환값: TrackedObject (Agent 또는 StaticObject)
        """
        # Create pose from position and heading
        pose = StateSE2(
            agent_data.get('x', 0.0) + self.MAP_ORIGIN_X,
            agent_data.get('y', 0.0) + self.MAP_ORIGIN_Y, 
            # agent_data.get('heading', 0.0)
            np.deg2rad(agent_data.get('heading', 0.0))
        )
        
        oriented_box = OrientedBox(
            pose, 
            width=agent_data.get('width', 0.0), 
            length=agent_data.get('length', 0.0), 
            height=agent_data.get('height', 0.0)
        )
        
        category = agent_data.get('category', 0)
        tracked_object_type = self.category_to_tracked_object_type.get(category, TrackedObjectType.VEHICLE)
        
        agent_id = agent_data.get('agent_id', 0)
        track_token = f"agent_{agent_id:08d}"
        
        metadata = SceneObjectMetadata(
            token=f"token_{agent_id:08d}_{timestamp_us or 0}",
            track_token=track_token,
            track_id=get_unique_incremental_track_id(track_token),
            timestamp_us=timestamp_us or 0,
            category_name=tracked_object_type.name.lower(),
        )
        
        if tracked_object_type in AGENT_TYPES:
            return Agent(
                tracked_object_type=tracked_object_type,
                oriented_box=oriented_box,
                velocity=StateVector2D(agent_data.get('v_x', 0.0), agent_data.get('v_y', 0.0)),
                predictions=[],  # Empty predictions list
                angular_velocity=0.0,  # Default angular velocity
                metadata=metadata,
            )
        else:
            return StaticObject(
                tracked_object_type=tracked_object_type,
                oriented_box=oriented_box,
                metadata=metadata,
            )
    
    def parse_log_entry(self, entry_data: Dict[str, Any], map_api) -> LogEntry:
        timestamp_us = entry_data.get('timestamp_us', 0)
        
        ego_state = self.parse_ego_state(entry_data.get('ego_state', {}), timestamp_us)

        traffic_light_status, ego_lane_id = self.parse_traffic_light(entry_data.get('ego_state', {}), map_api)
        
        dynamic_agents = []
        for agent_data in entry_data.get('dynamic_agents', []):
            dynamic_agent = self.parse_dynamic_agent(agent_data, timestamp_us)
            dynamic_agents.append(dynamic_agent)

        
        return LogEntry(
            timestamp_us=timestamp_us,
            ego_state=ego_state,
            dynamic_agents=dynamic_agents,
            traffic_light_status=TrafficLightStatusData(traffic_light_status, ego_lane_id, timestamp_us)
        )
    
    def get_parsed_entries(self, map_api = None) -> List[LogEntry]:
        if self.data is None:
            self.load()
        
        parsed_entries = []
        for entry_data in (self.data or []):
            parsed_entry = self.parse_log_entry(entry_data, map_api)
            parsed_entries.append(parsed_entry)
        
        return parsed_entries
    
    def get_ego_states(self, parsed_entries) -> List[EgoState]:
        return [entry.ego_state for entry in parsed_entries]
    
    def get_dynamic_agents(self, parsed_entries) -> List[List[TrackedObject]]:
        return [entry.dynamic_agents for entry in parsed_entries]
    
