"""
JSON 로그 파일 로더 모듈

이 모듈은 JSON 형식의 자율주행 로그 파일을 읽고 NuPlan 프레임워크의
데이터 구조로 변환하는 기능을 제공합니다.

주요 기능:
- JSON 파일 로딩 및 파싱
- 좌표계 변환 (로컬 좌표 → 맵 원점 기준 UTM 좌표)
- EgoState 객체 생성
- TrackedObject 객체 생성 (차량, 보행자, 자전거 등)
- 신호등 상태 파싱

데이터 흐름:
    JSON 로그 파일
      └─> JsonLogLoader.load()
          └─> JsonLogLoader.get_parsed_entries()
              └─> List[LogEntry]
                  └─> ScenarioWindow 추출

참조:
    - doc/01_architecture.md: JsonLogLoader 역할 및 데이터 로딩 레이어 설명
    - doc/02_data_structures.md: LogEntry 데이터 구조 설명
"""

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
    JSON 형식의 로그 파일을 읽고 NuPlan 프레임워크의 데이터 구조로 변환하는 로더.
    
    JsonLogLoader는 데이터 로딩 레이어의 핵심 컴포넌트로, JSON 로그 파일을
    파싱하여 LogEntry 리스트로 변환합니다. 각 LogEntry는 단일 타임스탬프의
    전체 상황 정보(ego 상태, 주변 객체, 신호등 상태)를 포함합니다.
    
    주요 기능:
    1. JSON 파일 로딩 및 파싱
    2. 좌표계 변환 (로컬 좌표 → UTM 좌표)
    3. EgoState 객체 생성
    4. TrackedObject 객체 생성
    5. 신호등 상태 파싱
    
    좌표계 변환:
        JSON 로그의 좌표는 로컬 좌표계(상대 좌표)를 사용합니다.
        이를 UTM 좌표계로 변환하기 위해 맵 원점(MAP_ORIGIN_X, MAP_ORIGIN_Y)을 사용합니다.
        
        변환 공식:
            UTM_x = local_x + MAP_ORIGIN_X
            UTM_y = local_y + MAP_ORIGIN_Y
    
    입력:
        JSON 로그 파일 형식:
        [
            {
                "timestamp_us": int,
                "ego_state": {
                    "x": float, "y": float, "heading": float,
                    "v_x": float, "v_y": float,
                    "acc_x": float, "acc_y": float,
                    "steer_angle": float,
                    "vehicle_traffic_light": int
                },
                "dynamic_agents": [
                    {
                        "agent_id": int,
                        "category": int,  # 0=차량, 1=보행자, 2=자전거, ...
                        "x": float, "y": float, "heading": float,
                        "width": float, "length": float, "height": float,
                        "v_x": float, "v_y": float
                    },
                    ...
                ]
            },
            ...
        ]
    
    출력:
        List[LogEntry]: 파싱된 로그 엔트리 리스트
    
    참조:
        - doc/01_architecture.md: JsonLogLoader 역할 및 데이터 로딩 레이어
        - doc/02_data_structures.md: LogEntry 데이터 구조
    """
    
    def __init__(self, log_file_path: str, mapOrigin):
        """
        JsonLogLoader 초기화.
        
        Args:
            log_file_path (str): JSON 로그 파일 경로
                DefaultParams.LOG_FILE_NAME에서 지정된 경로를 사용합니다.
            
            mapOrigin (List[float] or Tuple[float, float]): 맵 원점 좌표 [x, y]
                UTM 좌표계의 맵 원점 좌표입니다.
                DefaultParams.MAP_ORIGIN_X, MAP_ORIGIN_Y 값을 사용합니다.
                로컬 좌표를 UTM 좌표로 변환할 때 이 값을 더합니다.
        
        Attributes:
            log_file_path (str): 로그 파일 경로
            data (List[Dict[str, Any]]): 로드된 JSON 데이터 (lazy loading)
            MAP_ORIGIN_X (float): 맵 원점 X 좌표 (UTM, 미터)
            MAP_ORIGIN_Y (float): 맵 원점 Y 좌표 (UTM, 미터)
            category_to_tracked_object_type (Dict[int, TrackedObjectType]): 
                카테고리 번호를 TrackedObjectType으로 매핑하는 딕셔너리
        """
        self.log_file_path = log_file_path    # 로그 파일 경로
        self.data = None                      # 로드된 JSON 데이터 (lazy loading)
        self.MAP_ORIGIN_X = float(mapOrigin[0])  # 맵 원점 X 좌표 (UTM, 미터)
        self.MAP_ORIGIN_Y = float(mapOrigin[1])  # 맵 원점 Y 좌표 (UTM, 미터)
        
        # 카테고리 번호를 TrackedObjectType으로 매핑하는 딕셔너리
        # JSON 로그의 category 필드 값에 따라 객체 타입을 결정합니다.
        self.category_to_tracked_object_type = {
            0: TrackedObjectType.VEHICLE,      # 차량
            1: TrackedObjectType.PEDESTRIAN,   # 보행자
            2: TrackedObjectType.BICYCLE,      # 자전거
            3: TrackedObjectType.TRAFFIC_CONE, # 콘 (정적 객체)
            4: TrackedObjectType.BARRIER,      # 장벽 (정적 객체)
            5: TrackedObjectType.CZONE_SIGN,   # 공사구역 표지판 (정적 객체)
            6: TrackedObjectType.GENERIC_OBJECT, # 기타 객체
        }
    
    def load(self) -> List[Dict[str, Any]]:
        """
        JSON 로그 파일에서 데이터를 로드합니다.
        
        파일을 읽어 JSON 형식으로 파싱하고, 내부 데이터 캐시에 저장합니다.
        한 번 로드된 데이터는 self.data에 캐시되어 재사용됩니다.
        
        Returns:
            List[Dict[str, Any]]: 로드된 JSON 데이터 리스트
                각 요소는 하나의 로그 엔트리를 나타냅니다.
                파일이 없거나 파싱 실패 시 빈 리스트를 반환합니다.
        
        Raises:
            FileNotFoundError: 로그 파일이 존재하지 않는 경우 (에러 메시지 출력 후 빈 리스트 반환)
            json.JSONDecodeError: JSON 형식이 잘못된 경우 (에러 메시지 출력 후 빈 리스트 반환)
        
        참조:
            - doc/01_architecture.md: 데이터 로딩 레이어 설명
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
        Ego 차량 상태를 EgoState 객체로 변환합니다.
        
        JSON 로그의 ego_state 데이터를 NuPlan 프레임워크의 EgoState 객체로 변환합니다.
        좌표계 변환과 헤딩 각도 변환을 수행합니다.
        
        좌표계 변환:
            - 입력: 로컬 좌표계 (JSON 로그의 x, y)
            - 출력: UTM 좌표계 (시스템 표준)
            - 변환: UTM_x = local_x + MAP_ORIGIN_X, UTM_y = local_y + MAP_ORIGIN_Y
        
        헤딩 각도 변환:
            - JSON 로그의 헤딩은 ENU 좌표계 기준일 수 있습니다.
            - NuPlan은 유클리드 좌표계를 사용하므로 변환이 필요합니다.
            - 변환 공식: heading_rad = deg2rad((90 - rad2deg(heading_deg)) % 360.0)
        
        Args:
            ego_data (Dict[str, Any]): Ego 차량 상태 원시 데이터
                필수 필드:
                    - 'x' (float): 로컬 X 좌표 (미터)
                    - 'y' (float): 로컬 Y 좌표 (미터)
                    - 'heading' (float): 헤딩 각도 (라디안 또는 도)
                선택 필드:
                    - 'v_x' (float): X 방향 속도 (m/s, 기본값: 0.0)
                    - 'v_y' (float): Y 방향 속도 (m/s, 기본값: 0.0)
                    - 'acc_x' (float): X 방향 가속도 (m/s², 기본값: 0.0)
                    - 'acc_y' (float): Y 방향 가속도 (m/s², 기본값: 0.0)
                    - 'steer_angle' (float): 타이어 조향각 (라디안, 기본값: 0.0)
            
            timestamp_us (int): 타임스탬프 (마이크로초)
                Unix epoch 기준 마이크로초 단위입니다.
        
        Returns:
            EgoState: NuPlan 프레임워크의 EgoState 객체
                다음 정보를 포함:
                - rear_axle: 후축 위치 및 헤딩 (UTM 좌표계)
                - dynamic_car_state: 속도 및 가속도 벡터
                - car_footprint: 차량 외곽선 (Shapely Polygon)
                - timestamp_us: 타임스탬프
        
        참조:
            - doc/01_architecture.md: 좌표 변환 설명
            - doc/02_data_structures.md: EgoState 구조 설명
        """
        
        return EgoState.build_from_rear_axle(
            rear_axle_pose=StateSE2(
                ego_data['x'] + self.MAP_ORIGIN_X,    # 로컬 좌표 → UTM 좌표 변환 (X)
                ego_data['y'] + self.MAP_ORIGIN_Y,    # 로컬 좌표 → UTM 좌표 변환 (Y)
                # 헤딩 각도를 ENU에서 유클리드 좌표계로 변환
                # ENU: 0도=북쪽, 시계방향이 양수
                # 유클리드: 0도=동쪽, 반시계방향이 양수
                np.deg2rad((90 - np.rad2deg(float(ego_data['heading']))) % 360.0)
            ),
            rear_axle_velocity_2d=StateVector2D(        # 후축 기준 속도 벡터 (m/s)
                x=ego_data.get('v_x', 0.0),
                y=ego_data.get('v_y', 0.0)
            ),
            rear_axle_acceleration_2d=StateVector2D(    # 후축 기준 가속도 벡터 (m/s²)
                x=ego_data.get('acc_x', 0.0),
                y=ego_data.get('acc_y', 0.0)
            ),
            tire_steering_angle=ego_data.get('steer_angle', 0.0),  # 타이어 조향각 (라디안)
            time_point=TimePoint(time_us=int(timestamp_us)),       # 시간 정보 (microsec)
            vehicle_parameters=get_pacifica_parameters(),          # 차량 파라미터 (Pacifica 기본값)
            is_in_auto_mode=True                                   # 자율주행 모드 활성화
        )
        
        
    def parse_traffic_light(self, ego_data: Dict[str, Any], map_api):
        """
        Ego 차량 위치 기반의 신호등 상태 정보를 파싱합니다.
        
        JSON 로그에서 신호등 상태를 읽고, 맵 API를 사용하여
        Ego 차량이 위치한 차선의 ID를 조회합니다.
        
        Args:
            ego_data (Dict[str, Any]): Ego 차량 데이터
                필수 필드:
                    - 'x' (float): 로컬 X 좌표
                    - 'y' (float): 로컬 Y 좌표
                선택 필드:
                    - 'vehicle_traffic_light' (int): 신호등 상태 값
                        - 0: GO_STRAIGHT
                        - 1: GO_STRAIGHT_AND_TURNLEFT
                        - 2: STOP
                        - 3: STOP_AND_TURNLEFT
                        - 4: STOP_AND_WARNING
                        - 5: WARNING
                        - 255 또는 없음: UNKNOWN
            
            map_api: 맵 API 객체 (MapManager)
                get_nearest_lane() 메서드를 사용하여 가장 가까운 차선 ID를 조회합니다.
        
        Returns:
            Tuple[TrafficLightStatusType, int]: (신호등 상태, 차선 연결부 ID)
                - TrafficLightStatusType: 신호등 상태 열거형
                - int: Ego 차량과 가장 가까운 차선 연결부(LaneConnector) ID
        
        참조:
            - TrafficLightDataTypes: 신호등 상태 타입 정의
            - doc/02_data_structures.md: TrafficLightStatusData 설명
        """
        
        # 로컬 좌표를 UTM 좌표로 변환
        x = ego_data.get('x', 0.0) + self.MAP_ORIGIN_X      # UTM X 좌표
        y = ego_data.get('y', 0.0) + self.MAP_ORIGIN_Y      # UTM Y 좌표
        
        # 신호등 상태 파싱
        tl_status = ego_data.get('vehicle_traffic_light', TrafficLightStatusType.UNKNOWN)
        if tl_status == 255:  # 255는 UNKNOWN을 나타냄
            tl_status = TrafficLightStatusType.UNKNOWN
        
        # 맵 API를 사용하여 가장 가까운 차선 ID 조회
        ego_lane_id = int(map_api.get_nearest_lane((x, y))[0])  # 차선 연결부 ID
        
        return tl_status, ego_lane_id

    def parse_dynamic_agent(self, agent_data: Dict[str, Any], timestamp_us: Optional[int] = None) -> TrackedObject:
        """
        동적 에이전트 데이터를 TrackedObject로 변환합니다.
        
        JSON 로그의 dynamic_agents 항목을 NuPlan 프레임워크의 TrackedObject로 변환합니다.
        객체 타입(차량, 보행자, 자전거 등)에 따라 Agent 또는 StaticObject를 생성합니다.
        
        좌표계 변환:
            - 로컬 좌표 → UTM 좌표 (MAP_ORIGIN_X, MAP_ORIGIN_Y 사용)
        
        Args:
            agent_data (Dict[str, Any]): 에이전트 원시 데이터
                필수 필드:
                    - 'agent_id' (int): 에이전트 고유 ID
                    - 'category' (int): 객체 카테고리 (0=차량, 1=보행자, 2=자전거, ...)
                    - 'x' (float): 로컬 X 좌표 (미터)
                    - 'y' (float): 로컬 Y 좌표 (미터)
                    - 'heading' (float): 헤딩 각도 (라디안 또는 도)
                    - 'width' (float): 바운딩 박스 너비 (미터)
                    - 'length' (float): 바운딩 박스 길이 (미터)
                    - 'height' (float): 바운딩 박스 높이 (미터)
                선택 필드 (Agent인 경우):
                    - 'v_x' (float): X 방향 속도 (m/s, 기본값: 0.0)
                    - 'v_y' (float): Y 방향 속도 (m/s, 기본값: 0.0)
            
            timestamp_us (Optional[int]): 타임스탬프 (마이크로초)
                None인 경우 0을 사용합니다.
        
        Returns:
            TrackedObject: NuPlan 프레임워크의 TrackedObject
                - Agent: 동적 객체 (차량, 보행자, 자전거) - 속도 정보 포함
                - StaticObject: 정적 객체 (콘, 장벽, 표지판) - 속도 정보 없음
        
        객체 타입 매핑:
            - category 0 → VEHICLE (차량)
            - category 1 → PEDESTRIAN (보행자)
            - category 2 → BICYCLE (자전거)
            - category 3 → TRAFFIC_CONE (콘)
            - category 4 → BARRIER (장벽)
            - category 5 → CZONE_SIGN (공사구역 표지판)
            - category 6 → GENERIC_OBJECT (기타)
            - 기타 → VEHICLE (기본값)
        
        참조:
            - doc/02_data_structures.md: TrackedObject 구조 설명
        """
        # 위치 및 헤딩으로부터 pose 생성 (좌표계 변환 포함)
        pose = StateSE2(
            agent_data.get('x', 0.0) + self.MAP_ORIGIN_X,  # 로컬 → UTM 좌표 변환 (X)
            agent_data.get('y', 0.0) + self.MAP_ORIGIN_Y,  # 로컬 → UTM 좌표 변환 (Y)
            np.deg2rad(agent_data.get('heading', 0.0))     # 헤딩 각도 (라디안으로 변환)
        )
        
        # 바운딩 박스 생성
        oriented_box = OrientedBox(
            pose, 
            width=agent_data.get('width', 0.0),   # 너비 (미터)
            length=agent_data.get('length', 0.0), # 길이 (미터)
            height=agent_data.get('height', 0.0)  # 높이 (미터)
        )
        
        # 카테고리 번호로부터 객체 타입 결정
        category = agent_data.get('category', 0)
        tracked_object_type = self.category_to_tracked_object_type.get(category, TrackedObjectType.VEHICLE)
        
        # 에이전트 ID 및 추적 토큰 생성
        agent_id = agent_data.get('agent_id', 0)
        track_token = f"agent_{agent_id:08d}"  # 고유 추적 토큰
        
        # 메타데이터 생성
        metadata = SceneObjectMetadata(
            token=f"token_{agent_id:08d}_{timestamp_us or 0}",
            track_token=track_token,
            track_id=get_unique_incremental_track_id(track_token),
            timestamp_us=timestamp_us or 0,
            category_name=tracked_object_type.name.lower(),
        )
        
        # 객체 타입에 따라 Agent 또는 StaticObject 생성
        if tracked_object_type in AGENT_TYPES:
            # 동적 객체 (차량, 보행자, 자전거) - 속도 정보 포함
            return Agent(
                tracked_object_type=tracked_object_type,
                oriented_box=oriented_box,
                velocity=StateVector2D(agent_data.get('v_x', 0.0), agent_data.get('v_y', 0.0)),
                predictions=[],  # 예측 정보는 비어있음 (향후 확장용)
                angular_velocity=0.0,  # 기본 각속도 (0.0)
                metadata=metadata,
            )
        else:
            # 정적 객체 (콘, 장벽, 표지판) - 속도 정보 없음
            return StaticObject(
                tracked_object_type=tracked_object_type,
                oriented_box=oriented_box,
                metadata=metadata,
            )
    
    def parse_log_entry(self, entry_data: Dict[str, Any], map_api) -> LogEntry:
        """
        단일 로그 엔트리를 LogEntry 객체로 변환합니다.
        
        JSON 로그의 하나의 엔트리(타임스탬프)를 파싱하여 LogEntry를 생성합니다.
        Ego 상태, 주변 객체, 신호등 상태를 모두 파싱하여 통합합니다.
        
        Args:
            entry_data (Dict[str, Any]): JSON 로그의 단일 엔트리
                {
                    "timestamp_us": int,
                    "ego_state": {...},
                    "dynamic_agents": [...]
                }
            
            map_api: 맵 API 객체 (MapManager)
                신호등 파싱 시 가장 가까운 차선 ID를 조회하는 데 사용됩니다.
        
        Returns:
            LogEntry: 파싱된 로그 엔트리
                - timestamp_us: 타임스탬프
                - ego_state: Ego 차량 상태
                - dynamic_agents: 주변 객체 리스트
                - traffic_light_status: 신호등 상태 정보
        
        참조:
            - doc/02_data_structures.md: LogEntry 데이터 구조 설명
        """
        # 타임스탬프 추출
        timestamp_us = entry_data.get('timestamp_us', 0)
        
        # Ego 상태 파싱
        ego_state = self.parse_ego_state(entry_data.get('ego_state', {}), timestamp_us)

        # 신호등 상태 파싱 (맵 API 사용)
        traffic_light_status, ego_lane_id = self.parse_traffic_light(entry_data.get('ego_state', {}), map_api)
        
        # 주변 객체 파싱
        dynamic_agents = []
        for agent_data in entry_data.get('dynamic_agents', []):
            dynamic_agent = self.parse_dynamic_agent(agent_data, timestamp_us)
            dynamic_agents.append(dynamic_agent)

        # LogEntry 생성 및 반환
        return LogEntry(
            timestamp_us=timestamp_us,
            ego_state=ego_state,
            dynamic_agents=dynamic_agents,
            traffic_light_status=TrafficLightStatusData(traffic_light_status, ego_lane_id, timestamp_us)
        )
    
    def get_parsed_entries(self, map_api = None) -> List[LogEntry]:
        """
        전체 JSON 로그를 파싱하여 LogEntry 리스트로 반환합니다.
        
        JSON 파일을 로드하고, 각 엔트리를 파싱하여 LogEntry 리스트를 생성합니다.
        이 메서드는 데이터 로딩 레이어의 주요 진입점입니다.
        
        Args:
            map_api: 맵 API 객체 (MapManager, 선택적)
                신호등 파싱 시 사용됩니다. None인 경우 신호등 파싱이 실패할 수 있습니다.
        
        Returns:
            List[LogEntry]: 파싱된 로그 엔트리 리스트
                각 LogEntry는 단일 타임스탬프의 전체 상황 정보를 포함합니다.
                시간 순서대로 정렬되어 있습니다.
        
        데이터 흐름:
            JSON 파일
              └─> load() (lazy loading)
                  └─> parse_log_entry() (각 엔트리)
                      └─> List[LogEntry]
        
        참조:
            - doc/01_architecture.md: 데이터 로딩 레이어 설명
        """
        # JSON 파일이 아직 로드되지 않은 경우 로드
        if self.data is None:
            self.load()
        
        # 각 엔트리를 파싱하여 LogEntry 리스트 생성
        parsed_entries = []
        for entry_data in (self.data or []):
            parsed_entry = self.parse_log_entry(entry_data, map_api)
            parsed_entries.append(parsed_entry)
        
        return parsed_entries
    
    def get_ego_states(self, parsed_entries) -> List[EgoState]:
        """
        LogEntry 리스트에서 EgoState 리스트를 추출합니다.
        
        Args:
            parsed_entries (List[LogEntry]): 파싱된 로그 엔트리 리스트
        
        Returns:
            List[EgoState]: Ego 차량 상태 리스트
                각 LogEntry의 ego_state를 추출한 리스트입니다.
        """
        return [entry.ego_state for entry in parsed_entries]
    
    def get_dynamic_agents(self, parsed_entries) -> List[List[TrackedObject]]:
        """
        LogEntry 리스트에서 주변 객체 리스트를 추출합니다.
        
        Args:
            parsed_entries (List[LogEntry]): 파싱된 로그 엔트리 리스트
        
        Returns:
            List[List[TrackedObject]]: 주변 객체 리스트의 리스트
                각 LogEntry의 dynamic_agents를 추출한 리스트입니다.
                외부 리스트는 시간 순서, 내부 리스트는 각 시점의 객체들입니다.
        """
        return [entry.dynamic_agents for entry in parsed_entries]
    
