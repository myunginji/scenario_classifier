# 데이터 클래스 및 타입 힌트
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional

# NuPlan 프레임워크에서 사용되는 차량 상태 및 객체 추적 관련
from nuplan.common.actor_state.ego_state import EgoState
from nuplan.common.actor_state.tracked_objects import TrackedObject


@dataclass
class ScenarioLabel:
    """
    시나리오 라벨 클래스
    - 단일 시나리오에 대한 라벨과 메타데이터를 저장
    """
    label: str          # 라벨명 (예: "차선변경", "급정거" 등)
    confidence: float   # 라벨에 대한 신뢰도 (0.0 ~ 1.0)
    category: str       # 라벨 카테고리 (예: "안전", "위험" 등)
    description: str = ""  # 라벨에 대한 상세 설명


@dataclass
class ScenarioWindow:
    """
    101개 에폭 윈도우 데이터 구조
    - 중심 시점을 기준으로 과거 50개, 현재 1개, 미래 50개 프레임의 데이터를 포함
    """
    center_idx: int         # 중심 프레임의 인덱스
    center_timestamp: int   # 중심 프레임의 타임스탬프 (microsec)
    
    # Ego Vehicle 데이터
    ego_history: List[EgoState]    # 과거 ego 상태 리스트
    ego_current: EgoState          # 현재 ego 상태
    ego_future: List[EgoState]     # 미래 ego 상태 리스트
    
    # 주변 에이전트(차량, 보행자 등) 데이터  
    agents_history: List[List[TrackedObject]]  # 과거 에이전트 상태 리스트
    agents_current: List[TrackedObject]        # 현재 에이전트 상태 리스트
    agents_future: List[List[TrackedObject]]   # 미래 에이전트 상태 리스트
    
    # 신호등 상태 정보
    traffic_light_status: Any = None
    
    # 맵 컨텍스트 정보 (차선, 교차로 등)
    map_context: Dict[str, Any] = field(default_factory=dict)
    
    # 할당된 라벨들
    labels: List[ScenarioLabel] = field(default_factory=list)


@dataclass  
class LabeledScenario:
    """
    라벨링이 완료된 시나리오 클래스
    - 모든 라벨과 메타데이터가 포함된 완전한 시나리오 정보
    """
    scenario_id: str        # 시나리오 고유 ID
    center_idx: int         # 중심 프레임 인덱스
    center_timestamp: int   # 중심 프레임 타임스탬프
    
    # 현재 상태 요약 정보
    ego_position: Dict[str, float]  # ego 위치 (x, y, heading)
    ego_velocity: Dict[str, float]  # ego 속도 (vx, vy, magnitude)
    
    # 라벨 정보
    labels: List[str]                    # 라벨명 리스트
    label_details: List[Dict[str, Any]]  # 라벨 상세 정보 (신뢰도, 카테고리 포함)
    
    # 통계 정보
    num_agents: int       # 총 에이전트 수
    num_vehicles: int     # 차량 수
    num_pedestrians: int  # 보행자 수
    
    # 메타데이터
    confidence_mean: float  # 평균 신뢰도
    categories: List[str]   # 카테고리 리스트
