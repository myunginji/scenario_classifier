from dataclasses import dataclass
from typing import List

from TrafficLightDataTypes import TrafficLightStatusData
from nuplan.common.actor_state.ego_state import EgoState
from nuplan.common.actor_state.tracked_objects import TrackedObject

@dataclass
class LogEntry:
    """
    - 특정 시점의 전체 상황 정보를 담는 데이터 구조
    """
    timestamp_us: int                            # 타임스탬프 (microsec)
    ego_state: EgoState                          # ego 상태 정보 (위치, 속도, 가속도 등)
    dynamic_agents: List[TrackedObject]          # 주변 동적 객체들 (차량, 보행자 등)
    traffic_light_status: TrafficLightStatusData # 신호등 상태 정보
