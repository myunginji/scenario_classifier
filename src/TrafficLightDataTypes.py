from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Dict, List


class TrafficLightStatusType(IntEnum):
    # 확장된 신호등 상태 정의
    UNKNOWN = 6                     # unknown
    GO_STRAIGHT = 0                 # 직진 가능
    GO_STRAIGHT_AND_TURNLEFT = 1    # 직진 및 좌회전 가능
    STOP = 2                        # 정지
    STOP_AND_TURNLEFT = 3           # 정지 및 좌회전 가능
    STOP_AND_WARNING = 4            # 정지 및 경고
    WARNING = 5                     # 경고

    def serialize(self) -> str:
        return self.name

    @classmethod
    def deserialize(cls, key: str) -> TrafficLightStatusType:
        return TrafficLightStatusType.__members__[key]
    

@dataclass
class TrafficLightStatusData:

    status: TrafficLightStatusType  # 신호등 상태 (GO_STRAIGHT, STOP 등)
    lane_connector_id: int          # 신호등이 속한 차선 연결부 ID
    timestamp: int                  # 타임스탬프 (마이크로초 단위)

    def serialize(self) -> Dict[str, Any]:
        return {
            'status': self.status.serialize(),      # 상태를 문자열로 직렬화
            'lane_connector_id': self.lane_connector_id,
            'timestamp': self.timestamp,
        }

    @classmethod
    def deserialize(cls, data: Dict[str, Any]) -> TrafficLightStatusData:
        return TrafficLightStatusData(
            status=TrafficLightStatusType.deserialize(data['status']),  # 문자열을 상태 타입으로 변환
            lane_connector_id=data['lane_connector_id'],
            timestamp=data['timestamp'],
        )


@dataclass
class TrafficLightStatuses:
    traffic_lights: List[TrafficLightStatusData]  # 신호등 상태 데이터 리스트
