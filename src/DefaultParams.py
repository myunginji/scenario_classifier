# 데이터 파일 경로 설정
LOG_FILE_NAME = "dataset/observation_log_20250307-094118.json"  # 입력 로그 파일 경로
OUTPUT_DIR_SCENARIO_FILE = "./labeled_scenarios"               # 라벨링된 시나리오 JSON 출력 디렉토리
OUTPUT_DIR_IMAGE_FILE = "./imgs"                              # 시각화 이미지 출력 디렉토리

# 처리 옵션 설정
MAX_SCENARIOS = -1  # 처리할 최대 시나리오 수 (-1 = 전체 처리, 양수 = 해당 개수만 처리)
VISUALIZE = True    # 시각화 이미지 생성 여부 (True = 생성, False = 생성 안함)
STEP_SIZE = 100     # 시나리오 생성 스텝 크기 (1 = 모든 시나리오, 큰 값 = 해당 간격으로 샘플링)

# 맵 설정
MAP_FILE_PATH = "src/map.sqlite"  # SQLite 맵 데이터베이스 파일 경로
MAP_ORIGIN_X = 230388.61912       # 맵 원점 X 좌표 (UTM 좌표계)
MAP_ORIGIN_Y = 424695.37128       # 맵 원점 Y 좌표 (UTM 좌표계)
