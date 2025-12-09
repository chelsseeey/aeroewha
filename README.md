# 드론 자율 비행 제어 시스템

MAVSDK와 컴퓨터 비전(YOLO)을 활용한 모듈식 드론 자율 추적 및 미션 실행 시스템

## 프로젝트 구조

```
drone_project/
├── configs/          # 설정 파일 (YAML)
│   ├── camera.yaml   # 카메라 및 YOLO 설정
│   ├── control.yaml  # PID 제어 설정
│   └── system.yaml   # 시스템 및 미션 설정
├── vision/           # 비전 모듈 (카메라, 검출, 추적)
│   ├── stream.py     # 영상 입력
│   ├── detector.py   # YOLO 물체 검출
│   └── tracker.py    # 물체 추적
├── control/          # 제어 모듈 (PID, Offboard)
├── mission/          # 미션 상태 머신 (탐색/추적/복귀)
├── platform/         # 하드웨어 인터페이스 (MAVSDK, 안전)
├── common/           # 공통 타입 및 유틸리티
├── utils/            # 시각화 및 모니터링
├── training/         # 모델 학습 (YOLO 커스텀 학습)
│   ├── dataset/      # 학습 데이터
│   ├── train.py      # 학습 스크립트
│   └── QUICKSTART.md # 학습 가이드
├── models/           # 학습된 YOLO 모델 저장
├── logs/             # 실행 로그 및 녹화
├── tests/            # 유닛 테스트
├── deploy/           # 배포 스크립트
├── main.py           # 메인 실행 파일
└── test_vision.py    # 비전 시스템 테스트
```

## 주요 기능

### 비전 시스템
- **YOLO 기반 물체 검출**: 학습된 모델로 실시간 물체 인식
- **다중 추적 알고리즘**: KCF/CSRT/MOSSE + Kalman 필터 스무딩
- **다양한 입력 소스**: USB/CSI 카메라, 영상 파일, RTSP 스트림

### 제어 시스템
- **PID 제어**: 위치/속도/Yaw 제어
- **Offboard 모드**: 자동 명령 전송 (20Hz)
- **속도/고도 제한**: 안전 범위 내 제어

### 미션 관리
- **상태 머신**: IDLE → TAKEOFF → SEARCH → TRACK → RTL
- **자동 탐색**: 나선형/그리드 패턴
- **타겟 추적**: 검출된 물체 자동 추적
- **로스트 복구**: 타겟 소실 시 재탐색

### 안전 시스템
- **GeoFence**: 비행 영역 제한
- **배터리 모니터링**: 저전압 자동 복귀
- **고도 제한**: 최소/최대 고도 제한
- **비상 정지**: 긴급 상황 대응

### 데이터 기록
- **CSV 로그**: 텔레메트리, 검출 결과, 제어 명령
- **이벤트 로그**: JSON 형식 상태 변화 기록

## 빠른 시작

### 1. 환경 설정

```bash
# 의존성 설치
pip install -r requirements.txt

# 또는 자동 설치 스크립트
./deploy/scripts/setup.sh
```

### 2. 비전 시스템 테스트

```bash
# 카메라 + YOLO 검출 테스트
python test_vision.py

```

### 3. 설정 파일 수정

#### `configs/camera.yaml` - 카메라 및 YOLO 설정
```yaml
camera:
  source_type: "USB"  # 또는 "FILE", "RTSP"
  device_id: 0        # 카메라 번호

detection:
  method: "yolo"      # YOLO 사용
  yolo:
    model_path: "models/best.pt"  # 학습한 모델 경로
    conf_threshold: 0.5            # 신뢰도 임계값
```

#### `configs/system.yaml` - 시스템 설정
```yaml
system:
  mode: "SITL"  # 시뮬레이션 모드 (실제 드론은 "REAL")
```

#### `configs/control.yaml` - PID 제어 설정
```yaml
controller:
  position:
    p_gain: 0.5  # 비례 게인
    i_gain: 0.01 # 적분 게인
    d_gain: 0.1  # 미분 게인
```

### 4. 시스템 실행

```bash
# 직접 실행
python main.py

# 또는 스크립트 사용
./deploy/scripts/start_drone.sh
```

## 테스트

### 비전 시스템 테스트
```bash
# 카메라 + YOLO 검출 실시간 테스트
python test_vision.py
```

### 유닛 테스트
```bash
# 전체 테스트
python -m pytest tests/

# 특정 테스트
python -m pytest tests/test_controller.py      # 제어기
python -m pytest tests/test_state_machine.py   # 상태 머신
```

### SITL 시뮬레이션 테스트
```bash
# 1. PX4 SITL 실행 (별도 터미널)
cd ~/PX4-Autopilot
make px4_sitl gazebo

# 2. 드론 시스템 실행
python main.py
```

## 배포

### Jetson/라즈베리파이 배포

```bash
# 1. 프로젝트 복사
scp -r drone_project/ jetson@192.168.1.100:~/

# 2. Jetson에서 설치
ssh jetson@192.168.1.100
cd ~/drone_project
pip install -r requirements.txt

# 3. 자동 시작 설정 (systemd)
sudo cp deploy/systemd/drone.service /etc/systemd/system/
sudo systemctl enable drone.service
sudo systemctl start drone.service

# 4. 로그 확인
sudo journalctl -u drone.service -f
```

### Docker 배포

```bash
cd deploy/docker
docker-compose up -d

# 로그 확인
docker-compose logs -f
```


## 라이선스

MIT License
