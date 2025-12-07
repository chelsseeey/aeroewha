# YOLOv11 마커 검출 모델 학습 

드론 자율 추적 시스템을 위한 커스텀 YOLO 모델 학습 

## 📁 디렉토리 구조

```
training/
├── dataset/
│   ├── data.yaml          # 데이터셋 설정 
│   ├── train/             # 학습 데이터 (300장) 
│   │   ├── images/
│   │   └── labels/
│   ├── valid/             # 검증 데이터 (30장) 
│   │   ├── images/
│   │   └── labels/
│   └── test/              # 테스트 데이터 (15장) 
│       ├── images/
│       └── labels/
├── train.py               # 학습 스크립트 (YOLOv11m)
├── export_model.py        # 모델 변환 (ONNX/TFLite)
├── extract_frames.py      # 영상→이미지 추출
└── requirements.txt       # 의존성
```

## 빠른 시작 (Quick Start)

### 1. 환경 설정

#### macOS (Apple Silicon - M1/M2/M3)

```bash
pip install ultralytics
```

#### Linux / Windows

```bash
pip install ultralytics torch torchvision
```

### 2. 데이터 준비

- 학습: 300장
- 검증: 30장
- 테스트: 15장
- 라벨: YOLO 형식 (자동 생성)

### 3. 학습 시작

```bash
# YOLOv11m 모델 학습 (GPU 가속)
python train.py --epochs 100 --batch 16 --name marker_yolov11m

# 또는 빠른 테스트 (10 에포크)
python train.py --epochs 10 --batch 8 --name marker_test
```

### 4️⃣ 학습 완료 후

```bash
# 모델 확인
ls -la runs/train/marker_yolov11m/weights/best.pt

# 모델 복사
cp runs/train/marker_yolov11m/weights/best.pt ../models/best.pt

# 검증
python -c "from ultralytics import YOLO; model = YOLO('../models/best.pt'); model.val()"
```

---

## 학습 설정 상세

### 모델: YOLOv11m (Medium - Accurate)

```python
# train.py 설정
model = YOLO('yolov11m.pt')  # Medium 모델 (정확도 우선)

# 다른 옵션:
# yolov11n.pt  - Nano (가장 빠름, 낮은 정확도)
# yolov11s.pt  - Small (빠름)
# yolov11m.pt  - Medium (균형)
# yolov11l.pt  - Large (느림, 높은 정확도)
# yolov11x.pt  - Extra Large (가장 느림, 최고 정확도)
```

### 증강(Augmentation) 설정

```python
# 3가지 증강 기법 적용
flipud=0.5,        # 상하 반전 (50%)
fliplr=0.5,        # 좌우 반전 (50%)
degrees=15.0,      # 회전 (-15° ~ +15°)
translate=0.1,     # 이동 (10%)
scale=0.5,         # 크기 (50% ~ 150%)
hsv_h=0.015,       # 색조 변화
hsv_s=0.7,         # 채도 변화
hsv_v=0.4,         # 명도 변화
mosaic=1.0,        # 모자이크 증강
```

### GPU 가속

```bash
# macOS (Apple Silicon)
--device mps         

# NVIDIA GPU
--device 0           # CUDA GPU 0번

# CPU
--device cpu         # CPU만 사용 (느림)
```

---

## 데이터 준비 (새로운 데이터셋)

### 1. 이미지 수집

```bash
# 영상에서 프레임 추출
python extract_frames.py --video flight_video.mp4 --output dataset/images/train --fps 2

# 권장 데이터 양:
# - 최소: 100-200장
# - 권장: 300-500장
# - 최적: 1000+장
```

### 2. 라벨링

### 3. 데이터셋 설정 (data.yaml)

```yaml
# dataset/data.yaml
train: ../train/images
val: ../valid/images
test: ../test/images

nc: 1
names: ['marker']
```

---
