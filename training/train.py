#!/usr/bin/env python3
"""
커스텀 YOLO 모델 학습 스크립트

사용법:
    python train.py --epochs 100 --batch 16
"""

import argparse
from pathlib import Path
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='yolov11m.pt', help='Base model (YOLOv11 Medium - Accurate)')
    parser.add_argument('--data', default='dataset/data.yaml', help='Dataset config')
    parser.add_argument('--epochs', type=int, default=100, help='Training epochs')
    parser.add_argument('--batch', type=int, default=16, help='Batch size')
    parser.add_argument('--imgsz', type=int, default=640, help='Image size')
    parser.add_argument('--device', default='mps', help='Device: mps (Apple Silicon GPU), cpu, or 0 (CUDA)')
    parser.add_argument('--project', default='runs/train', help='Save results to project/name')
    parser.add_argument('--name', default='custom_model', help='Save results to project/name')
    args = parser.parse_args()

    print("=" * 60)
    print("Starting Custom Model Training")
    print("=" * 60)
    
    # 모델 로드 (사전학습 모델 사용)
    model = YOLO(args.model)
    print(f"Loaded base model: {args.model}")
    
    # 학습 시작 (증강 포함)
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        patience=20,  # Early stopping
        save=True,
        plots=True,
        # ========== 증강(Augmentation) 설정 ==========
        flipud=0.5,        # 1️⃣ 상하 반전 (50% 확률)
        fliplr=0.5,        # 1️⃣ 좌우 반전 (50% 확률)
        degrees=15.0,      # 2️⃣ 회전 (-15° ~ +15°)
        translate=0.1,     # 2️⃣ 이동 (10% 범위)
        scale=0.5,         # 3️⃣ 크기 조정 (50% ~ 150%)
        hsv_h=0.015,       # 3️⃣ 색조 변화
        hsv_s=0.7,         # 3️⃣ 채도 변화
        hsv_v=0.4,         # 3️⃣ 명도 변화
        mosaic=1.0,        # 모자이크 증강 (드론 시뮬레이션)
    )
    
    print("\n" + "=" * 60)
    print("Training Complete")
    print(f"Results saved to: {Path(args.project) / args.name}")
    print(f"Best model: {Path(args.project) / args.name / 'weights' / 'best.pt'}")
    print("=" * 60)
    
    # 검증
    print("\n🔍 Running validation...")
    metrics = model.val()
    print(f"mAP50: {metrics.box.map50:.3f}")
    print(f"mAP50-95: {metrics.box.map:.3f}")


if __name__ == '__main__':
    main()
