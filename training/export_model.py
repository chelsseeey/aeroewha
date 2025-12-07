#!/usr/bin/env python3
"""
학습된 모델을 다양한 형식으로 변환

사용법:
    python export_model.py --model runs/train/custom_model/weights/best.pt --format onnx
"""

import argparse
from pathlib import Path
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True, help='Trained model path (.pt)')
    parser.add_argument('--format', default='onnx', 
                       choices=['onnx', 'torchscript', 'tflite', 'openvino'],
                       help='Export format')
    parser.add_argument('--output', default='../models', help='Output directory')
    parser.add_argument('--imgsz', type=int, default=640, help='Image size')
    args = parser.parse_args()

    print("=" * 60)
    print("Exporting Model")
    print("=" * 60)
    
    # 모델 로드
    model = YOLO(args.model)
    print(f"Loaded model: {args.model}")
    
    # Export
    export_path = model.export(
        format=args.format,
        imgsz=args.imgsz,
    )
    
    print(f"\nExport complete")
    print(f"📁 Exported to: {export_path}")
    
    # 프로젝트 models/ 디렉토리로 복사
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    import shutil
    export_file = Path(export_path)
    dest = output_dir / export_file.name
    shutil.copy(export_file, dest)
    print(f"📋 Copied to: {dest}")
    
    print("\n" + "=" * 60)
    print("Done: Use this model in your drone project:")
    print(f"   detector = ObjectDetector(method='yolo', model_path='{dest}')")
    print("=" * 60)


if __name__ == '__main__':
    main()
