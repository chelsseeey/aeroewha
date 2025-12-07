#!/usr/bin/env python3
"""
영상에서 프레임 추출 (데이터셋 준비용)

사용법:
    python extract_frames.py --video flight.mp4 --output dataset/images/train --interval 10
"""

import argparse
import cv2
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--video', required=True, help='Input video path')
    parser.add_argument('--output', default='dataset/images/train', help='Output directory')
    parser.add_argument('--interval', type=int, default=30, 
                       help='Extract every N frames (30 = 1fps at 30fps video)')
    parser.add_argument('--max-frames', type=int, default=1000, help='Maximum frames to extract')
    args = parser.parse_args()

    # 출력 디렉토리 생성
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 영상 열기
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"Error: Cannot open video {args.video}")
        return
    
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    print(f"📹 Video info:")
    print(f"   Total frames: {total_frames}")
    print(f"   FPS: {fps}")
    print(f"   Extract every {args.interval} frames")
    print(f"   Estimated output: ~{min(total_frames // args.interval, args.max_frames)} images")
    print()
    
    frame_idx = 0
    extracted = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # interval마다 저장
        if frame_idx % args.interval == 0:
            output_path = output_dir / f"frame_{extracted:06d}.jpg"
            cv2.imwrite(str(output_path), frame)
            extracted += 1
            
            if extracted % 10 == 0:
                print(f"Extracted {extracted} frames...")
            
            if extracted >= args.max_frames:
                print(f"Reached max frames limit ({args.max_frames})")
                break
        
        frame_idx += 1
    
    cap.release()
    
    print()
    print("=" * 60)
    print(f"Extraction complete!")
    print(f"Output: {output_dir}")
    print(f"Total frames extracted: {extracted}")
    print()
    print("Next steps:")
    print("1. Label these images using Roboflow or Label Studio")
    print("2. Run: python train.py --epochs 100")
    print("=" * 60)


if __name__ == '__main__':
    main()
