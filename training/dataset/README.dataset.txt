# YOLOv11 Marker Detection Dataset
# Downloaded from Roboflow

Dataset Structure:
- train/: 300 images + 300 labels (with 3x augmentation)
- valid/: 30 images + 30 labels  
- test/: 15 images + 15 labels

Classes:
- marker (class_id: 0)

Format: YOLO (text files)
- Each .txt file: class_id center_x center_y width height
- All coordinates normalized (0-1)

Source: https://universe.roboflow.com/detection-oyhgk/my-first-project-3t6or/dataset/1

