from ultralytics import YOLO

# Load your trained model
model = YOLO('/home/suryansh/All-Coding-FIles/YOLO-Trained/runs/train/yolo11_visdrone_final2/weights/best.pt')

# Test on validation images
# results = model.predict(
#     source='/home/suryansh/All-Coding-FIles/YOLO-Trained/Visdrone/Output/images/val',
#     save=True,
#     conf=0.25,  # Confidence threshold
#     iou=0.45,   # NMS IoU threshold
#     imgsz=640
# )

# Test real-time performance
# model.predict(source=2, show=True, conf=0.25)  # 0 = webcam


#Test on video
# model.predict(
#     source='test.mp4',
#     save=True,
#     conf=0.25,
#     show=True
# )