from ultralytics import YOLO
import cv2

model = YOLO('/home/suryansh/All-Coding-FIles/YOLO-Trained/runs/train/yolo11_visdrone_final2/weights/best.pt')
img = cv2.imread('/home/suryansh/All-Coding-FIles/YOLO-Trained/Visdrone/VisDrone2019-DET-train/images/0000016_01352_d_0000069.jpg')  # <-- use the same image you ran before
    # image_path = ""  # <-- change to your test image

results = model(img, conf=0.25, imgsz=640, verbose=False)[0]

print("Detected classes:", [results.names[int(c)] for c in results.boxes.cls])
print("Number of detections:", len(results.boxes))
    