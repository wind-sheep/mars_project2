from ultralytics import YOLO

# Load a model
#model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)
model = YOLO(r"C:\Users\stu10\yolov11\yolo11n.pt")

# Train the model
results = model.train(data="VOC3.yaml", epochs=10000, imgsz=640, workers=0)