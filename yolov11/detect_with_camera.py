from ultralytics import YOLO

# 載入訓練好的模型
model = YOLO(r"C:\Users\stu10\yolov11\runs\detect\train\weights\best.pt")  # 替換為實際的權重路徑

# 使用攝影機進行檢測
model.predict(source=0, show=True, conf=0.90) 

# source是鏡頭選擇，如果0不行可以試試換其他數字
# conf是閾值 可以調 介於0~1之間