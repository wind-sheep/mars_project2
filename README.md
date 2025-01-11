# mars_project2

這是113學年度上學期 電機專題二 賣場購物機器人之應用
實作影片在 https://www.youtube.com/watch?v=NKbWQ3h64bs
內容為影像辨識與機器手臂取物
主要在使用YOLOv11訓練資料集
並用深度相機做辨識與獲取座標
經過座標轉換後使用手臂做夾取

其中local資料夾是在本機端執行的workspace
robot資料夾是在機器人端(MARS)執行的workspace
best3.pt是透過ultralytics提供的YOLOv11所訓練完成的model
