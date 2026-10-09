"""
detect_image.py — 用 YOLOv4-tiny 對單張圖片做草莓偵測（測試模型用）

用法：python detect_image.py <圖片路徑>
需要同資料夾的 yolov4-tiny.weights、yolov4-tiny.cfg、classes.txt。

來源：畢業專題小組 Notion 紀錄的 Colab 測試程式。
原本用 google.colab 的 cv2_imshow 顯示結果，這裡改成 cv.imshow 以便在 Jetson / 一般電腦上執行，
圖片路徑也改成命令列參數。
"""

import sys
import time

import cv2 as cv

# YOLO 偵測的信心度門檻與 NMS 門檻
Conf_threshold = 0.4
NMS_threshold = 0.4
COLORS = [(0, 255, 0), (0, 0, 255), (255, 0, 0),
          (255, 255, 0), (255, 0, 255), (0, 255, 255)]

# 讀取類別名稱（每行一個）
with open('classes.txt', 'r') as f:
    class_name = [cname.strip() for cname in f.readlines()]

# 載入 YOLOv4-tiny，使用 OpenCV DNN 的 CUDA 後端
net = cv.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')
net.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA_FP16)

model = cv.dnn_DetectionModel(net)
model.setInputParams(size=(416, 416), scale=1/255, swapRB=True)

# 讀取單張圖片
image_path = sys.argv[1] if len(sys.argv) > 1 else '5.jpg'
frame = cv.imread(image_path)
if frame is None:
    sys.exit(f"讀不到圖片: {image_path}")

starting_time = time.time()
classes, scores, boxes = model.detect(frame, Conf_threshold, NMS_threshold)

# 保存檢測結果的列表
detections = []

for (classid, score, box) in zip(classes, scores, boxes):
    cid = int(classid)
    color = COLORS[cid % len(COLORS)]
    label = "%s : %f" % (class_name[cid], float(score))
    detections.append((class_name[cid], float(score), box))
    cv.rectangle(frame, box, color, 1)
    cv.putText(frame, label, (box[0], box[1]-10),
               cv.FONT_HERSHEY_COMPLEX, 0.3, color, 1)

endingTime = time.time() - starting_time
fps = 1/endingTime
cv.putText(frame, f'FPS: {fps:.2f}', (20, 50),
           cv.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)

# 顯示處理後的圖片，按任意鍵關閉
cv.imshow('detection', frame)
cv.waitKey(0)
cv.destroyAllWindows()

# 輸出檢測結果
for detection in detections:
    print(f'Class: {detection[0]}, Score: {detection[1]:.2f}, Box: {detection[2]}')
