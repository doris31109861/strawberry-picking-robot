"""
strawberry_picker.py — 草莓偵測 + 深度定位 + 機械手臂採摘（Jetson Nano 上執行的整合主程式）

整體流程：
  1. Intel RealSense D435 同時取得彩色影像與深度影像（640x480, 30 FPS）
  2. YOLOv4-tiny（OpenCV DNN + CUDA FP16）偵測草莓，取得每個框的中心點
  3. 中心點 → 校正格子 (x, y)，再讀該點深度 z（公分）
  4. arm_control.xyz_to_pulses() 依查表換算 0、2、4 號舵機的脈寬
  5. 張爪 → 2、4 號舵機伸出 → 0 號舵機轉向 → 夾取 → 收回 → 放開

讀不到深度時（多半是草莓在畫面邊緣）：手臂往草莓方向多轉一些（偏移 150），停 3 秒後收回，並結束主迴圈。
注意：程式中「先轉向再重新偵測」的分支，依目前的判斷條件不會執行到（見下方註解）。

硬體：Jetson Nano、Intel RealSense D435、PCA9685 PWM 驅動板、6 軸舵機機械手臂
執行前需在同一資料夾放入：yolov4-tiny.weights、yolov4-tiny.cfg、classes.txt（訓練時的類別清單，見 README）

來源：小組 Notion 上的最終版 one_OK_change.py。查表、舵機驅動與收回動作移到 arm_control.py，
主迴圈的流程與參數（每步 30ms、偏移 150）維持原樣。
"""

import sys
import time

import cv2 as cv
import numpy as np
import pyrealsense2 as rs

from arm_control import (BASE_CENTER, GRIPPER_CLOSED, GRIPPER_OPEN, REACH_HOME, SERVO_BASE, SERVO_GRIPPER,
                         SERVO_REACH_1, SERVO_REACH_2, SERVO_WRIST_1, SERVO_WRIST_2, WRIST_HOME, PCA9685,
                         move_home, pixel_to_cell, xyz_to_pulses)

# YOLO 偵測的信心度門檻與 NMS 門檻
Conf_threshold = 0.4
NMS_threshold = 0.4
COLORS = [(0, 255, 0), (0, 0, 255), (255, 0, 0),
          (255, 255, 0), (255, 0, 255), (0, 255, 255)]
STEP_DELAY = 0.03       # 舵機每移動一步（10us）的間隔，太快容易轉過頭
NO_DEPTH_OFFSET = 150   # 讀不到深度時，0 號舵機先往目標方向偏移的量

# 讀取 YOLO 訓練時的類別名稱（每行一個）
try:
    with open('classes.txt', 'r') as f:
        class_name = [cname.strip() for cname in f.readlines()]
except FileNotFoundError:
    sys.exit("找不到 classes.txt：請放入訓練 YOLOv4-tiny 時使用的類別清單（見 README）")

pwm = PCA9685(0x40)
pwm.setPWMFreq(50)

# ---- 載入 YOLOv4-tiny，使用 OpenCV DNN 的 CUDA 後端（FP16 在 Jetson Nano 上較快）----
net = cv.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')
net.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA_FP16)
model = cv.dnn_DetectionModel(net)
model.setInputParams(size=(416, 416), scale=1/255, swapRB=True)

# ---- 設定 RealSense：同時開啟深度與彩色串流 ----
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# lastTimeNoDepth：上一次「讀不到深度」時 0 號舵機相對中央的偏移；0 代表上一次有讀到深度
lastTimeNoDepth = 0
finish = 0      # 本次是否已夾到草莓（夾到才執行放置動作）
count = 0
try:
    # ---- 手臂回到初始姿勢 ----
    pwm.setServoPulse(SERVO_BASE, BASE_CENTER)
    pwm.setServoPulse(SERVO_REACH_1, REACH_HOME)
    time.sleep(0.3)
    pwm.setServoPulse(SERVO_REACH_2, REACH_HOME)
    pwm.setServoPulse(SERVO_WRIST_1, WRIST_HOME)
    pwm.setServoPulse(SERVO_WRIST_2, WRIST_HOME)
    pwm.setServoPulse(SERVO_GRIPPER, GRIPPER_CLOSED)
    while count == 0:
        time.sleep(0.6)
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())

        # ---- YOLOv4-tiny 偵測，畫出框與中心點 ----
        starting_time = time.time()
        classes, scores, boxes = model.detect(color_image, Conf_threshold, NMS_threshold)
        detections = []
        for (classid, score, box) in zip(classes, scores, boxes):
            class_index = int(classid)
            color = COLORS[class_index % len(COLORS)]
            label = f"{class_name[class_index]} : {score[0]:.2f}"
            detections.append((class_name[class_index], score[0], box))
            cv.rectangle(color_image, box, color, 1)
            cv.circle(color_image, (box[0] + box[2] // 2, box[1] + box[3] // 2), 5, (0, 0, 255), -1)
            cv.putText(color_image, label, (box[0], box[1] - 10), cv.FONT_HERSHEY_COMPLEX, 0.3, color, 1)
        fps = 1 / (time.time() - starting_time)
        cv.putText(color_image, f'FPS: {fps:.2f}', (20, 50), cv.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
        cv.imshow('RealSense', color_image)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        count = len(detections)
        print(f"檢測到的物件數量 = {count}")
        # ---- 逐一處理偵測到的每顆草莓 ----
        for detection in detections:
            center_x = detection[2][0] + detection[2][2] // 2
            center_y = detection[2][1] + detection[2][3] // 2
            depth_cm = depth_frame.get_distance(center_x, center_y) * 100
            x, y = pixel_to_cell(center_x, center_y)
            v1, v2, v3 = xyz_to_pulses(x, y, depth_cm)
            print(f'Class: {detection[0]}, Score: {detection[1]:.2f}, Cell: {(x, y)}, depth: {depth_cm:.1f}cm, '
                  f'pulses: {v1} {v2} {v3}')
            count = count - 1

            # 上一次沒有深度值：把上次先轉過去的偏移量補回來
            if lastTimeNoDepth != 0:
                v1 = v1 + (lastTimeNoDepth - NO_DEPTH_OFFSET)
            # 這次沒有深度值：先往目標方向轉一些，讓草莓進入看得到深度的範圍
            if v2 == REACH_HOME and v3 == REACH_HOME:
                lastTimeNoDepth = v1 - BASE_CENTER
                v1 = v1 - NO_DEPTH_OFFSET
                print(f"no depth, lastTimeNoDepth = {lastTimeNoDepth}")
            else:
                lastTimeNoDepth = 0

            i0, i2, i4 = v1, v2, v3
            flag4 = 10 if i4 > REACH_HOME else (0 if i4 == REACH_HOME else -10)  # 每步移動方向
            flag2 = -10

            # ---- 第一段：張開夾爪，2、4 號舵機一步步伸到目標 ----
            temp2 = temp4 = REACH_HOME
            while temp4 != i4 or temp2 != i2:
                pwm.setServoPulse(SERVO_GRIPPER, GRIPPER_OPEN)
                if temp4 != i4:
                    temp4 += flag4
                    pwm.setServoPulse(SERVO_REACH_2, temp4)
                if temp2 != i2:
                    temp2 += flag2
                    pwm.setServoPulse(SERVO_REACH_1, temp2)
                time.sleep(STEP_DELAY)

            # ---- 第二段：0 號舵機轉向草莓（i0 < 1500 在右邊，否則在左邊或中間）----
            step = -10 if i0 < BASE_CENTER else 10
            for i in range(BASE_CENTER, i0, step):
                pwm.setServoPulse(SERVO_BASE, i + step)
                time.sleep(STEP_DELAY)

            if lastTimeNoDepth == 0:
                # 有深度值：到達後夾取
                pwm.setServoPulse(SERVO_GRIPPER, GRIPPER_CLOSED)
                finish = 1
            elif v2 == REACH_HOME and v3 == REACH_HOME:
                # 讀不到深度：收回手臂並結束（原程式註解寫 "don't see depth twice"，但因為 lastTimeNoDepth
                # 在上面同一輪就被設成非 0，第一次讀不到深度就會進入這裡）
                # 原程式此分支的收回迴圈每步沒有等待，retract_delay=0 維持原本行為
                time.sleep(3)
                move_home(pwm, i0, temp2, temp4, flag2, flag4, STEP_DELAY, retract_delay=0)
                pwm.setServoPulse(SERVO_GRIPPER, GRIPPER_CLOSED)
                lastTimeNoDepth = 0
                finish = 0
                print("don't see depth twice")
                count = -1   # 讓 while count == 0 結束，程式停止
                break
            else:
                # 原設計：第一次讀不到深度時轉向目標後重新偵測。依目前條件不會執行到，保留原程式邏輯
                count = 0
                break

            # ---- 第三段：夾到後收回並放開草莓 ----
            if lastTimeNoDepth == 0 and finish == 1:
                time.sleep(3)
                move_home(pwm, i0, temp2, temp4, flag2, flag4, STEP_DELAY)
                pwm.setServoPulse(SERVO_GRIPPER, GRIPPER_OPEN)   # 放下草莓
                time.sleep(2)
                pwm.setServoPulse(SERVO_GRIPPER, GRIPPER_CLOSED)
                lastTimeNoDepth = 0
                finish = 0
finally:
    pipeline.stop()
    cv.destroyAllWindows()
