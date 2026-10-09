"""
strawberry_picker.py — 草莓偵測 + 深度定位 + 機械手臂採摘（Jetson Nano 上執行的整合主程式）

整體流程：
  1. Intel RealSense D435 同時取得彩色影像與深度影像（640x480, 30 FPS）
  2. YOLOv4-tiny（OpenCV DNN + CUDA FP16）偵測草莓，取得每個框的中心點
  3. 把中心點像素座標除以 30 轉成「格子座標」(x: 0~20, y: 0~15)，再讀該點深度 z（公分）
  4. 依實驗量測的查表 + 等差公式，把 (x, y, z) 換算成各舵機的 PWM 脈寬：
        v1 = 10 * firstAngle[x]                                  # 0 號舵機：左右旋轉
        v2 = 10 * (array[x][y][0] - array[x][y][2] * (z - 20))   # 2 號舵機：上下前後
        v3 = 10 * (array[x][y][1] - array[x][y][3] * (z - 20))   # 4 號舵機：上下前後
  5. 透過 PCA9685（I2C）輸出 50Hz PWM 控制舵機：張爪 → 伸出 → 轉向 → 夾取 → 回到原位 → 放開

硬體：Jetson Nano、Intel RealSense D435、PCA9685 16 路 PWM 驅動板、6 軸舵機機械手臂
執行前需在同一資料夾放入：yolov4-tiny.weights、yolov4-tiny.cfg、classes.txt（訓練時的類別清單）

來源：畢業專題小組 Notion 紀錄中的最終整合版本（「Ruby改」），只補上註解，程式邏輯未更動。
"""

import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import time
import math
import smbus

# YOLO 偵測的信心度門檻與 NMS 門檻
Conf_threshold = 0.4
NMS_threshold = 0.4
COLORS = [(0, 255, 0), (0, 0, 255), (255, 0, 0),
          (255, 255, 0), (255, 0, 255), (0, 255, 255)]

# ---------------------------------------------------------------------------
# 舵機查表（由實驗逐點量測而得）
#   array[x][y] = [2 號舵機在深度 20cm 的角度, 4 號舵機在深度 20cm 的角度,
#                  2 號舵機每增加 1cm 深度要減少的角度, 4 號舵機每增加 1cm 深度要減少的角度]
#   x、y 是畫面格子座標（像素 / 30），預設值 [230,230,0,0] 代表尚未量測（手臂不伸出）
# ---------------------------------------------------------------------------
# 創建一個 21x16 的三維陣列，每個格子存入 4 個整數
array = [[[230,230,0,0] for _ in range(16)] for _ in range(21)]
# 更改特定位置的整數值
array[10][5] = [170,165,9,12]
array[9][5] = [165,158,6,8]
array[8][5] = [155,145,9,11]
array[10][7] = [170,165,8,10]
array[9][7] = [165,158,6,8]
array[8][7] = [160,150,8,9]
array[7][7] = [163,153,7,8]
array[6][7] = [155,145,9,10]
array[5][7] = [150,140,9,11]
array[4][7] = [145,140,9,11]
array[4][5] = [140,120,9,12]
array[5][5] = [145,120,7,8]
array[6][5] = [150,130,9,14]
array[7][5] = [158,143,8,10]
array[10][4] = [170,160,9,12]
array[9][4] = [165,143,9,12]
array[8][4] = [155,145,9,12]
array[7][4] = [158,143,9,12]
array[6][4] = [160,135,9,12]
array[5][4] = [145,120,9,12]
array[4][4] = [140,120,9,12]
array[10][6] = [170,165,9,12]
array[9][6] = [165,158,6,8]
array[8][6] = [155,145,9,11]
array[7][6] = [158,148,8,10]
array[6][6] = [150,140,9,14]
array[5][6] = [145,130,7,13]
array[4][6] = [140,120,9,12]
array[13][10] = [175,170,5,5]
array[12][10] = [180,167,6,6]
array[11][10] = [175,165,5,5]
array[10][10] = [170,165,5,5]
array[9][10] = [165,158,6,8]
array[8][10] = [180,168,6,7]
array[7][10] = [178,168,7,7]
array[6][10] = [170,160,9,9]
array[5][10] = [145,155,9,9]
array[4][10] = [160,150,8,8]
array[3][10] = [155,145,8,8]
array[12][9] = [180,177,7,7]
array[11][9] = [175,165,6,6]
array[10][9] = [170,165,7,8]
array[9][9] = [165,158,6,8]
array[8][9] = [170,160,7,8]
array[7][9] = [173,163,8,8]
array[6][9] = [165,155,4,10]
array[5][9] = [160,150,9,10]
array[4][9] = [155,145,9,11]
array[3][9] = [150,140,9,11]
array[9][11] = [165,168,6,6]
array[8][11] = [170,160,8,8]
array[7][11] = [170,165,7,7]
array[6][11] = [165,155,9,9]
array[5][11] = [160,160,9,10]
array[4][11] = [160,160,9,11]
array[3][11] = [160,160,9,11]
array[10][8] = [170,165,7,7]
array[9][8] = [165,158,6,8]
array[8][8] = [165,155,9,11]
array[7][8] = [168,158,8,8]
array[6][8] = [160,150,9,9]
array[5][8] = [155,145,9,10]

# 0 號舵機（左右旋轉）的角度只跟 x 有關，用一維陣列 firstAngle[x] 查表
firstAngle = [0]*21
for i in range(21):
  firstAngle[i] = 150
firstAngle[1] = 181
firstAngle[2] = 179
firstAngle[3] = 177
firstAngle[4] = 175
firstAngle[5] = 173
firstAngle[6] = 171
firstAngle[7] = 169
firstAngle[8] = 167
firstAngle[9] = 163
firstAngle[10] = 160
firstAngle[11] = 159
firstAngle[12] = 155
firstAngle[13] = 153
firstAngle[14] = 151
firstAngle[15] = 149
firstAngle[16] = 147
firstAngle[17] = 145
firstAngle[18] = 143
firstAngle[19] = 141
firstAngle[20] = 139

# 讀取 YOLO 訓練時的類別名稱（每行一個）
class_name = []
with open('classes.txt', 'r') as f:
    class_name = [cname.strip() for cname in f.readlines()]


class PCA9685:
    """PCA9685 16 路 PWM 驅動板（I2C bus 1, 位址 0x40）。舵機需要 50Hz、脈寬 500~2500us 的 PWM。"""

    # Registers/etc.
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self, address=0x40, debug=False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        if (self.debug):
            print("Reseting PCA9685")
        self.write(self.__MODE1, 0x00)

    def write(self, reg, value):
        "Writes an 8-bit value to the specified register/address"
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg):
        "Read an unsigned byte from the I2C device"
        result = self.bus.read_byte_data(self.address, reg)
        if (self.debug):
            print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
        return result

    def setPWMFreq(self, freq):
        "Sets the PWM frequency"
        # 預除頻值 = 25MHz 內部時脈 / (4096 階 * 目標頻率) - 1
        prescaleval = 25000000.0  # 25MHz
        prescaleval /= 4096.0  # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        if (self.debug):
            print("Setting PWM frequency to %d Hz" % freq)
            print("Estimated pre-scale: %d" % prescaleval)
        prescale = math.floor(prescaleval + 0.5)
        if (self.debug):
            print("Final pre-scale: %d" % prescale)

        # 修改 PRESCALE 前必須先讓晶片進入 sleep 模式
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10  # sleep
        self.write(self.__MODE1, newmode)  # go to sleep
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        "Sets a single PWM channel"
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

    def setServoPulse(self, channel, pulse):
        "Sets the Servo Pulse,The PWM frequency must be 50HZ"
        # 把脈寬（微秒）換算成 12-bit 計數值：週期 20000us 對應 4096 階
        pulse = int(pulse * 4096 / 20000)  # PWM frequency is 50HZ,the period is 20000us
        self.setPWM(channel, 0, pulse)

    def setMotoPluse(self, channel, pulse):
        if pulse > 3000:
            self.setPWM(channel, 0, 3000)
        else:
            self.setPWM(channel, 0, pulse)


pwm=PCA9685(0x40)
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

# Start streaming
pipeline.start(config)

# lastTimeNoDepth：上一次目標「讀不到深度」時，0 號舵機相對中心(1500)的偏移量。
# 讀不到深度通常是草莓在畫面邊緣，手臂會先轉過去再重新偵測一次。
lastTimeNoDepth = 0
# finish：本次是否已完成轉向（完成才會執行回原位的動作）
finish = 0
count = 0
try:
    # ---- 手臂回到初始姿勢 ----
    # 舵機編號：0 左右旋轉、2/4 上下前後、6/8 手腕、10 夾爪（1000 張開、1600 閉合）
    pwm.setServoPulse(0, 1500)
    pwm.setServoPulse(2, 2300)
    time.sleep(0.3)
    pwm.setServoPulse(4, 2300)
    pwm.setServoPulse(6, 2500)
    pwm.setServoPulse(8, 2500)
    pwm.setServoPulse(10, 1600)
    while count == 0 :
        # Wait for a coherent color frame
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # YOLOv4-tiny detection
        starting_time = time.time()
        classes, scores, boxes = model.detect(color_image, Conf_threshold, NMS_threshold)

        # List to hold detections
        detections = []

        for (classid, score, box) in zip(classes, scores, boxes):
            class_index = int(classid)
            color = COLORS[class_index % len(COLORS)]
            class_name_str = class_name[class_index]
            label = f"{class_name_str} : {score[0]:.2f}"
            detections.append((class_name_str, score[0], box))
            cv.rectangle(color_image, box, color, 1)
            # Calculate the center of the rectangle
            center_x = box[0] + box[2] // 2
            center_y = box[1] + box[3] // 2
            cv.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)
            cv.putText(color_image, label, (box[0], box[1] - 10),
                       cv.FONT_HERSHEY_COMPLEX, 0.3, color, 1)

        endingTime = time.time() - starting_time
        fps = 1 / endingTime
        cv.putText(color_image, f'FPS: {fps:.2f}', (20, 50),
                   cv.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
        # Show images
        cv.imshow('RealSense', color_image)
        key = cv.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        count = len(detections)
        print(f"檢測到的物件數量 = {count}")
        if count != 0:
            # 逐一處理偵測到的每顆草莓
            for detection in detections:
                # 取框中心點的深度值（公尺）
                height, width, _ = color_image.shape
                center_x = detection[2][0]+detection[2][2]//2
                center_y = detection[2][1]+detection[2][3]//2
                depth = depth_frame.get_distance(center_x, center_y)
                print(f'Class: {detection[0]}, Score: {detection[1]:.2f}, Box: {detection[2]}, Center: {int(center_x/30),int(center_y/30),depth*100}')
                # (x, y, z) → 舵機脈寬；z 只在 0~30cm 的採摘範圍內才伸出手臂
                v1 = 10*firstAngle[int(center_x/30)]
                if depth*100 != 0 and depth*100 < 30 :
                    v2 = 10*(array[int(center_x/30)][int(center_y/30)][0] - array[int(center_x/30)][int(center_y/30)][2] * (int(depth*100) - 20))
                    v3 = 10*(array[int(center_x/30)][int(center_y/30)][1] - array[int(center_x/30)][int(center_y/30)][3] * (int(depth*100) - 20))
                else:
                    # 讀不到深度或超出範圍：2、4 號舵機維持初始位置（不伸出）
                    v2 = 2300
                    v3 = 2300
                print(f"v1 = {v1}")
                print(f"v2 = {v2}")
                print(f"v3 = {v3}")
                count = count-1
                print(f"count = {count}")
                #上一次沒有深度值：這次要把上次先轉過去的偏移量補回來
                if lastTimeNoDepth != 0 :
                    #v1範圍要設定 最大2500
                    v1 = v1 + (lastTimeNoDepth-200)
                    print("lastTimeNoDepth!=0")
                    print(f"lastTimeNoDepth = {lastTimeNoDepth}")
                    print(f"v1 = {v1}")
                #這次沒有深度值：先往目標方向轉一些，讓草莓進入看得到深度的範圍
                if v2 == 2300 and v3 == 2300 :
                    lastTimeNoDepth = v1-1500
                    #v1範圍要設定 最小500
                    v1 = v1-200
                    print("no depth")
                    print(f"lastTimeNoDepth = {lastTimeNoDepth}")
                    print(f"v1 = {v1}")
                #這次有深度值
                else:
                    lastTimeNoDepth = 0
                # 目標脈寬
                i0 = v1
                i2 = v2
                i4 = v3
                i6 = 2500
                # 每一步移動 10us 的方向（正：增加、負：減少）
                if i0 > 1500 :
                    flag0 = 10
                elif i0 == 1500 :
                    flag0 = 0
                else:
                    flag0 = -10
                if i4 > 2300 :
                    flag4 = 10
                elif i4 == 2300 :
                    flag4 = 0
                else:
                    flag4 = -10
                flag2 = -10

                # ---- 第一段：張開夾爪，2、4 號舵機一步步移到目標（慢慢移動避免轉過頭）----
                temp2 = 2300
                temp4 = 2300
                while  temp4 != i4 or temp2 != i2:
                    #開始動開爪
                    i10 = 1000
                    pwm.setServoPulse(10, i10)
                    if temp4 != i4:
                            temp4 += flag4
                            pwm.setServoPulse(4, temp4)

                    if temp2 != i2:
                            temp2 += flag2
                            pwm.setServoPulse(2, temp2)
                    time.sleep(0.01)
                # ---- 第二段：0 號舵機左右轉向草莓 ----
                #草莓在右邊
                if(i0 < 1500):
                    for i in range(1500, i0, -10):
                            pwm.setServoPulse(0, i-10)
                            time.sleep(0.005)
                            finish = 1
                    #有深度值到達後夾
                    if lastTimeNoDepth == 0 :
                            i10 = 1600
                            pwm.setServoPulse(10, i10)

                    #這次沒深度轉到看得到的地方之後重跑while count == 0迴圈
                    else :
                            count = 0
                            break
                #草莓在左邊或中間
                else:
                    for i in range(1500, i0, 10):
                            pwm.setServoPulse(0, i+10)
                            time.sleep(0.005)
                            finish = 1
                    #有深度值到達後夾
                    if lastTimeNoDepth == 0 :
                            i10 = 1600
                            pwm.setServoPulse(10, i10)

                    #這次沒深度轉到看得到的地方之後重跑while count == 0迴圈
                    else :
                            count = 0
                            break
                # ---- 第三段：完成夾取，依相反順序回到起始位置，再張爪放下草莓 ----
                if lastTimeNoDepth == 0 and finish == 1:
                    time.sleep(3)
                    #左右轉回
                    if(i0 < 1500):
                        for i in range(i0, 1500, 10):
                            pwm.setServoPulse(0, i+10)
                            time.sleep(0.005)
                    else:
                        for i in range(i0, 1500, -10):
                            pwm.setServoPulse(0, i-10)
                            time.sleep(0.005)
                    while  temp4 != 2300 or temp2 != 2300:
                        if temp4 != 2300:
                            temp4 -= flag4
                            pwm.setServoPulse(4, temp4)

                        if temp2 != 2300:
                            temp2 -= flag2
                            pwm.setServoPulse(2, temp2)
                        time.sleep(0.01)
                    pwm.setServoPulse(6, 2500)
                    pwm.setServoPulse(8, 2500)
                    #放完草莓合起來
                    pwm.setServoPulse(10, 1000)
                    time.sleep(2)
                    pwm.setServoPulse(10, 1600)
                    lastTimeNoDepth = 0
                    finish = 0
finally:
    # Stop streaming
    pipeline.stop()
    cv.destroyAllWindows()
