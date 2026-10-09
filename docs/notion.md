# 專題 Notion 紀錄整理

> 整理自小組的公開 Notion 頁面「畢業專題」（2024/3 – 2024/12）。原頁面含大量參考連結、截圖與實驗紀錄，這裡只保留與系統設計、實驗結果相關的重點。
> 帳號密碼、API key 等資訊已移除。

## 分工

| 成員 | 工作 |
|---|---|
| 呂奕萱 | 閱讀相關文獻、模型訓練、報告撰寫 |
| 邱怡清 | 閱讀相關文獻、模型訓練、報告撰寫 |
| 曾苔湘 | 機械手臂程式控制、報告撰寫 |
| 陳宥蓉 | 報告撰寫 |

> 曾苔湘的分工依本人說明更新（Notion 分工表原本只列「報告撰寫」）。

## 時程

- **2024/3–5**：決定辨識方法（成熟度、光線、效率、良率）、選定硬體（Jetson Nano）、期中計畫書
- **2024/6–7**：Jetson Nano 上跑通 YOLOv4、接上 RealSense 深度攝影機、手臂與 PCA9685 到貨
- **2024/7–9**：鏡頭＋模型整合、手臂舵機角度量測、建立座標→角度查表
- **2024/12/20**：專題發表

## 硬體與預算

| 項目 | 約略價格（NT$） |
|---|---|
| Intel RealSense D435 深度攝影機 | 6,700 |
| NVIDIA Jetson Nano | 4,500 |
| 6 軸舵機機械手臂 | 1,500 |
| 電源供應器 | 420 |
| PCA9685 PWM 驅動板 | 130 |

## 環境

- Jetson Nano：JetPack 4.6.4、Ubuntu 18.04.6、CUDA 10.2、Python 3.6.9
- YOLOv5 + PyTorch 在 Jetson Nano 上無法使用 GPU（PyTorch / CUDA / Python 版本相容問題），最後改用 **YOLOv4-tiny + OpenCV DNN（CUDA 後端）** 部署
- RealSense：`pyrealsense2`（照 librealsense issue #6964 的方法排除安裝問題）
- 舵機：PCA9685 透過 I2C（`smbus`）控制，50Hz PWM

## 資料集

- Roboflow Universe 上的草莓資料集（Strawberry 4.0 約 571 張：Flower / Ripe / Unripe / Rotten；berries 2,132 張等）
- 小組自行重新標註的資料集（604 張），之後加入 **Badly-shaped**（畸形果）類別
- 資料增強：saturation 1.5、exposure 1.5、hue 0.1、blur 1

## 模型實驗

### YOLOv4-tiny（Darknet 訓練，部署在 Jetson Nano）

| 設定 | mAP | Precision | Recall |
|---|---|---|---|
| 輸入 408×408 | 53.38% | 0.51 | 0.60 |
| 輸入 512×512 | 55.13% | 0.52 | 0.60 |
| 重新標註的資料集（604 張） | 72.89% | 0.77 | 0.82 |
| 只偵測成熟草莓（ripe） | 88.41% | 0.81 | 0.91 |

其他嘗試：直方圖均衡化 / CLAHE 前處理、把 badly-shaped 歸入 ripe 或 rotten、加入背景圖、平衡各類別數量。
模型大小約 26 MB，BFLOPS 約 6.8–10.3。

### YOLOv8s（Colab Tesla T4，25 epochs，比較用）

| 資料集 | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|
| 0716 版（4 類） | 0.688 | 0.509 | 0.577 | 0.363 |
| 0731 版（5 類，含 Badly-shaped） | 0.901 | 0.711 | 0.782 | 0.438 |

0731 版各類 mAP50：Ripe 0.914、Unripe 0.896、Flower 0.904、Rotten 0.652、Badly-shaped 0.545。

## 手臂控制：從影像座標到舵機角度

舵機編號：0 左右旋轉、2 / 4 上下前後、6 / 8 手腕、10 夾爪。

- 採摘範圍：深度 z = 20–30 cm；z = 20 時 X 0–35 cm、Y 10–30 cm
- 把 640×480 畫面以每 30 像素切成格子 (x: 0–20, y: 0–15)，逐格量測手臂碰到目標所需的舵機角度
- 觀察到：同一個 (x, y) 深度每增加 1 cm，2、4 號舵機角度大致成**等差數列**遞減；離畫面中心越遠，公差越大。用單一線性或非線性公式擬合全部資料都不夠準，因此改用「查表 + 等差」：

```
v1 = 10 * firstAngle[x]
v2 = 10 * (array[x][y][0] - array[x][y][2] * (z - 20))
v3 = 10 * (array[x][y][1] - array[x][y][3] * (z - 20))
```

- 畫面邊緣（x = 1–3、14–20）常讀不到深度：程式會先把手臂轉向該方向，讓草莓進入可取得深度的區域後再重新偵測

## 主要程式版本

| 檔案 | 說明 |
|---|---|
| `src/strawberry_picker.py` | 最終整合版（Notion 上的「Ruby改」）：偵測 → 深度 → 查表 → 舵機採摘 |
| `src/ui.py` | tkinter 操作介面（UIV2） |
| `src/detect_image.py` | 單張圖片的 YOLOv4-tiny 測試程式 |
