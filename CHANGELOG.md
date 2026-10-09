# Changelog

## 2026-10-09 — 更新分工說明

- **內容**：README（中英）與 `docs/notion.md` 的分工改為：曾苔湘負責機械手臂程式控制（PCA9685 舵機驅動、座標與深度轉舵機角度的查表、夾取／歸位流程）；呂奕萱、邱怡清負責文獻與模型訓練；全員撰寫報告。
- **原因**：依曾苔湘本人說明更正；公開的 Notion 分工表目前仍只寫「報告撰寫」。
- **測試**：只改文件。

## 2026-10-09 — README 加入採摘 GIF

- **內容**：從專題影片的「細節回放」片段擷取約 6 秒（1.5 倍速），做成 `docs/demo.gif`（480px、約 2.3MB），放在 README 開頭；完整影片仍在 GitHub Release。
- **原因**：README 一打開就能看到手臂實際夾取草莓。
- **測試**：逐格確認畫面從夾取開始到草莓放入盒中。

## 2026-10-09 — 建立 repo

- **內容**：從小組 Notion 紀錄整理出最終整合程式（`src/strawberry_picker.py`）、操作介面（`src/ui.py`）、單張偵測（`src/detect_image.py`），補上中文註解；`docs/notion.md` 整理硬體、資料集、YOLOv4-tiny / YOLOv8 實驗數據與手臂校正方法；中英雙語 README；demo 影片放在 GitHub Release。
- **程式改動**：`strawberry_picker.py` 只加註解，並刪除查表中重複且數值相同的賦值，邏輯未變；`ui.py` 的執行目標從佔位的 `paste.txt` 改成 `strawberry_picker.py`；`detect_image.py` 把 Colab 專用的 `cv2_imshow` 改成 `cv.imshow`、圖片路徑改為命令列參數。
- **原因**：專題程式原本只存在 Notion 頁面中。
- **測試**：三個檔案皆通過 `py_compile` 語法檢查。**未測**：需要 Jetson Nano、RealSense、PCA9685 與機械手臂硬體，無法在此電腦實際執行。
- **注意**：原 Notion 頁面公開了 Roboflow API key 與 Jetson 預設登入資訊，已不放入 repo。
