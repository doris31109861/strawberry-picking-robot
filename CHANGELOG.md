# Changelog

## 2026-10-09 — 手臂控制獨立成模組、主程式更新為最終版

- **內容**：
  - 讀取小組 Notion 的附件後發現 `one_OK_change.py` 才是最終版（舵機每步 30ms、偏移改為 150、新增讀不到深度時收回的處理），主程式改為這個版本（之前用的是程式碼區塊中較早的「Ruby改」）。
  - 新增 `src/arm_control.py`：校正查表、`pixel_to_cell()`、`xyz_to_pulses()`、PCA9685 驅動、`move_home()`（原本重複三次的收回動作）。主程式改為呼叫這些函式，流程與參數不變。
  - 新增 `tools/angle_calculator.py`（Notion 附件 angle_code.py：輸入 x、y、z 算出舵機脈寬），修正原本把 v3 印成 "v2" 的小錯。
  - 新增 `tests/test_arm_control.py` 與 GitHub Actions：重構後的算法與原本寫在主程式裡的算法，在所有格子、0–35cm 深度都完全相同；脈寬都是 10 的倍數（否則逐步移動的迴圈會停不下來）；PCA9685 脈寬換算正確。
  - 修正：畫面最右側 630–639 像素會算出 x = 21，超出查表範圍造成程式當掉；`pixel_to_cell()` 把格子座標限制在 0–20。
  - 註明：最終版中「讀不到深度時先轉向再重新偵測」的分支依判斷條件不會執行到，實際行為是收回後結束程式；註解已照實寫明，邏輯保留原樣。
  - 找不到 `classes.txt` 時印出說明（訓練好的 YOLOv4-tiny 權重、cfg、classes.txt 都沒有留存）。
- **原因**：讓手臂控制（曾苔湘負責的部分）獨立、可測試，並以實際使用的最終版為準。
- **測試**：`tests/test_arm_control.py` 6 項本機全部通過；`angle_calculator.py` 以 (10, 5, 25cm) 測試得到 1600／1250／1050。主程式需要 Jetson、相機與舵機，未實機測試。

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
