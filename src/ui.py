"""
ui.py — 採摘系統的操作介面（tkinter）

功能：
  - 即時顯示 RealSense D435 的彩色畫面（每 10ms 更新一次）
  - 「執行程式」按鈕：以子行程啟動 strawberry_picker.py（偵測 + 手臂採摘）
  - 「結束程式」按鈕：停止攝影機串流並關閉視窗

來源：畢業專題小組 Notion 紀錄（UI.py / UIV2.py）。
原本按鈕執行的檔名是佔位用的 "paste.txt"，這裡改成同資料夾的 strawberry_picker.py。
"""

import tkinter as tk
from tkinter import ttk
import pyrealsense2 as rs
import numpy as np
import cv2
from PIL import Image, ImageTk
import subprocess
import sys
from pathlib import Path

# 按下「執行程式」時要啟動的主程式
PICKER_SCRIPT = Path(__file__).resolve().parent / "strawberry_picker.py"


class CameraApp:
    def __init__(self, window):
        self.window = window
        self.window.title("攝影機控制介面")

        # 建立主框架
        self.main_frame = ttk.Frame(window, padding="10")
        self.main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # 攝影機畫面框架
        self.camera_frame = ttk.Frame(self.main_frame)
        self.camera_frame.grid(row=0, column=0, columnspan=2, pady=5)

        # 攝影機標籤（用於顯示畫面）
        self.camera_label = ttk.Label(self.camera_frame)
        self.camera_label.grid(row=0, column=0)

        # 狀態訊息
        self.status_var = tk.StringVar(value="系統狀態: 就緒")
        self.status_label = ttk.Label(self.main_frame, textvariable=self.status_var)
        self.status_label.grid(row=1, column=0, columnspan=2, pady=5)

        # 按鈕框架
        self.button_frame = ttk.Frame(self.main_frame)
        self.button_frame.grid(row=2, column=0, columnspan=2, pady=5)

        # 執行按鈕
        self.run_button = ttk.Button(self.button_frame, text="執行程式", command=self.run_program)
        self.run_button.grid(row=0, column=0, padx=5)

        # 結束按鈕
        self.quit_button = ttk.Button(self.button_frame, text="結束程式", command=self.quit_program)
        self.quit_button.grid(row=0, column=1, padx=5)

        # 初始化 RealSense 攝影機
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        # 更新攝影機畫面
        self.update_camera()

    def update_camera(self):
        # Wait for a coherent color frame
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # 轉換顏色空間從BGR到RGB
        frame_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # 轉換為PIL影像
        pil_image = Image.fromarray(frame_rgb)

        # 調整大小（可以根據需要修改）
        pil_image = pil_image.resize((640, 480))

        # 轉換為PhotoImage（要存在 self 上，否則會被回收導致畫面空白）
        self.photo = ImageTk.PhotoImage(image=pil_image)

        # 更新標籤
        self.camera_label.configure(image=self.photo)

        # 每10毫秒更新一次
        self.window.after(10, self.update_camera)

    def run_program(self):
        try:
            # 以獨立行程執行採摘主程式，介面不會被卡住
            subprocess.Popen([sys.executable, str(PICKER_SCRIPT)], cwd=PICKER_SCRIPT.parent)
            self.status_var.set("系統狀態: 程式執行中")
        except Exception as e:
            self.status_var.set(f"系統狀態: 執行錯誤 - {str(e)}")

    def quit_program(self):
        # Stop streaming
        self.pipeline.stop()
        self.window.quit()


if __name__ == "__main__":
    root = tk.Tk()
    app = CameraApp(root)
    root.mainloop()
