"""
angle_calculator.py — 輸入格子座標與深度，算出三個舵機的脈寬（校正時使用的小工具）

不需要任何硬體，可在一般電腦執行：python tools/angle_calculator.py
來源：小組 Notion 上的 angle_code.py（Colab 筆記本），計算改為呼叫 src/arm_control.py。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from arm_control import GRID_H, GRID_W, xyz_to_pulses  # noqa: E402

while True:
    try:
        x = int(input(f"請輸入整數 x (0~{GRID_W - 1}): "))
        y = int(input(f"請輸入整數 y (0~{GRID_H - 1}): "))
        z = int(input("請輸入深度 z (cm): "))
        if not (0 <= x < GRID_W and 0 <= y < GRID_H):
            print("x、y 超出校正範圍")
            continue
        v1, v2, v3 = xyz_to_pulses(x, y, z)
        print(f"v1（0 號舵機）= {v1}")
        print(f"v2（2 號舵機）= {v2}")
        print(f"v3（4 號舵機）= {v3}")    # 原程式這行誤印成 "v2 ="
        if input("是否繼續輸入？(y/n): ").lower() != 'y':
            print("程式結束。")
            break
    except ValueError:
        print("請輸入有效的整數。")
