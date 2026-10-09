"""
tests/test_arm_control.py — 手臂控制的單元測試（不需要 Jetson、相機或舵機）

用法：python tests/test_arm_control.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import arm_control as ac  # noqa: E402


def original_formula(x, y, depth_cm):
    """原本 angle_code.py / one_OK_change.py 裡直接寫在主程式的算法，用來比對重構後結果相同。"""
    v1 = 10 * ac.firstAngle[x]
    if depth_cm != 0 and depth_cm < 30:
        v2 = 10 * (ac.array[x][y][0] - ac.array[x][y][2] * (int(depth_cm) - 20))
        v3 = 10 * (ac.array[x][y][1] - ac.array[x][y][3] * (int(depth_cm) - 20))
    else:
        v2 = v3 = 2300
    return v1, v2, v3


def test_matches_original_formula_everywhere():
    # 所有格子 × 深度 0~35cm（含小數）都與原算法相同
    for x in range(ac.GRID_W):
        for y in range(ac.GRID_H):
            for d10 in range(0, 351, 5):
                d = d10 / 10
                assert ac.xyz_to_pulses(x, y, d) == original_formula(x, y, d), (x, y, d)


def test_known_cell():
    # (10, 5) 校正值 [170, 165, 9, 12]、firstAngle[10] = 160
    assert ac.xyz_to_pulses(10, 5, 20) == (1600, 1700, 1650)        # 20cm：直接是校正值 ×10
    assert ac.xyz_to_pulses(10, 5, 25) == (1600, 1250, 1050)        # 每多 1cm 減 9、12 度
    assert ac.xyz_to_pulses(10, 5, 25.9) == (1600, 1250, 1050)      # 深度取整數公分


def test_no_extension_when_unusable():
    assert ac.xyz_to_pulses(0, 0, 25)[1:] == (2300, 2300)            # 未校正的格子：不伸出
    assert ac.xyz_to_pulses(10, 5, 0)[1:] == (2300, 2300)            # 讀不到深度
    assert ac.xyz_to_pulses(10, 5, 30)[1:] == (2300, 2300)           # 超出 30cm


def test_pulses_are_multiples_of_10():
    # 伸出／收回時每步移動 10us，目標值必須是 10 的倍數，否則 while 迴圈永遠到不了目標
    for x in range(ac.GRID_W):
        for y in range(ac.GRID_H):
            for d in range(0, 31):
                assert all(v % 10 == 0 for v in ac.xyz_to_pulses(x, y, d))


def test_pixel_to_cell():
    assert ac.pixel_to_cell(0, 0) == (0, 0)
    assert ac.pixel_to_cell(319, 159) == (10, 5)
    assert ac.pixel_to_cell(639, 479) == (20, 15)   # 原程式在 x >= 630 會得到 21 而超出查表


class FakeBus:
    """假的 I2C：記錄寫入的暫存器值"""
    def __init__(self):
        self.regs = {}

    def write_byte_data(self, addr, reg, value):
        self.regs[reg] = value

    def read_byte_data(self, addr, reg):
        return self.regs.get(reg, 0)


def test_pca9685_pulse_conversion():
    bus = FakeBus()
    pwm = ac.PCA9685(bus=bus)
    pwm.setPWMFreq(50)
    assert bus.regs[0xFE] == 121                         # 25MHz / (4096 × 50) − 1 ≈ 121
    pwm.setServoPulse(2, 1500)                           # 1500us → 1500 × 4096 / 20000 = 307
    off = bus.regs[0x08 + 4 * 2] | (bus.regs[0x09 + 4 * 2] << 8)
    assert off == 307


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"PASS {name}")
