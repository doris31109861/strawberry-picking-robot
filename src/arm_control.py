"""
arm_control.py — 機械手臂控制：影像座標＋深度 → 舵機脈寬，以及 PCA9685 舵機驅動

這個模組集中了手臂控制的部分（曾苔湘負責），主程式 strawberry_picker.py 只負責偵測與流程。
數學部分（pixel_to_cell、xyz_to_pulses）不需要任何硬體，可以直接單元測試（tests/test_arm_control.py）。

校正方法（2024/8–9 實驗）：
  - 把 640×480 畫面每 30 像素切成一格，x: 0–20、y: 0–15
  - 逐格量測手臂碰到目標所需的舵機角度；同一格裡，深度每增加 1 cm，
    2 號與 4 號舵機的角度大致以固定公差減少（等差數列），因此每格只需記 4 個數字：
        array[x][y] = [2 號舵機在 20cm 的角度, 4 號舵機在 20cm 的角度, 2 號舵機的公差, 4 號舵機的公差]
  - 0 號舵機（左右旋轉）只跟 x 有關，記在 firstAngle[x]
  - 角度 × 10 就是 PCA9685 要輸出的脈寬（微秒）

來源：小組 Notion 上的 angle_code.py（角度計算）與 one_OK_change.py（整合主程式）。
"""

import math
import time

# ---------------------------------------------------------------------------
# 舵機編號與初始（收回）位置的脈寬（微秒）
# ---------------------------------------------------------------------------
SERVO_BASE = 0          # 左右旋轉
SERVO_REACH_1 = 2       # 上下前後
SERVO_REACH_2 = 4       # 上下前後
SERVO_WRIST_1 = 6
SERVO_WRIST_2 = 8
SERVO_GRIPPER = 10      # 夾爪

BASE_CENTER = 1500      # 0 號舵機正中央
REACH_HOME = 2300       # 2、4 號舵機收回的位置（不伸出）
WRIST_HOME = 2500
GRIPPER_OPEN = 1000
GRIPPER_CLOSED = 1600

CELL_SIZE = 30          # 每格 30 像素
GRID_W, GRID_H = 21, 16 # 640/30 → 0..20、480/30 → 0..15
MIN_DEPTH_CM, MAX_DEPTH_CM = 0, 30   # 深度只在 (0, 30) cm 之間才伸出手臂

# ---------------------------------------------------------------------------
# 校正查表（實驗量測值）。預設 [230, 230, 0, 0] 代表這一格沒有量測：
# 換算後 2、4 號舵機都是 2300，也就是手臂不伸出。
# ---------------------------------------------------------------------------
array = [[[230, 230, 0, 0] for _ in range(GRID_H)] for _ in range(GRID_W)]
array[10][5] = [170, 165, 9, 12]
array[9][5] = [165, 158, 6, 8]
array[8][5] = [155, 145, 9, 11]
array[10][7] = [170, 165, 8, 10]
array[9][7] = [165, 158, 6, 8]
array[8][7] = [160, 150, 8, 9]
array[7][7] = [163, 153, 7, 8]
array[6][7] = [155, 145, 9, 10]
array[5][7] = [150, 140, 9, 11]
array[4][7] = [145, 140, 9, 11]
array[4][5] = [140, 120, 9, 12]
array[5][5] = [145, 120, 7, 8]
array[6][5] = [150, 130, 9, 14]
array[7][5] = [158, 143, 8, 10]
array[10][4] = [170, 160, 9, 12]
array[9][4] = [165, 143, 9, 12]
array[8][4] = [155, 145, 9, 12]
array[7][4] = [158, 143, 9, 12]
array[6][4] = [160, 135, 9, 12]
array[5][4] = [145, 120, 9, 12]
array[4][4] = [140, 120, 9, 12]
array[10][6] = [170, 165, 9, 12]
array[9][6] = [165, 158, 6, 8]
array[8][6] = [155, 145, 9, 11]
array[7][6] = [158, 148, 8, 10]
array[6][6] = [150, 140, 9, 14]
array[5][6] = [145, 130, 7, 13]
array[4][6] = [140, 120, 9, 12]
array[13][10] = [175, 170, 5, 5]
array[12][10] = [180, 167, 6, 6]
array[11][10] = [175, 165, 5, 5]
array[10][10] = [170, 165, 5, 5]
array[9][10] = [165, 158, 6, 8]
array[8][10] = [180, 168, 6, 7]
array[7][10] = [178, 168, 7, 7]
array[6][10] = [170, 160, 9, 9]
array[5][10] = [145, 155, 9, 9]
array[4][10] = [160, 150, 8, 8]
array[3][10] = [155, 145, 8, 8]
array[12][9] = [180, 177, 7, 7]
array[11][9] = [175, 165, 6, 6]
array[10][9] = [170, 165, 7, 8]
array[9][9] = [165, 158, 6, 8]
array[8][9] = [170, 160, 7, 8]
array[7][9] = [173, 163, 8, 8]
array[6][9] = [165, 155, 4, 10]
array[5][9] = [160, 150, 9, 10]
array[4][9] = [155, 145, 9, 11]
array[3][9] = [150, 140, 9, 11]
array[9][11] = [165, 168, 6, 6]
array[8][11] = [170, 160, 8, 8]
array[7][11] = [170, 165, 7, 7]
array[6][11] = [165, 155, 9, 9]
array[5][11] = [160, 160, 9, 10]
array[4][11] = [160, 160, 9, 11]
array[3][11] = [160, 160, 9, 11]
array[10][8] = [170, 165, 7, 7]
array[9][8] = [165, 158, 6, 8]
array[8][8] = [165, 155, 9, 11]
array[7][8] = [168, 158, 8, 8]
array[6][8] = [160, 150, 9, 9]
array[5][8] = [155, 145, 9, 10]

# 0 號舵機（左右旋轉）的角度只跟 x 有關：越往右（x 越大）角度越小
firstAngle = [150] * GRID_W
for _x, _angle in enumerate([150, 181, 179, 177, 175, 173, 171, 169, 167, 163, 160,
                             159, 155, 153, 151, 149, 147, 145, 143, 141, 139]):
    firstAngle[_x] = _angle


def pixel_to_cell(cx, cy):
    """影像像素座標 → 校正格子座標 (x, y)。

    原程式直接用 int(cx/30)，畫面最右邊 630–639 像素會得到 x = 21，超出查表範圍而當掉；
    這裡把格子座標限制在查表範圍內。
    """
    x = min(max(int(cx / CELL_SIZE), 0), GRID_W - 1)
    y = min(max(int(cy / CELL_SIZE), 0), GRID_H - 1)
    return x, y


def xyz_to_pulses(x, y, depth_cm):
    """格子座標 (x, y) 與深度（公分）→ (v1, v2, v3) 三個舵機的脈寬（微秒）。

        v1 = 10 * firstAngle[x]                                   0 號舵機：左右
        v2 = 10 * (array[x][y][0] - array[x][y][2] * (z - 20))    2 號舵機
        v3 = 10 * (array[x][y][1] - array[x][y][3] * (z - 20))    4 號舵機
    讀不到深度（0）或超出 30 cm 時，2、4 號舵機維持收回位置 2300（手臂不伸出）。
    """
    v1 = 10 * firstAngle[x]
    if MIN_DEPTH_CM < depth_cm < MAX_DEPTH_CM:
        z = int(depth_cm)
        a = array[x][y]
        v2 = 10 * (a[0] - a[2] * (z - 20))
        v3 = 10 * (a[1] - a[3] * (z - 20))
    else:
        v2 = v3 = REACH_HOME
    return v1, v2, v3


class PCA9685:
    """PCA9685 16 路 PWM 驅動板（I2C bus 1, 位址 0x40）。舵機需要 50Hz、脈寬 500~2500us 的 PWM。

    bus 參數可以傳入假的 I2C 物件做測試；沒有傳入時才在 Jetson 上開啟 smbus。
    """

    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09

    def __init__(self, address=0x40, debug=False, bus=None):
        if bus is None:
            import smbus  # 只有在 Jetson 上才需要
            bus = smbus.SMBus(1)
        self.bus = bus
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg, value):
        """寫入一個 8-bit 暫存器"""
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg):
        """讀取一個 8-bit 暫存器"""
        return self.bus.read_byte_data(self.address, reg)

    def setPWMFreq(self, freq):
        """設定 PWM 頻率：預除頻值 = 25MHz / (4096 × 頻率) − 1，改值前要先讓晶片進入 sleep"""
        prescale = math.floor(25000000.0 / 4096.0 / float(freq) - 1.0 + 0.5)
        oldmode = self.read(self.__MODE1)
        self.write(self.__MODE1, (oldmode & 0x7F) | 0x10)   # sleep
        self.write(self.__PRESCALE, int(prescale))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)            # restart

    def setPWM(self, channel, on, off):
        """設定單一通道在 4096 階中的開啟、關閉時間點"""
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

    def setServoPulse(self, channel, pulse):
        """以微秒設定舵機脈寬（50Hz 週期 20000us 對應 4096 階）"""
        self.setPWM(channel, 0, int(pulse * 4096 / 20000))


def move_home(pwm, i0, temp2, temp4, flag2, flag4, step_delay=0.03, retract_delay=0.03):
    """把手臂收回起始姿勢：0 號舵機轉回中央，2、4 號舵機一步步退回 2300，手腕歸位。

    i0 為 0 號舵機目前位置；temp2 / temp4 為 2、4 號舵機目前位置；flag2 / flag4 為伸出時每步的增量
    （收回時反向）。retract_delay 是收回每一步的間隔，原程式在「兩次都讀不到深度」的分支沒有等待，
    呼叫端可傳 0 保持原本的行為。
    """
    if i0 < BASE_CENTER:
        for i in range(i0, BASE_CENTER, 10):
            pwm.setServoPulse(SERVO_BASE, i + 10)
            time.sleep(step_delay)
    else:
        for i in range(i0, BASE_CENTER, -10):
            pwm.setServoPulse(SERVO_BASE, i - 10)
            time.sleep(step_delay)
    while temp4 != REACH_HOME or temp2 != REACH_HOME:
        if temp4 != REACH_HOME:
            temp4 -= flag4
            pwm.setServoPulse(SERVO_REACH_2, temp4)
        if temp2 != REACH_HOME:
            temp2 -= flag2
            pwm.setServoPulse(SERVO_REACH_1, temp2)
        time.sleep(retract_delay)
    pwm.setServoPulse(SERVO_WRIST_1, WRIST_HOME)
    pwm.setServoPulse(SERVO_WRIST_2, WRIST_HOME)
