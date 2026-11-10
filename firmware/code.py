import math
import sys
import time
import board
import busio
import pwmio
import supervisor
from adafruit_motor import servo
from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

kp = 1.0
kd = 0.15
limit = 15.0
max_throttle = 0.5
run_time = 45
cutoff = 25.0

# flip these to -1 if an axis pushes the wrong way
flip_a = 1
flip_b = 1
centre_a = 90
centre_b = 90

# fan angle -> servo angle, from the cad. measure the real ones and replace
fan_pts = [-15, -10, -5, 0, 5, 10, 15]
servo_a_pts = [-37, -24, -11, 0, 11, 22, 33]
servo_b_pts = [-37, -24, -12, 0, 11, 23, 36]

i2c = busio.I2C(board.GP5, board.GP4, frequency=400000)
imu = BNO08X_I2C(i2c)
imu.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
imu.enable_feature(BNO_REPORT_GYROSCOPE)

servo_a = servo.Servo(pwmio.PWMOut(board.GP14, frequency=50), min_pulse=500, max_pulse=2500)
servo_b = servo.Servo(pwmio.PWMOut(board.GP15, frequency=50), min_pulse=500, max_pulse=2500)
esc = pwmio.PWMOut(board.GP16, frequency=50)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def lookup(fan, pts):
    fan = clamp(fan, fan_pts[0], fan_pts[-1])
    for i in range(len(fan_pts) - 1):
        if fan <= fan_pts[i + 1]:
            t = (fan - fan_pts[i]) / (fan_pts[i + 1] - fan_pts[i])
            return pts[i] + t * (pts[i + 1] - pts[i])
    return pts[-1]


def throttle(amount):
    us = 1000 + 1000 * clamp(amount, 0, max_throttle)
    esc.duty_cycle = int(us / 20000 * 65535)


def tilt():
    # which way is down, seen from the rocket. no compass so the fan magnet can't mess it up
    x, y, z, w = imu.game_quaternion
    gx = 2 * (x * z - w * y)
    gy = 2 * (y * z + w * x)
    gz = 1 - 2 * (x * x + y * y)
    return math.degrees(math.atan2(gy, gz)), math.degrees(math.atan2(gx, gz))


def key():
    if supervisor.runtime.serial_bytes_available:
        return sys.stdin.read(1)
    return None


# arm
throttle(0)
servo_a.angle = centre_a
servo_b.angle = centre_b
time.sleep(3)

fan_on = False
use_d = True
started = 0
power = 0.0
last_print = 0

# keys: g = fan on, s = fan off, p = switch between P and PD
while True:
    k = key()
    if k == "g":
        fan_on = True
        started = time.monotonic()
    elif k == "s":
        fan_on = False
    elif k == "p":
        use_d = not use_d

    tilt_x, tilt_y = tilt()
    rate_x, rate_y, _ = imu.gyro
    rate_x = math.degrees(rate_x)
    rate_y = math.degrees(rate_y)

    d = kd if use_d else 0
    fan_a = clamp(flip_a * (kp * tilt_x + d * rate_x), -limit, limit)
    fan_b = clamp(flip_b * (kp * tilt_y + d * rate_y), -limit, limit)

    servo_a.angle = clamp(centre_a + lookup(fan_a, servo_a_pts), 0, 180)
    servo_b.angle = clamp(centre_b + lookup(fan_b, servo_b_pts), 0, 180)

    # safety
    if fan_on and (time.monotonic() - started > run_time or abs(tilt_x) > cutoff or abs(tilt_y) > cutoff):
        fan_on = False
    target = max_throttle if fan_on else 0
    power += clamp(target - power, -0.05, 0.01)
    throttle(power)

    now = time.monotonic()
    if now - last_print > 0.05:
        last_print = now
        print(f"{tilt_x:.1f},{tilt_y:.1f},{fan_a:.1f},{fan_b:.1f},{power:.2f},{'PD' if use_d else 'P'}")
