# accelerometer - magnetometer: lsm303dlhc
# gyroscope: l3gd20 (in fact )

# reference document:
# dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.pdf

import math
import time
from smbus2 import SMBus

RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0


bus = SMBus(9)

# For acc and magneto see "Table 3. Sensor characteristics" in datasheet
# For gyro see "Table 4. Sensor characteristics" in datasheet

# acc. range: +/- 2g, sensibility: 1 mg/LSB
bus.write_byte_data(0x19, 0x20, 0x37)  # data rate = 25Hz, enable all 3 angles
bus.write_byte_data(0x19, 0x23, 0xc8)  # BDU = 1, BLE = 1, scale = +/- 2g, HR = 1

# gyro. range: +/- 250 deg/ s, sensibility: 0.00875 deg/s
# bus.write_byte_data(0x69, 0x24, [0x80])
bus.write_byte_data(0x69, 0x20, 0x0f)  # data rate = 95Hz, Cut-Off = 12.5, normal mode, enable all 3 angles
bus.write_byte_data(0x69, 0x23, 0xc0)  # BDU = 1, BLE = 1, scale = 250 deg/s

# magneto; range:+/- 2.5 gauss, gain: 670 for X, Y and 600 for Z
bus.write_byte_data(0x1e, 0x00, 0x14)  # data rate = 30Hz
bus.write_byte_data(0x1e, 0x01, 0x60)  # scale = +/- 2.5 Gauss
bus.write_byte_data(0x1e, 0x02, 0x00)  # continuous conversion mode

# initial  values
current_time = 0
pitch = 0.0
roll = 0.0

while True:
    time.sleep(0.1)

    old_time = current_time
    current_time = time.time_ns()
    if old_time == 0:
        dt = 0.1
    else:
        dt = (current_time - old_time) / 1000000000  # convert ns to s

    # read acc
    data = bus.read_i2c_block_data(0x19, 0xa8, 6)  # 0x28 + 0x80
    # print([hex(x) for x in data])
    a_x = int.from_bytes(data[0:2], "big", signed=True) / 16
    a_y = int.from_bytes(data[2:4], "big", signed=True) / 16
    a_z = int.from_bytes(data[4:6], "big", signed=True) / 16
    _roll = math.atan2(a_y, a_z)
    cr = math.cos(roll)
    sr = math.sin(roll)
    gz2 = a_y * sr + a_z * cr
    _pitch = math.atan(-a_x / gz2)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    # print(f"_pitch={_pitch:0.2f}, _roll={_roll:0.2f}")

    # read gyro
    data = bus.read_i2c_block_data(0x69, 0xa8, 6)
    # print([hex(x) for x in data])
    g_x = int.from_bytes(data[0:2], "big", signed=True) * 0.00875 * DEG_TO_RAD
    g_y = int.from_bytes(data[2:4], "big", signed=True) * 0.00875 * DEG_TO_RAD
    g_z = int.from_bytes(data[4:6], "big", signed=True) * 0.00875 * DEG_TO_RAD
    # print(f"g_x={g_x}, g_y={g_y}, g_z={g_z}")

    # complementery filter (in radians)
    pitch = 0.8 * (pitch + dt * g_y) + 0.2 * _pitch
    roll = 0.8 * (roll + dt * g_x) + 0.2 * _roll

    # read magnetometer / unit = Gauss
    data = bus.read_i2c_block_data(0x1e, 0x03, 6)
    # print([hex(x) for x in data])
    _x = int.from_bytes(data[0:2], "big", signed=True) / 670.0
    _y = int.from_bytes(data[4:6], "big", signed=True) / 670.0
    _z = int.from_bytes(data[2:4], "big", signed=True) / 600.0
    # print(f"_x={_x}, _y={_y}, _z={_z}")
    by2 = _z * sr - _y * cr
    bz2 = _y * sr + _z * cr
    bx3 = _x * cp + bz2 * sp
    yaw = math.atan2(by2, bx3)
    print(f"pitch={pitch * RAD_TO_DEG:0.2f}, roll={roll * RAD_TO_DEG:0.2f}, yaw={yaw * RAD_TO_DEG:0.2f}")
