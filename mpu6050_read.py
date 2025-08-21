#!/usr/bin/env python3
import time, math, struct
from smbus import SMBus

I2C_BUS   = 1
ADDR      = 0x68  # 0x69 if AD0 high

# MPU6050 registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A   # DLPF
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B

# scale factors
ACC_LSB_PER_G   = 16384.0   # ±2g
GYRO_LSB_PER_DPS= 16.4      # ±2000 dps
G_TO_MS2        = 9.80665
DEG2RAD         = math.pi / 180.0

def _twos_compliment(h, l):
    v = (h << 8) | l
    return v - 65536 if v & 0x8000 else v

def write_reg(bus, reg, val):
    bus.write_byte_data(ADDR, reg, val & 0xFF)

def init_mpu(bus):
    # wake up
    write_reg(bus, PWR_MGMT_1, 0x00)
    time.sleep(0.05)
    # DLPF 42 Hz (CONFIG=3)
    write_reg(bus, CONFIG, 0x03)
    # sample rate: 1kHz / (1 + SMPLRT_DIV) = 100 Hz
    write_reg(bus, SMPLRT_DIV, 9)
    # gyro ±2000 dps
    write_reg(bus, GYRO_CONFIG, 0x18)
    # accel ±2g
    write_reg(bus, ACCEL_CONFIG, 0x00)
    time.sleep(0.05)

def read_all(bus):
    # Read 14 bytes: Ax,Ay,Az,Temp,Gx,Gy,Gz
    raw = bus.read_i2c_block_data(ADDR, ACCEL_XOUT_H, 14)
    ax = _twos_compliment(raw[0], raw[1])
    ay = _twos_compliment(raw[2], raw[3])
    az = _twos_compliment(raw[4], raw[5])
    # temp = _twos_compliment(raw[6], raw[7])
    gx = _twos_compliment(raw[8], raw[9])
    gy = _twos_compliment(raw[10], raw[11])
    gz = _twos_compliment(raw[12], raw[13])

    # convert to SI
    ax_ms2 = (ax / ACC_LSB_PER_G) * G_TO_MS2
    ay_ms2 = (ay / ACC_LSB_PER_G) * G_TO_MS2
    az_ms2 = (az / ACC_LSB_PER_G) * G_TO_MS2

    gx_rads = (gx / GYRO_LSB_PER_DPS) * DEG2RAD
    gy_rads = (gy / GYRO_LSB_PER_DPS) * DEG2RAD
    gz_rads = (gz / GYRO_LSB_PER_DPS) * DEG2RAD

    return ax_ms2, ay_ms2, az_ms2, gx_rads, gy_rads, gz_rads

def calibrate_gyro(bus, seconds=2.0):
    print(f"[MPU] Calibrating gyro bias for {seconds}s… keep still")
    t_end = time.monotonic() + seconds
    n = 0
    sx = sy = sz = 0.0
    while time.monotonic() < t_end:
        _, _, _, gx, gy, gz = read_all(bus)
        sx += gx; sy += gy; sz += gz
        n += 1
        time.sleep(0.002)
    if n == 0: return 0.0, 0.0, 0.0
    return sx/n, sy/n, sz/n

def main():
    bus = SMBus(I2C_BUS)
    # quick WHO_AM_I check (0x75 should be 0x68)
    who = bus.read_byte_data(ADDR, 0x75)
    print(f"[MPU] WHO_AM_I: 0x{who:02X}")
    if who not in (0x68, 0x69):
        print("[MPU] Unexpected WHO_AM_I; check wiring/address")

    init_mpu(bus)
    bias_gx, bias_gy, bias_gz = calibrate_gyro(bus, 2.0)
    print(f"[MPU] Gyro bias (rad/s): {bias_gx:.5f}, {bias_gy:.5f}, {bias_gz:.5f}")

    period = 1.0 / 100.0
    next_t = time.monotonic()
    try:
        while True:
            t_us = time.monotonic_ns() // 1000  # µs timestamp
            ax, ay, az, gx, gy, gz = read_all(bus)

            gx -= bias_gx; gy -= bias_gy; gz -= bias_gz

            # print one line (no saving)
            print(f"{t_us}, ax={ax:+7.3f} ay={ay:+7.3f} az={az:+7.3f}  "
                  f"gx={gx:+6.3f} gy={gy:+6.3f} gz={gz:+6.3f}")

            # keep ~100 Hz cadence
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()  # drift catch-up
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()

if __name__ == "__main__":
    main()
