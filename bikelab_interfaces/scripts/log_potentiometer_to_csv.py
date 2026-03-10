#!/usr/bin/env python3
import os
import csv
import time
import math
from typing import List, Optional

import smbus

# -------- config (same as your ROS script) --------
I2C_BUS = 1
I2C_ADDR = 0x24
REG = 0x10

ADC_minus_90 = 235
ADC_0 = 551
ADC_90 = 867

SAMPLE_PERIOD_S = 0.1     # 10 Hz, same as your timer
FLUSH_ROWS = 500          # write every 500 samples (adjust as you like)

out_dir = os.path.expanduser("~/bikelab_interface_logs")
ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
OUT_CSV = os.path.join(out_dir, f"steering_angle_{ts}.csv")


def interpolate_angle(adc_value: int, points):
    """
    Linear interpolation between calibration points.
    points: [(adc, angle_deg), ...] in ascending adc order.
    Out-of-range -> clamp to min/max angle.
    """
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        if x0 <= adc_value <= x1:
            return y0 + (y1 - y0) * (adc_value - x0) / (x1 - x0)

    # clamp
    if adc_value < points[0][0]:
        return float(points[0][1])
    if adc_value > points[-1][0]:
        return float(points[-1][1])
    return float("nan")


def now_unix_ns() -> int:
    return time.time_ns()


class PotentiometerCSVLogger:
    def __init__(self, out_csv: str, flush_rows: int = 500):
        self.out_csv = out_csv
        self.flush_rows = int(flush_rows)

        os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)

        # I2C
        self.bus = smbus.SMBus(I2C_BUS)

        # calibration points
        self.points = [
            (ADC_minus_90, -90.0),
            (ADC_0, 0.0),
            (ADC_90, 90.0),
        ]

        # CSV
        self._open_csv()

        # buffer
        self.rows: List[list] = []

    def _open_csv(self):
        new_file = not os.path.exists(self.out_csv) or os.path.getsize(self.out_csv) == 0
        self.f = open(self.out_csv, "a", newline="")
        self.w = csv.writer(self.f)
        if new_file:
            self.w.writerow([
                "t_unix_ns",
                "ok",
                "adc_raw",
                "angle_deg",
                "angle_deg_clamped",
                "error",
            ])
            self.f.flush()

    def read_adc(self) -> int:
        """
        Mimic your ROS code:
          write_byte(addr, reg)
          read_word_data(addr, reg)
        Note: read_word_data endianness can differ per chip.
        This keeps the exact behavior you had.
        """
        self.bus.write_byte(I2C_ADDR, REG)
        return self.bus.read_word_data(I2C_ADDR, REG)

    def log_once(self):
        t = now_unix_ns()
        try:
            adc = int(self.read_adc())

            angle = interpolate_angle(adc, self.points)

            # "clamped" angle is already clamped by interpolate_angle for out-of-range;
            # keep both columns for clarity.
            angle_clamped = angle

            row = [t, 1, adc, angle, angle_clamped, ""]
        except Exception as e:
            # distinguish "missing" from "0": use NaN and ok=0
            row = [t, 0, math.nan, math.nan, math.nan, str(e)]

        self.rows.append(row)

        if len(self.rows) >= self.flush_rows:
            self.flush()

    def flush(self):
        if not self.rows:
            return
        self.w.writerows(self.rows)
        self.f.flush()
        self.rows.clear()

    def close(self):
        try:
            self.flush()
        finally:
            try:
                self.f.close()
            except Exception:
                pass
            try:
                self.bus.close()
            except Exception:
                pass

    def run(self, period_s: float = 0.1):
        print(f"[INFO] Logging potentiometer to {self.out_csv}")
        print(f"[INFO] Sampling period: {period_s}s, flush every {self.flush_rows} rows")
        next_t = time.time()
        try:
            while True:
                self.log_once()

                # simple scheduler to keep a stable period
                next_t += period_s
                sleep_s = next_t - time.time()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    # we're behind; reset schedule to avoid drift explosion
                    next_t = time.time()
        except KeyboardInterrupt:
            print("\n[INFO] Ctrl-C received. Flushing and exiting…")
        finally:
            self.close()
            print("[INFO] Done.")


if __name__ == "__main__":
    logger = PotentiometerCSVLogger(out_csv=OUT_CSV, flush_rows=FLUSH_ROWS)
    logger.run(period_s=SAMPLE_PERIOD_S)