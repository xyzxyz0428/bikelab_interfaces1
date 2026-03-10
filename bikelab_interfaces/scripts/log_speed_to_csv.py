#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import time
import math
import argparse
from collections import Counter

from openant.easy.node import Node as ANTNode
from openant.devices import ANTPLUS_NETWORK_KEY
from openant.devices.bike_speed_cadence import (
    BikeSpeed,
    BikeSpeedData as ANT_BikeSpeedData,
)

WHEEL_CIRCUMFERENCE_M = 2.20114


def nan():
    return float("nan")


class BikeSpeedLogger:
    def __init__(
        self,
        bike_speed_id: int = 18412,
        wheel_circumference_m: float = WHEEL_CIRCUMFERENCE_M,
        out_dir: str = "~/bikelab_interface_logs",
        prefix: str = "bike_speed",
    ):
        self.bike_speed_id = int(bike_speed_id)
        self.wheel_circumference_m = float(wheel_circumference_m)

        # Output dir/file
        out_dir = os.path.expanduser("~/bikelab_interface_logs")
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self.csv_path  = os.path.join(out_dir, f"speed_decoded_{ts}.csv")
        self.page_log_path = os.path.join(out_dir, f"speed_pages_{ts}.text")

        self.csv_f = open(self.csv_path, "w", newline="")
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow([
            "t_unix_ns",
            "t_unix_s",
            "device_id",
            "page",
            "page_hex",
            "page_name",
            "speed_mps",
            "distance_m",
            "wheel_circumference_m",
        ])

        # Page statistics
        self.page_counter = Counter()
        self.page_set = set()
        self.last_stat_t = time.time_ns()

        # ANT setup
        self.ant_node = ANTNode()
        self.ant_node.set_network_key(0x00, ANTPLUS_NETWORK_KEY)

        self.bike_speed_device = BikeSpeed(self.ant_node, device_id=self.bike_speed_id)
        self.bike_speed_device.on_found = self.on_bike_speed_found
        self.bike_speed_device.on_device_data = self.on_bike_speed_data

    def on_bike_speed_found(self):
        print(f"[SPEED] Bike Speed Device {self.bike_speed_device} found and receiving")

    def on_bike_speed_data(self, page: int, page_name: str, data):
        now_ns = time.time_ns()
        now_s = now_ns / 1e9

        # page discovery
        self.page_counter[page] += 1
        self.page_set.add(page)

        if now_ns - self.last_stat_t > 10_000_000_000:  # 10 s
            s = ", ".join([f"0x{k:02X}:{v}" for k, v in self.page_counter.most_common()])
            print(f"[SPEED] pages (last 10s): {s}")
            with open(self.page_log_path, "w") as f:
                f.write("\n".join([f"0x{p:02X}" for p in sorted(self.page_set)]))
            self.page_counter.clear()
            self.last_stat_t = now_ns

        if not isinstance(data, ANT_BikeSpeedData):
            return

        speed = data.calculate_speed(self.wheel_circumference_m)
        distance = data.calculate_distance(self.wheel_circumference_m)

        # Keep exact semantics:
        # if speed cannot be computed, store NaN
        speed_val = float(speed) if speed is not None else nan()
        distance_val = float(distance) if distance is not None else nan()

        self.csv_w.writerow([
            now_ns,
            now_s,
            self.bike_speed_id,
            page,
            f"0x{page:02X}",
            page_name,
            speed_val,
            distance_val,
            self.wheel_circumference_m,
        ])
        self.csv_f.flush()

    def run(self):
        print("[SPEED] Starting ANT+ speed logger, press Ctrl-C to stop")
        self.ant_node.start()

        try:
            while True:
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\n[SPEED] Stopping...")
        finally:
            self.close()

    def close(self):
        try:
            self.bike_speed_device.close_channel()
        except Exception:
            pass
        try:
            self.ant_node.stop()
        except Exception:
            pass
        try:
            self.csv_f.close()
        except Exception:
            pass
        print(f"[SPEED] CSV saved to: {self.csv_path}")


def main():
    parser = argparse.ArgumentParser(description="ROS-free Garmin bike speed logger")
    parser.add_argument("--bike_speed_id", type=int, default=18412, help="ANT+ bike speed sensor device ID")
    parser.add_argument("--wheel_circumference_m", type=float, default=WHEEL_CIRCUMFERENCE_M, help="Wheel circumference in meters")
    parser.add_argument("--out_dir", type=str, default="/mnt/bikelab_data", help="Output directory")
    parser.add_argument("--prefix", type=str, default="bike_speed", help="Output file prefix")
    args = parser.parse_args()

    logger = BikeSpeedLogger(
        bike_speed_id=args.bike_speed_id,
        wheel_circumference_m=args.wheel_circumference_m,
        out_dir=args.out_dir,
        prefix=args.prefix,
    )
    logger.run()


if __name__ == "__main__":
    main()