#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import time
import argparse
from collections import Counter
from typing import Optional

from openant.easy.node import Node as ANTNode
from openant.devices import ANTPLUS_NETWORK_KEY
from openant.devices.bike_speed_cadence import (
    BikeSpeed,
    BikeSpeedData as ANT_BikeSpeedData,
)
from openant.devices.power_meter import PowerMeter


WHEEL_CIRCUMFERENCE_M = 2.20114


def nan():
    return float("nan")


def u16_le(b: bytes, i: int) -> int:
    return b[i] | (b[i + 1] << 8)


class BikeSpeedCSVLogger:
    def __init__(
        self,
        ant_node: ANTNode,
        bike_speed_id: int,
        wheel_circumference_m: float,
        out_dir: str,
        prefix: str,
        stats_every_s: float = 10.0,
    ):
        self.bike_speed_id = int(bike_speed_id)
        self.wheel_circumference_m = float(wheel_circumference_m)
        self.stats_every_s = float(stats_every_s)

        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self.csv_path = os.path.join(out_dir, f"{prefix}_{ts}.csv")
        self.page_log_path = os.path.join(out_dir, f"{prefix}_pages_{ts}.txt")

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

        self.page_counter = Counter()
        self.page_set = set()
        self.last_stat_t = time.time_ns()

        self.dev = BikeSpeed(ant_node, device_id=self.bike_speed_id)
        self.dev.on_found = self.on_found
        self.dev.on_device_data = self.on_device_data

    def on_found(self):
        print(f"[SPEED] Found device_id={self.bike_speed_id}")
        print(f"[SPEED] CSV: {self.csv_path}")

    def on_device_data(self, page: int, page_name: str, data):
        now_ns = time.time_ns()
        now_s = now_ns / 1e9

        self.page_counter[page] += 1
        self.page_set.add(page)

        if now_ns - self.last_stat_t > int(self.stats_every_s * 1e9):
            s = ", ".join([f"0x{k:02X}:{v}" for k, v in self.page_counter.most_common()])
            print(f"[SPEED] pages ({self.stats_every_s:.0f}s): {s}")
            with open(self.page_log_path, "w") as f:
                f.write("\n".join([f"0x{p:02X}" for p in sorted(self.page_set)]))
            self.page_counter.clear()
            self.last_stat_t = now_ns

        if not isinstance(data, ANT_BikeSpeedData):
            return

        speed = data.calculate_speed(self.wheel_circumference_m)
        distance = data.calculate_distance(self.wheel_circumference_m)

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

    def close(self):
        try:
            self.dev.close_channel()
        except Exception:
            pass
        try:
            self.csv_f.close()
        except Exception:
            pass
        print(f"[SPEED] CSV saved to: {self.csv_path}")


class PowerMeterPayloadLogger(PowerMeter):
    def __init__(
        self,
        ant_node: ANTNode,
        device_id: int,
        csv_path: str,
        flush_every_n: int = 200,
        flush_every_s: float = 2.0,
        stats_every_s: float = 10.0,
    ):
        super().__init__(ant_node, device_id=device_id)

        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        self.f = open(csv_path, "w", newline="")
        self.w = csv.writer(self.f)

        self.w.writerow([
            "t_unix_ns", "t_unix_s",
            "device_id",
            "page", "page_hex",
            "page_name",
            "cadence_rpm",
            "p10_update_event_count",
            "p10_pedal_power_percent",
            "p10_pedal_power_is_right",
            "p10_accumulated_power_w",
            "p10_instantaneous_power_w",
            "p12_update_event_count",
            "p12_crank_ticks",
            "p12_crank_period_1_2048s",
            "p12_accumulated_torque_1_32nm",
        ])

        self.flush_every_n = int(flush_every_n)
        self.flush_every_s = float(flush_every_s)
        self._rows_since_flush = 0
        self._last_flush_t = time.monotonic()

        self.stats_every_s = float(stats_every_s)
        self._last_stat_t = time.monotonic()
        self.page_counter = Counter()

        self._pending_raw: Optional[bytes] = None
        self._pending_page: Optional[int] = None
        self._pending_t_ns: Optional[int] = None

        self.on_device_data = self._on_device_data_logged

    def close(self):
        try:
            self.f.flush()
        finally:
            self.f.close()

    def _maybe_flush(self):
        now = time.monotonic()
        if self._rows_since_flush >= self.flush_every_n or (now - self._last_flush_t) >= self.flush_every_s:
            self.f.flush()
            self._rows_since_flush = 0
            self._last_flush_t = now

    def _maybe_stats(self):
        now = time.monotonic()
        if now - self._last_stat_t >= self.stats_every_s:
            c10 = self.page_counter.get(0x10, 0)
            c12 = self.page_counter.get(0x12, 0)
            total = sum(self.page_counter.values())
            others = [(k, v) for k, v in self.page_counter.items() if k not in (0x10, 0x12)]
            others_str = ", ".join([f"0x{k:02X}:{v}" for k, v in sorted(others)]) if others else "-"
            print(f"[POWER] stats {self.stats_every_s:.0f}s total={total}, 0x10={c10}, 0x12={c12}, others={others_str}")
            self.page_counter.clear()
            self._last_stat_t = now

    def on_data(self, data):
        try:
            raw = bytes(data)
        except Exception:
            raw = bytes(bytearray(data))
        if len(raw) != 8:
            raw = raw[:8].ljust(8, b"\x00")

        page = raw[0]
        self.page_counter[page] += 1
        self._maybe_stats()

        self._pending_raw = raw
        self._pending_page = page
        self._pending_t_ns = time.time_ns()

        return super().on_data(data)

    def _on_device_data_logged(self, page: int, page_name: str, parsed):
        if self._pending_raw is None or self._pending_page is None or self._pending_t_ns is None:
            raw = b"\x00" * 8
            raw_page = page
            t_ns = time.time_ns()
        else:
            raw = self._pending_raw
            raw_page = self._pending_page
            t_ns = self._pending_t_ns

        if raw_page not in (0x10, 0x12):
            self._pending_raw = None
            self._pending_page = None
            self._pending_t_ns = None
            return

        t_s = t_ns / 1e9

        cadence = nan()

        p10_evt = nan()
        p10_pp_pct = nan()
        p10_pp_is_right = nan()
        p10_acc_pwr = nan()
        p10_inst_pwr = nan()

        p12_evt = nan()
        p12_ticks = nan()
        p12_crank_period = nan()
        p12_acc_torque = nan()

        cad_byte = raw[3]
        if cad_byte != 0xFF:
            cadence = float(cad_byte)

        if raw_page == 0x10:
            p10_evt = float(raw[1])
            pedal_power = raw[2]
            is_right = (pedal_power >> 7) & 1
            pct = pedal_power & 0x7F
            p10_pp_is_right = float(is_right)
            p10_pp_pct = float(pct)
            p10_acc_pwr = float(u16_le(raw, 4))
            p10_inst_pwr = float(u16_le(raw, 6))

        elif raw_page == 0x12:
            p12_evt = float(raw[1])
            p12_ticks = float(raw[2])
            p12_crank_period = float(u16_le(raw, 4))
            p12_acc_torque = float(u16_le(raw, 6))

        self.w.writerow([
            t_ns, t_s,
            self.device_id,
            raw_page, f"0x{raw_page:02X}",
            page_name,
            cadence,
            p10_evt,
            p10_pp_pct,
            p10_pp_is_right,
            p10_acc_pwr,
            p10_inst_pwr,
            p12_evt,
            p12_ticks,
            p12_crank_period,
            p12_acc_torque,
        ])
        self._rows_since_flush += 1
        self._maybe_flush()

        self._pending_raw = None
        self._pending_page = None
        self._pending_t_ns = None


def main():
    parser = argparse.ArgumentParser(description="One ANT+ logger for Garmin speed + Rally power")
    parser.add_argument("--bike_speed_id", type=int, default=18412)
    parser.add_argument("--power_meter_id", type=int, default=64434)
    parser.add_argument("--wheel_circumference_m", type=float, default=WHEEL_CIRCUMFERENCE_M)
    parser.add_argument("--out_dir", type=str, default="/mnt/bikelab_data")
    parser.add_argument("--speed_prefix", type=str, default="speed_decoded")
    parser.add_argument("--power_prefix", type=str, default="rally_payload_decoded")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())

    speed_csv = os.path.join(args.out_dir, f"{args.speed_prefix}_{ts}.csv")
    power_csv = os.path.join(args.out_dir, f"{args.power_prefix}_{ts}.csv")

    ant = ANTNode()
    ant.set_network_key(0x00, ANTPLUS_NETWORK_KEY)

    speed_logger = BikeSpeedCSVLogger(
        ant_node=ant,
        bike_speed_id=args.bike_speed_id,
        wheel_circumference_m=args.wheel_circumference_m,
        out_dir=args.out_dir,
        prefix=args.speed_prefix,
        stats_every_s=10.0,
    )

    power_logger = PowerMeterPayloadLogger(
        ant_node=ant,
        device_id=args.power_meter_id,
        csv_path=power_csv,
        flush_every_n=200,
        flush_every_s=2.0,
        stats_every_s=10.0,
    )

    def power_found():
        print(f"[POWER] Found device_id={args.power_meter_id}")
        print(f"[POWER] CSV: {power_csv}")

    power_logger.on_found = power_found

    print("[INFO] Starting one ANT node for speed + power (Ctrl-C to stop)...")
    try:
        ant.start()
    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
    finally:
        try:
            speed_logger.close()
        except Exception:
            pass
        try:
            power_logger.close_channel()
        except Exception:
            pass
        try:
            power_logger.close()
        except Exception:
            pass
        try:
            ant.stop()
        except Exception:
            pass
        print("[DONE] All CSV files closed.out_dir")


if __name__ == "__main__":
    main()