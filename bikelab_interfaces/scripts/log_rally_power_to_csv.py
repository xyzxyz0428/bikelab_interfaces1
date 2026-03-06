#!/usr/bin/env python3
import os
import csv
import time
import math
from collections import Counter
from typing import Optional

from openant.easy.node import Node as ANTNode
from openant.devices import ANTPLUS_NETWORK_KEY
from openant.devices.power_meter import PowerMeter


def nan():
    return float("nan")


def u16_le(b: bytes, i: int) -> int:
    """little-endian uint16 from b[i],b[i+1]"""
    return b[i] | (b[i + 1] << 8)


class PowerMeterPayloadLogger(PowerMeter):
    """
    - Intercepts raw 8-byte payload in on_data()
    - Uses page_name from on_device_data callback
    - Writes ONLY decoded payload-coded fields (no raw bytes columns)
    """

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

        # One table for both pages; page-specific fields are NaN when not applicable
        self.w.writerow([
            "t_unix_ns", "t_unix_s",
            "device_id",
            "page", "page_hex",
            "page_name",

            # Common-ish
            "cadence_rpm",

            # ---- Page 0x10 fields ----
            "p10_update_event_count",
            "p10_pedal_power_percent",
            "p10_pedal_power_is_right",   # 1 if bit7 indicates "right pedal power contribution", else 0, NaN if not valid
            "p10_accumulated_power_w",
            "p10_instantaneous_power_w",

            # ---- Page 0x12 fields ----
            "p12_update_event_count",
            "p12_crank_ticks",
            "p12_crank_period_1_2048s",
            "p12_accumulated_torque_1_32nm",
        ])

        # buffering
        self.flush_every_n = int(flush_every_n)
        self.flush_every_s = float(flush_every_s)
        self._rows_since_flush = 0
        self._last_flush_t = time.monotonic()

        # stats
        self.stats_every_s = float(stats_every_s)
        self._last_stat_t = time.monotonic()
        self.page_counter = Counter()

        # pending raw for pairing with page_name
        self._pending_raw: Optional[bytes] = None
        self._pending_page: Optional[int] = None
        self._pending_t_ns: Optional[int] = None

        # hook into decoded callback
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
            print(f"[stats {self.stats_every_s:.0f}s] total={total}, 0x10={c10}, 0x12={c12}, others={others_str}")
            self.page_counter.clear()
            self._last_stat_t = now

    # ---- raw 8 bytes arrive here ----
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

        # continue normal parsing
        return super().on_data(data)

    # ---- decoded callback gives page_name; we log using pending raw ----
    def _on_device_data_logged(self, page: int, page_name: str, parsed):
        # pair with pending raw
        if self._pending_raw is None or self._pending_page is None or self._pending_t_ns is None:
            raw = b"\x00" * 8
            raw_page = page
            t_ns = time.time_ns()
        else:
            raw = self._pending_raw
            raw_page = self._pending_page
            t_ns = self._pending_t_ns

        # only log 0x10 and 0x12
        if raw_page not in (0x10, 0x12):
            self._pending_raw = None
            self._pending_page = None
            self._pending_t_ns = None
            return

        t_s = t_ns / 1e9

        # defaults (NaN where not applicable)
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

        # cadence byte is byte3 on both 0x10 and 0x12
        cad_byte = raw[3]
        if cad_byte != 0xFF:  # 0xFF = invalid/unknown
            cadence = float(cad_byte)

        if raw_page == 0x10:
            # Table: 0x10 Power-Only
            p10_evt = float(raw[1])

            pedal_power = raw[2]
            # bit7 = differentiation (1 means "right pedal contribution", 0 means unknown)
            is_right = (pedal_power >> 7) & 1
            pct = pedal_power & 0x7F  # percent 0..100 typically; 0x7F used as invalid in some contexts
            # We'll log percent as numeric, and is_right as 0/1.
            p10_pp_is_right = float(is_right)
            p10_pp_pct = float(pct)

            p10_acc_pwr = float(u16_le(raw, 4))
            p10_inst_pwr = float(u16_le(raw, 6))

        elif raw_page == 0x12:
            # Table 10-1: 0x12 Crank Torque
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

        # clear pending
        self._pending_raw = None
        self._pending_page = None
        self._pending_t_ns = None


def main():
    POWER_METER_ID = 64434  # change if needed
    out_dir = os.path.expanduser("~/bikelab_interface_logs")
    ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    csv_path = os.path.join(out_dir, f"rally_payload_decoded_{ts}.csv")

    ant = ANTNode()
    ant.set_network_key(0x00, ANTPLUS_NETWORK_KEY)

    pm = PowerMeterPayloadLogger(
        ant_node=ant,
        device_id=POWER_METER_ID,
        csv_path=csv_path,
        flush_every_n=200,
        flush_every_s=2.0,
        stats_every_s=10.0,
    )

    def on_found():
        print(f"[ok] Power Meter found: {pm} (device_id={POWER_METER_ID})")
        print(f"[log] Writing CSV: {csv_path}")

    pm.on_found = on_found

    print("[info] Starting ANT node (Ctrl-C to stop)...")
    try:
        ant.start()
    except KeyboardInterrupt:
        print("\n[info] Stopping...")
    finally:
        try:
            pm.close_channel()
        except Exception:
            pass
        try:
            ant.stop()
        except Exception:
            pass
        try:
            pm.close()
        except Exception:
            pass
        print("[done] CSV closed.")


if __name__ == "__main__":
    main()