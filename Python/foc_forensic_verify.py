#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
foc_forensic_verify.py — 取证级调试：固件转速反馈标度独立验证

对应 README 1.2 节调试方法论中的"取证级调试"：
  同一串口流内同时解析固件转速反馈(ch4)和固件输出电角度(ch5)，
  用净转数法独立验证反馈标度，定位标度误差或低速锁死等问题。

原理：
  保持转矩模式恒流输出，采集原始串口流，每0.5秒分片统计：
    - net_elec_rev  : 电角度(ch5)解卷绕后的净转数
    - rpm_angle     : 由电角度净转数折算的真值转速（不依赖固件反馈）
    - rpm_firmware  : 固件自报转速(ch4 = Motor_GetSpeedFB())
    - ratio         : rpm_firmware / rpm_angle
  若固件自洽，ratio -> 1；若 ratio 为稳定常数，说明反馈存在固定比例标度误差。
  同时保留原始 .dat 文件，同一流可随时重新解码，无需重新跑电机。

本脚本的实际贡献：
  定位了"反馈链5x增益误差"的误判——实际根因是低速锁死bug
  （角度助推被一行 return 0 关掉），用净转数法证明了反馈标度本身正确。

依赖：
  pip install pyserial

用法：
  python foc_forensic_verify.py [iq] [dwell_s]
  python foc_forensic_verify.py 0.25 6.0    # 转矩0.25A，采集6秒
"""
import csv
import os
import struct
import sys
import time

import serial

TAIL = b"\x00\x00\x80\x7f"
FL = 28
POLE_PAIRS = 4.0
IQ = float(sys.argv[1]) if len(sys.argv) > 1 else 0.25
DWELL = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0


def phase_lock(raw, fl, need=6):
    p = raw.find(TAIL)
    while p >= 0:
        c = p + 4 - fl
        if c >= 0 and all(c + i * fl + fl <= len(raw) and
                          raw[c + i * fl + fl - 4:c + i * fl + fl] == TAIL
                          for i in range(need)):
            return c
        p = raw.find(TAIL, p + 1)
    return -1


def frames_of(raw):
    st = phase_lock(raw, FL)
    if st < 0:
        return []
    out, k = [], st
    while k + FL <= len(raw):
        if raw[k + FL - 4:k + FL] == TAIL:
            v = struct.unpack("<6f", raw[k:k + FL - 4])
            if all(x == x and abs(x) < 1e7 for x in v):
                out.append(v)
            k += FL
        else:
            nx = phase_lock(raw[k:], FL)
            if nx < 0:
                break
            k += nx
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    d = os.path.abspath(os.path.join(here, "..", "_data"))
    os.makedirs(d, exist_ok=True)
    tag = "fwproof_%s" % time.strftime("%Y%m%d_%H%M%S")
    s = serial.Serial("COM12", 921600, timeout=0.05)
    s.reset_input_buffer()
    time.sleep(0.2)
    s.write(b"m0\n"); time.sleep(0.4); s.write(b"iq=0\n"); time.sleep(1.0)
    s.write(("iq=%.3f\n" % IQ).encode())
    t0 = time.perf_counter()
    raw = bytearray()
    while time.perf_counter() - t0 < DWELL:
        c = s.read(max(1, s.in_waiting or 16384))
        if c:
            raw.extend(c)
    t1 = time.perf_counter()
    s.write(b"iq=0\n"); time.sleep(0.4)
    s.close()
    with open(os.path.join(d, tag + ".dat"), "wb") as fh:
        fh.write(bytes(raw))
    fr = frames_of(bytes(raw))
    if len(fr) < 100:
        print("no frames"); return 1
    dt_frame = (t1 - t0) / len(fr)
    fps = 1.0 / dt_frame
    print("iq=%.3fA  dwell=%.2fs  raw=%d B  frames=%d  fps=%.0f  dt=%.4f ms" % (
        IQ, t1 - t0, len(raw), len(fr), fps, dt_frame * 1000))
    th = [v[5] for v in fr]
    # per-frame unwrapped deltas (electrical degrees)
    dd = []
    for i in range(1, len(th)):
        x = (th[i] - th[i - 1]) % 360.0
        if x > 180.0:
            x -= 360.0
        dd.append(x)
    dabs = sum(abs(x) for x in dd) / len(dd)
    hdr = ("  t(s)  n_fr  net_elec_rev  mech_rev  rpm_angle  rpm_firmware  ratio"
           "   rpm_from_|d|  Iq_act")
    print(hdr); print("-" * len(hdr))
    K = int(0.5 / dt_frame)          # frames per slice
    rows = []
    for k in range(0, len(fr), K):
        seg = fr[k:k + K]
        if len(seg) < 20:
            break
        sub = [v[5] for v in seg]
        sdd = []
        for i in range(1, len(sub)):
            x = (sub[i] - sub[i - 1]) % 360.0
            if x > 180.0:
                x -= 360.0
            sdd.append(x)
        net_e = sum(sdd) / 360.0
        secs = (len(seg) - 1) * dt_frame
        mech = net_e / POLE_PAIRS
        rpm_a = mech * 60.0 / secs if secs else 0.0
        rpm_f = sum(v[4] for v in seg) / len(seg)
        rpm_abs = (sum(abs(x) for x in sdd) / len(sdd)) * fps * 60.0 / (
            POLE_PAIRS * 360.0)
        ratio = rpm_f / rpm_a if abs(rpm_a) > 5.0 else float("nan")
        iqa = sum(v[3] for v in seg) / len(seg)
        t = (k + len(seg) / 2.0) * dt_frame
        rows.append((t, len(seg), net_e, mech, rpm_a, rpm_f, ratio, rpm_abs, iqa))
        print("%6.2f %5d %12.3f %9.3f %10.2f %13.2f %7.2f %12.2f %8.4f" % (
            t, len(seg), net_e, mech, rpm_a, rpm_f, ratio, rpm_abs, iqa))
    with open(os.path.join(d, tag + "_slices.csv"), "w", newline="",
              encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "n_frames", "net_elec_rev", "mech_rev", "rpm_angle",
                    "rpm_firmware", "ratio", "rpm_from_abs_delta", "Iq_act"])
        for r in rows:
            w.writerow(["%.3f" % r[0], r[1], "%.4f" % r[2], "%.4f" % r[3],
                        "%.3f" % r[4], "%.3f" % r[5],
                        ("" if r[6] != r[6] else "%.4f" % r[6]),
                        "%.3f" % r[7], "%.5f" % r[8]])
    print("\nmean |delta| = %.4f deg/frame  ->  %.2f rpm if all motion were one-way"
          % (dabs, dabs * fps * 60.0 / (POLE_PAIRS * 360.0)))
    print("saved: %s\\%s.dat + _slices.csv" % (d, tag))
    return 0


if __name__ == "__main__":
    sys.exit(main())
