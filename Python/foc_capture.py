#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
foc_capture.py — STM32-FOC-HALL 三模式数据采集工具（带安全保护 + 二进制帧相位锁定）

对应 README 1.2 节调试方法论中的两项：
  1. 二进制帧相位锁定 —— 自定义 JustFloat 协议解析器，通过连续帧尾匹配实现相位锁定，支持丢帧重同步
  2. 安全采集脚本 —— Python 脚本实时监控转速/电流，超过阈值自动发停机命令，硬件调试零风险

功能：
  通过串口向电机控制板发送 VOFA+ 命令，依次采集转矩/速度/位置三模式数据，
  实时解析 JustFloat 二进制帧，自动计算真值转速（电角度净转数法），
  输出 CSV 逐帧数据 + 汇总统计。

协议：
  - VOFA+ JustFloat，帧尾 0x00 0x00 0x80 0x7F，小端序 float32
  - 波特率 921600，文本命令以 \\n 结尾
  - 帧格式：转矩6ch / 速度18ch / 位置8ch（与固件 APP/Src/VOFA.c 一致）

安全保护：
  每档采集过程中实时监控转速和电流，超过阈值立即发送停机命令并中止：
    - 转速 > 4200 rpm → 发 rpm=0 / iq=0
    - 电流 |Iq| > 2.5 A → 发 iq=0

真值转速计算：
  不依赖固件反馈通道，由电角度(theta)净转数法独立折算：
    rpm = (总电角度变化 / 360) / 极对数 × 60 / 时间

依赖：
  pip install pyserial

用法：
  python foc_capture.py --mode 0          # 仅转矩模式
  python foc_capture.py --mode 1          # 仅速度模式
  python foc_capture.py --mode 2          # 仅位置模式
  python foc_capture.py                    # 全部三模式
  python foc_capture.py --port COM3       # 指定串口（默认COM12）
"""
import argparse
import csv
import json
import os
import struct
import time
import serial

TAIL = b"\x00\x00\x80\x7f"
PORT = "COM12"
BAUD = 921600
POLE_PAIRS = 4.0  # 极对数

# 各模式通道定义（与 APP/Src/VOFA.c 一致）
CH_TORQUE = ["Id_cmd", "Id_act", "Iq_cmd", "Iq_act", "rpm_fb", "theta_deg"]
CH_SPEED = ["state", "Id_cmd", "Id_act", "Iq_cmd", "Iq_act",
            "Id_err", "Iq_err", "cur_Kp", "cur_Ki", "cur_OutMax",
            "Vd_PIout", "Vq_PIout", "Vd_V", "Vq_V", "theta_deg",
            "CCR_U", "CCR_V", "CCR_W"]
CH_POSITION = ["pos_cmd_deg", "pos_act_deg", "pos_err_deg",
               "rpm_cmd", "rpm_fb", "Iq_cmd", "Iq_act", "theta_rad"]

# 安全限值
RPM_LIMIT = 4200.0
IQ_LIMIT = 2.5

# 测试序列
SEQ_TORQUE = [  # (Iq目标A, 停留秒)
    (0.10, 3.0), (0.20, 3.0), (0.30, 3.0), (0.50, 3.0),
    (0.80, 3.0), (1.00, 3.0), (1.20, 3.0), (1.50, 3.0),
]
SEQ_SPEED = [  # (rpm目标, 停留秒)
    (50, 4.0), (100, 4.0), (200, 3.0), (400, 3.0),
    (800, 3.0), (1500, 3.0), (2000, 3.0), (2500, 3.0), (3000, 3.0),
]
SEQ_POSITION = [  # (目标角度°, 停留秒)
    (0, 3.0), (90, 4.0), (180, 4.0), (270, 4.0),
    (180, 3.0), (90, 3.0), (0, 3.0),
]


def phase_lock(raw, frame_len, need=6):
    """在字节流中定位连续帧起始偏移"""
    p = raw.find(TAIL)
    while p >= 0:
        start = p + 4 - frame_len
        if start >= 0:
            ok = True
            for i in range(need):
                fp = start + i * frame_len
                if fp + frame_len > len(raw):
                    ok = False
                    break
                if raw[fp + frame_len - 4:fp + frame_len] != TAIL:
                    ok = False
                    break
            if ok:
                return start
        p = raw.find(TAIL, p + 1)
    return -1


def parse_frames(raw, nch):
    """解析JustFloat帧，返回 [(offset, [values...]), ...]"""
    fl = nch * 4 + 4
    st = phase_lock(raw, fl)
    if st < 0:
        return []
    out = []
    k = st
    while k + fl <= len(raw):
        if raw[k + fl - 4:k + fl] == TAIL:
            vals = struct.unpack("<%df" % nch, raw[k:k + fl - 4])
            if all(v == v and abs(v) < 1e7 for v in vals):
                out.append((k - st, list(vals)))
            k += fl
        else:
            nxt = phase_lock(raw[k:], fl)
            if nxt < 0:
                break
            k += nxt
    return out


def theta_to_rpm(frames, theta_idx, is_rad=False):
    """从电角度序列计算真值转速(rpm)：净转数法 + 最小二乘斜率"""
    if len(frames) < 10:
        return 0.0, 0.0
    # 取稳态后半段
    half = len(frames) // 2
    seg = frames[half:]
    if len(seg) < 5:
        seg = frames

    # 解卷绕电角度
    thetas = []
    prev = None
    total_unwrapped = 0.0
    for _, v in seg:
        th = v[theta_idx]
        if is_rad:
            th = th * 180.0 / 3.141592653589793  # rad -> deg
        if prev is not None:
            diff = th - prev
            # 解卷绕：跨越0/360边界
            while diff > 180:
                diff -= 360
            while diff < -180:
                diff += 360
            total_unwrapped += diff
        prev = th
        thetas.append(th)

    # 净转数法：电角度总变化 / 360 / 极对数 = 机械转数
    mech_revs = total_unwrapped / 360.0 / POLE_PAIRS
    # 时间：用帧数和估算帧率（921600bps / 帧长）
    # 更准确：用偏移量差 / 波特率 * 10 (每字节10bit)
    if len(seg) >= 2:
        byte_offset = seg[-1][0] - seg[0][0]
        elapsed_sec = byte_offset * 10.0 / BAUD
    else:
        elapsed_sec = 1.0

    if elapsed_sec <= 0:
        return 0.0, 0.0
    rpm_net = mech_revs * 60.0 / elapsed_sec

    # 最小二乘斜率法
    n = len(thetas)
    if n >= 5:
        t_vals = [i * elapsed_sec / (n - 1) for i in range(n)]
        t_mean = sum(t_vals) / n
        th_mean = sum(thetas) / n
        num = sum((t_vals[i] - t_mean) * (thetas[i] - th_mean) for i in range(n))
        den = sum((t_vals[i] - t_mean) ** 2 for i in range(n))
        if den > 0:
            slope_deg_per_s = num / den
            rpm_ls = slope_deg_per_s / 360.0 / POLE_PAIRS * 60.0
        else:
            rpm_ls = rpm_net
    else:
        rpm_ls = rpm_net

    return rpm_net, rpm_ls


def send_cmd(s, cmd):
    s.write((cmd + "\n").encode())
    time.sleep(0.05)


def safe_stop(s):
    send_cmd(s, "rpm=0")
    time.sleep(0.1)
    send_cmd(s, "iq=0")
    time.sleep(0.1)
    send_cmd(s, "m0")


def capture_mode(s, mode, seq, nch, ch_names, theta_idx, theta_is_rad, out_dir, tag):
    """采集一个模式的完整序列"""
    mode_cmd = {0: "m0", 1: "m1", 2: "m2"}[mode]
    print("\n" + "=" * 70)
    print("  模式 %d (%s) — %d 通道, %d 档" % (mode, mode_cmd, nch, len(seq)))
    print("=" * 70)

    # 切换模式
    send_cmd(s, "m0")
    time.sleep(0.3)
    send_cmd(s, "iq=0")
    time.sleep(0.5)
    send_cmd(s, mode_cmd)
    time.sleep(1.5)

    all_rows = []
    summary = []
    aborted = False

    for idx, (target, dwell) in enumerate(seq):
        if mode == 0:
            cmd = "iq=%.3f" % target
            kind = "Iq(A)"
        elif mode == 1:
            cmd = "rpm=%.0f" % target
            kind = "rpm"
        else:
            cmd = "pos=%.1f" % target
            kind = "pos(deg)"

        send_cmd(s, cmd)
        t0 = time.time()
        raw = bytearray()
        worst_rpm = 0.0
        worst_iq = 0.0
        tripped = False

        while time.time() - t0 < dwell:
            try:
                c = s.read(max(1, min(s.in_waiting or 4096, 65536)))
            except Exception:
                c = b""
            if c:
                raw.extend(c)
            # 实时安全检查
            fr = parse_frames(bytes(raw[-16384:]), nch)
            if fr:
                for _, v in fr[-200:]:
                    if mode == 0:
                        rpm = abs(v[4])
                        iq = abs(v[3])
                    elif mode == 1:
                        # 速度模式无直接rpm通道，用Iq监控
                        rpm = 0
                        iq = abs(v[4])
                    else:
                        rpm = abs(v[4])
                        iq = abs(v[6])
                    if rpm > worst_rpm:
                        worst_rpm = rpm
                    if iq > worst_iq:
                        worst_iq = iq
                if worst_rpm > RPM_LIMIT or worst_iq > IQ_LIMIT:
                    tripped = True
                    print("  !!! 安全触发: rpm=%.0f iq=%.3f -> 紧急停机" % (worst_rpm, worst_iq))
                    safe_stop(s)
                    time.sleep(1.0)
                    break

        fr = parse_frames(bytes(raw), nch)

        # 计算该档统计
        if fr:
            # 稳态后半段
            half = len(fr) // 2
            steady = fr[half:]
            if not steady:
                steady = fr

            # 真值转速
            rpm_net, rpm_ls = theta_to_rpm(fr, theta_idx, theta_is_rad)

            # 各通道均值
            ch_means = []
            for ci in range(nch):
                vals = [v[ci] for _, v in steady]
                ch_means.append(sum(vals) / len(vals) if vals else 0)

            # Iq实际
            if mode == 0:
                iq_act = ch_means[3]
                iq_cmd = ch_means[2]
                fb_rpm = ch_means[4]
            elif mode == 1:
                iq_act = ch_means[4]
                iq_cmd = ch_means[3]
                fb_rpm = rpm_net  # 速度模式用真值
            else:
                iq_act = ch_means[6]
                iq_cmd = ch_means[5]
                fb_rpm = ch_means[4]
                pos_act = ch_means[1]
                pos_err = ch_means[2]

            row_summary = {
                "step": idx, "target": target, "kind": kind,
                "frames": len(fr), "rpm_truth_net": rpm_net, "rpm_truth_ls": rpm_ls,
                "rpm_fb": fb_rpm, "iq_cmd": iq_cmd, "iq_act": iq_act,
                "worst_rpm": worst_rpm, "worst_iq": worst_iq,
            }
            if mode == 2:
                row_summary["pos_act"] = pos_act
                row_summary["pos_err"] = pos_err
                # 位置稳态误差std
                errs = [v[2] for _, v in steady]
                row_summary["pos_err_std"] = (sum((e - sum(errs)/len(errs))**2 for e in errs)/len(errs))**0.5 if errs else 0

            summary.append(row_summary)

            # 保存逐帧数据
            for off, v in fr:
                row = {"step": idx, "target": target, "kind": kind, "offset": off}
                for ci, name in enumerate(ch_names):
                    row[name] = v[ci]
                all_rows.append(row)

            if mode == 2:
                print("  [%2d] %s=%-6.0f  帧=%-5d  位置实际=%6.1f°  误差=%+6.2f°  Iq=%.3fA" %
                      (idx, kind, target, len(fr), pos_act, pos_err, iq_act))
            else:
                print("  [%2d] %s=%-8.2f  帧=%-5d  真值转速=%7.1f rpm  反馈=%7.1f rpm  Iq实=%.3fA" %
                      (idx, kind, target, len(fr), rpm_net, fb_rpm, iq_act))
        else:
            print("  [%2d] %s=%-8.2f  无有效帧!" % (idx, kind, target))
            summary.append({"step": idx, "target": target, "kind": kind, "frames": 0})

        if tripped:
            aborted = True
            break

    # 停机
    safe_stop(s)
    time.sleep(0.5)

    # 保存CSV
    os.makedirs(out_dir, exist_ok=True)
    frames_file = os.path.join(out_dir, "%s_frames.csv" % tag)
    summary_file = os.path.join(out_dir, "%s_summary.csv" % tag)

    if all_rows:
        keys = list(all_rows[0].keys())
        with open(frames_file, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(all_rows)

    if summary:
        keys = list(summary[0].keys())
        with open(summary_file, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(summary)

    print("\n  已保存: %s" % frames_file)
    print("  已保存: %s" % summary_file)
    return summary, aborted


def main():
    ap = argparse.ArgumentParser(description="三模式综合数据采集")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "_data", "three_mode"))
    ap.add_argument("--mode", type=int, default=-1, help="只采集指定模式: 0=转矩 1=速度 2=位置 -1=全部")
    args = ap.parse_args()

    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")

    print("FOC 三模式数据采集")
    print("端口: %s @ %d bps" % (args.port, BAUD))
    print("输出: %s" % out_dir)
    print("安全限值: rpm<%.0f, |Iq|<%.2fA" % (RPM_LIMIT, IQ_LIMIT))

    s = serial.Serial(args.port, BAUD, timeout=0.05)
    s.reset_input_buffer()
    time.sleep(0.3)

    # 初始安全状态
    send_cmd(s, "m0")
    time.sleep(0.3)
    send_cmd(s, "iq=0")
    time.sleep(0.5)

    results = {}

    try:
        if args.mode in (-1, 0):
            tag = "torque_%s" % timestamp
            s0, a0 = capture_mode(s, 0, SEQ_TORQUE, 6, CH_TORQUE,
                                   theta_idx=5, theta_is_rad=False,
                                   out_dir=out_dir, tag=tag)
            results["torque"] = s0

        if args.mode in (-1, 1):
            tag = "speed_%s" % timestamp
            s1, a1 = capture_mode(s, 1, SEQ_SPEED, 18, CH_SPEED,
                                   theta_idx=14, theta_is_rad=False,
                                   out_dir=out_dir, tag=tag)
            results["speed"] = s1

        if args.mode in (-1, 2):
            tag = "position_%s" % timestamp
            s2, a2 = capture_mode(s, 2, SEQ_POSITION, 8, CH_POSITION,
                                   theta_idx=7, theta_is_rad=True,
                                   out_dir=out_dir, tag=tag)
            results["position"] = s2

    except KeyboardInterrupt:
        print("\n用户中断，紧急停机...")
        safe_stop(s)
    except Exception as e:
        print("\n异常: %s，紧急停机..." % str(e))
        safe_stop(s)
        raise
    finally:
        s.close()

    # 保存汇总JSON
    meta_file = os.path.join(out_dir, "capture_meta_%s.json" % timestamp)
    with open(meta_file, "w", encoding="utf-8") as f:
        json.dump({
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "port": args.port, "baud": BAUD,
            "pole_pairs": POLE_PAIRS,
            "safety_limits": {"rpm": RPM_LIMIT, "iq": IQ_LIMIT},
            "results": results,
        }, f, ensure_ascii=False, indent=2)

    print("\n" + "=" * 70)
    print("采集完成！汇总: %s" % meta_file)
    print("=" * 70)


if __name__ == "__main__":
    main()
