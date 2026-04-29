#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
蜘蛛机器人底层共享内存实时曲线绘制工具 (基于 matplotlib)

示例用法:
  python plot_hw_shm.py --duration 10 --hz 50
  python plot_hw_shm.py --signals cmd_p_00 state_p_00 cmd_t_00 state_t_00

可选信号名称说明 (索引 00~11 对应 12 个电机):
  - cmd_p_00 .. cmd_p_11    (目标位置)
  - cmd_v_00 .. cmd_v_11    (目标速度)
  - cmd_t_00 .. cmd_t_11    (前馈力矩)
  - cmd_kp_00 .. cmd_kp_11  (位置刚度 Kp)
  - cmd_kd_00 .. cmd_kd_11  (速度阻尼 Kd)
  
  - state_p_00 .. state_p_11 (实际反馈位置)
  - state_v_00 .. state_v_11 (实际反馈速度)
  - state_t_00 .. state_t_11 (实际反馈力矩)
"""

from __future__ import annotations

import argparse
import time
import ctypes
from dataclasses import dataclass
from typing import Dict, List, Tuple
from multiprocessing import shared_memory

import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. 共享内存结构定义 (与 dm_hw.py 一致)
# ==========================================
class MotorData(ctypes.Structure):
    _fields_ = [("p", ctypes.c_float), ("v", ctypes.c_float), ("t", ctypes.c_float), 
                ("kp", ctypes.c_float), ("kd", ctypes.c_float)]

class SharedRobotData(ctypes.Structure):
    _fields_ = [("cmd", MotorData * 12), ("state", MotorData * 12), ("is_running", ctypes.c_bool)]

SHM_NAME = "spider_robot_shm"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="实时绘制蜘蛛机器人共享内存数据")
    p.add_argument("--hz", type=float, default=50.0, help="绘图刷新率 (Hz)")
    p.add_argument("--duration", type=float, default=10.0, help="X轴时间窗口长度 (秒)")
    p.add_argument(
        "--signals",
        nargs="*",
        default=["cmd_p_00", "state_p_00", "cmd_p_01", "state_p_01"],
        help="需要绘制的信号列表",
    )
    p.add_argument("--wait", action="store_true", default=True, help="如果共享内存未就绪，则保持等待")
    return p.parse_args()


def _connect_shm(wait: bool) -> Tuple[shared_memory.SharedMemory, SharedRobotData]:
    printed = False
    while True:
        try:
            shm_obj = shared_memory.SharedMemory(name=SHM_NAME)
            robot_data = SharedRobotData.from_buffer(shm_obj.buf)
            return shm_obj, robot_data
        except FileNotFoundError:
            if not wait:
                raise FileNotFoundError(f"找不到共享内存 '{SHM_NAME}'")
            if not printed:
                print(f"[SHM_PLOT] 等待共享内存 '{SHM_NAME}' 就绪 (请先运行 moto.py)...", flush=True)
                printed = True
            time.sleep(0.5)


def _get_from_sample(sample: Dict[str, float], key: str) -> float:
    return float(sample.get(key, float("nan")))


def _build_sample(robot_data: SharedRobotData) -> Dict[str, float]:
    """将 CTypes 结构体转换为扁平的字典，方便提取变量"""
    out: Dict[str, float] = {}

    for i in range(12):
        # 记录指令数据 (Command)
        out[f"cmd_p_{i:02d}"] = float(robot_data.cmd[i].p)
        out[f"cmd_v_{i:02d}"] = float(robot_data.cmd[i].v)
        out[f"cmd_t_{i:02d}"] = float(robot_data.cmd[i].t)
        out[f"cmd_kp_{i:02d}"] = float(robot_data.cmd[i].kp)
        out[f"cmd_kd_{i:02d}"] = float(robot_data.cmd[i].kd)

        # 记录状态数据 (State)
        out[f"state_p_{i:02d}"] = float(robot_data.state[i].p)
        out[f"state_v_{i:02d}"] = float(robot_data.state[i].v)
        out[f"state_t_{i:02d}"] = float(robot_data.state[i].t)

    return out


def main() -> None:
    args = parse_args()

    # 1. 连接共享内存
    shm_obj, robot_data = _connect_shm(wait=args.wait)
    print(f"✔ 成功连接共享内存。开始绘制信号: {args.signals}")

    # 2. 图表参数配置
    hz = float(args.hz) if float(args.hz) > 0 else 50.0
    period = 1.0 / hz
    duration = max(2.0, float(args.duration))
    capacity = int(duration * hz) + 1

    signals: List[str] = list(args.signals)

    # 3. 初始化 matplotlib 画布
    plt.ion() # 开启交互模式
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_title("Spider Robot Hardware SHM Live Plot")
    ax.set_xlabel("Time (s, relative)")
    ax.grid(True)

    t0 = time.time()
    ts: List[float] = []
    ys: Dict[str, List[float]] = {s: [] for s in signals}

    lines = {}
    for s in signals:
        (line,) = ax.plot([], [], label=s, linewidth=1.5)
        lines[s] = line
    ax.legend(loc="upper right")

    try:
        # 当硬件停止运行时，自动退出循环；或捕捉 Ctrl+C
        while robot_data.is_running:
            now = time.time()
            # 提取全量数据
            sample = _build_sample(robot_data)

            t_rel = now - t0
            ts.append(t_rel)
            if len(ts) > capacity:
                ts.pop(0)

            # 更新数据缓存
            for s in signals:
                ys[s].append(_get_from_sample(sample, s))
                if len(ys[s]) > capacity:
                    ys[s].pop(0)

            # 更新图表折线
            for s in signals:
                lines[s].set_data(ts, ys[s])

            # 动态调整 X 轴时间窗
            if ts:
                ax.set_xlim(max(0.0, ts[-1] - duration), ts[-1])

            # 动态调整 Y 轴缩放
            all_vals = []
            for s in signals:
                all_vals.extend([v for v in ys[s] if np.isfinite(v)])
            if all_vals:
                vmin = float(np.min(all_vals))
                vmax = float(np.max(all_vals))
                if vmin == vmax:
                    vmin -= 1.0
                    vmax += 1.0
                margin = 0.05 * (vmax - vmin)
                ax.set_ylim(vmin - margin, vmax + margin)

            # 渲染画面
            fig.canvas.draw()
            fig.canvas.flush_events()

            # 补偿休眠时间，控制刷新率
            elapsed = time.time() - now
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[INFO] 检测到手动退出。")
    finally:
        print("正在清理绘图资源...")
        # 仅解除引用，不销毁实际内存（让硬件进程自行管理）
        del robot_data
        try:
            shm_obj.close()
        except Exception:
            pass
        plt.ioff()
        plt.show() # 退出前保留最后一张静态图
        print("✔ 绘图已结束。")

if __name__ == "__main__":
    main()

