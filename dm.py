import os
import sys
import time
import math
import threading
import can

# ==========================================
# 0. 屏蔽警告
# ==========================================
stderr_fileno = sys.stderr.fileno()
dup_stderr = os.dup(stderr_fileno)
null_fd = os.open(os.devnull, os.O_WRONLY)
os.dup2(null_fd, stderr_fileno)
import can
os.dup2(dup_stderr, stderr_fileno)
os.close(null_fd)
os.close(dup_stderr)

# ==========================================
# 1. 硬件映射 (12个电机)
# ==========================================
MOTOR_CONFIG = {}
# CAN0: 显示 01-06
for i in range(1, 7):
    MOTOR_CONFIG[i] = {"bus_idx": 0, "can_id": i, "master_id": i + 0x10, "disp_id": f"{i:02d}"}
# CAN1: 显示 11-16
for i in range(7, 13):
    phys_id = i - 6
    MOTOR_CONFIG[i] = {"bus_idx": 1, "can_id": phys_id, "master_id": phys_id + 0x10, "disp_id": f"{phys_id + 10}"}

MASTER_TO_ID = {(v["bus_idx"], v["master_id"]): k for k, v in MOTOR_CONFIG.items()}

P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -30.0, 30.0
T_MIN, T_MAX = -10.0, 10.0

motor_status = {k: {"p": 0.0, "t": 0.0, "ts": 0.0} for k in MOTOR_CONFIG.keys()}
is_running = True

# ==========================================
# 2. 核心转换工具
# ==========================================
def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return ((float(x_int) * span) / ((1 << bits) - 1)) + x_min

def pack_mit_command(p, v, kp, kd, t):
    def float_to_uint(val, v_min, v_max, bits):
        span = v_max - v_min
        val = max(min(val, v_max), v_min)
        return int(((val - v_min) * ((1 << bits) - 1)) / span)
    pi = float_to_uint(p, P_MIN, P_MAX, 16)
    vi = float_to_uint(v, V_MIN, V_MAX, 12)
    kpi, kdi = float_to_uint(kp, 0, 500, 12), float_to_uint(kd, 0, 5, 12)
    ti = float_to_uint(t, T_MIN, T_MAX, 12)
    d = bytearray(8)
    d[0], d[1] = pi >> 8, pi & 0xFF
    d[2], d[3] = vi >> 4, ((vi & 0xF) << 4) | (kpi >> 8)
    d[4], d[5] = kpi & 0xFF, kdi >> 4
    d[6], d[7] = ((kdi & 0xF) << 4) | (ti >> 8), ti & 0xFF
    return d

def unpack_reply(msg, bus_idx):
    key = (bus_idx, msg.arbitration_id)
    if key in MASTER_TO_ID and len(msg.data) >= 6:
        logical_id = MASTER_TO_ID[key]
        p_int = (msg.data[1] << 8) | msg.data[2]
        t_int = ((msg.data[4] & 0x0F) << 8) | msg.data[5]
        motor_status[logical_id]["p"] = uint_to_float(p_int, P_MIN, P_MAX, 16)
        motor_status[logical_id]["t"] = uint_to_float(t_int, T_MIN, T_MAX, 12)
        motor_status[logical_id]["ts"] = time.time()

def can_rx_thread(bus, bus_idx):
    while is_running:
        try:
            msg = bus.recv(timeout=0.01)
            if msg: unpack_reply(msg, bus_idx)
        except: pass

def send_special(bus, can_id, suffix):
    try:
        bus.send(can.Message(arbitration_id=can_id, data=bytearray([0xFF]*7 + [suffix]), is_extended_id=False), timeout=0.001)
    except: pass

# ==========================================
# 4. 主程序
# ==========================================
if __name__ == "__main__":
    buses = []
    try:
        buses.append(can.Bus(interface='socketcan', channel='can0', bitrate=1000000, fd=True))
        buses.append(can.Bus(interface='socketcan', channel='can1', bitrate=1000000, fd=True))
        sys.stdout.write("✔ CAN0/CAN1 就绪\n")
    except Exception as e:
        print(f"✘ 初始化失败: {e}"); sys.exit()

    for i, b in enumerate(buses):
        threading.Thread(target=can_rx_thread, args=(b, i), daemon=True).start()

    try:
        sys.stdout.write("正在使能并校准零点...")
        sys.stdout.flush()
        for cfg in MOTOR_CONFIG.values(): send_special(buses[cfg["bus_idx"]], cfg["can_id"], 0xFC)
        time.sleep(0.4)
        for cfg in MOTOR_CONFIG.values(): send_special(buses[cfg["bus_idx"]], cfg["can_id"], 0xFE)
        time.sleep(0.1)
        sys.stdout.write(" 完成\n") # 紧跟使能提示后换行
        sys.stdout.write("显示CAN通道(0,1)、数字电机ID(1-6)：位置/扭矩\n") # 紧跟使能提示后换行
        start_t = time.time()
        last_disp = 0
        next_loop = start_t + 0.001

        while True:
            now = time.time()
            t_rel = now - start_t
            tp = 0.5 * math.sin(1.25 * t_rel)
            tv = 0.6 * math.cos(1.25 * t_rel)
            
            for mid, cfg in MOTOR_CONFIG.items():
                dir_m = 1 if mid <= 6 else -1
                cmd = pack_mit_command(tp * dir_m, tv * dir_m, 30.0, 1.0, 0.0)
                try:
                    buses[cfg["bus_idx"]].send(can.Message(arbitration_id=cfg["can_id"], data=cmd, is_extended_id=False), timeout=0.0005)
                except: pass

            # --- 双行紧凑显示逻辑 ---
            if now - last_disp >= 0.1:
                # 第一行 CAN0 (01-06), 第二行 CAN1 (11-16)
                groups = [range(1, 7), range(7, 13)]
                lines = []
                for group in groups:
                    segs = []
                    for i in group:
                        m = motor_status[i]
                        d_id = MOTOR_CONFIG[i]["disp_id"]
                        if now - m['ts'] < 0.5:
                            segs.append(f"{d_id}:{m['p']:>5.1f}/{m['t']:>4.1f}")
                        else:
                            segs.append(f"{d_id}:  ---/ ---")
                    lines.append(" | ".join(segs))
                
                # \033[K 清除当前行, \n 换行, \033[F 回到上一行 (保持光标位置在输出块首)
                sys.stdout.write(f"\r\033[K{lines[0]}\n\033[K{lines[1]}\033[F")
                sys.stdout.flush()
                last_disp = now

            wait = next_loop - time.time()
            if wait > 0: time.sleep(wait)
            next_loop += 0.001

    except KeyboardInterrupt:
        # 退出时跳出覆盖区域，避免重叠
        sys.stdout.write("\n\n停止控制...\n")
    finally:
        is_running = False
        for cfg in MOTOR_CONFIG.values(): send_special(buses[cfg["bus_idx"]], cfg["can_id"], 0xFD)
        for b in buses: b.shutdown()
        print("✔ 资源已释放")
