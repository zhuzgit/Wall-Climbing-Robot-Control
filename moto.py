import os
import sys
import time
import threading
import subprocess
import math
from multiprocessing import shared_memory
import ctypes
import warnings

# ==========================================
# 0. 环境与屏蔽
# ==========================================
warnings.filterwarnings("ignore")

# 屏蔽 python-can 内部的 stderr 警告（保持界面干净）
stderr_fileno = sys.stderr.fileno()
dup_stderr = os.dup(stderr_fileno)
null_fd = os.open(os.devnull, os.O_WRONLY)
os.dup2(null_fd, stderr_fileno)
import can
os.dup2(dup_stderr, stderr_fileno)
os.close(null_fd)
os.close(dup_stderr)

# ==========================================
# 1. 共享内存结构
# ==========================================
class MotorData(ctypes.Structure):
    _fields_ = [("p", ctypes.c_float), ("v", ctypes.c_float), ("t", ctypes.c_float), 
                ("kp", ctypes.c_float), ("kd", ctypes.c_float)]

class SharedRobotData(ctypes.Structure):
    _fields_ = [("cmd", MotorData * 12), ("state", MotorData * 12), ("is_running", ctypes.c_bool)]

SHM_NAME = "spider_robot_shm"
SHM_SIZE = ctypes.sizeof(SharedRobotData)

# ==========================================
# 2. 硬件参数
# ==========================================
MOTOR_CONFIG = {}
for i in range(1, 7): 
    MOTOR_CONFIG[i] = {"bus_idx": 0, "can_id": i, "master_id": i + 0x10, "disp_id": f"{i:02d}"}
for i in range(7, 13): 
    phys_id = i - 6
    MOTOR_CONFIG[i] = {"bus_idx": 1, "can_id": phys_id, "master_id": phys_id + 0x10, "disp_id": f"{phys_id + 10}"}

MASTER_TO_ID = {(v["bus_idx"], v["master_id"]): k for k, v in MOTOR_CONFIG.items()}
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -30.0, 30.0
T_MIN, T_MAX = -10.0, 10.0

def init_can():
    print("正在配置 CAN 接口...")
    cmds = [
        "sudo ip link set can0 down", "sudo ip link set can1 down",
        "sudo ip link set can0 type can bitrate 1000000 fd off",
        "sudo ip link set can1 type can bitrate 1000000 fd off",
        "sudo ip link set can0 up", "sudo ip link set can1 up"
    ]
    for c in cmds: subprocess.run(c, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("✔ CAN 接口配置完成")

def uint_to_float(x_int, x_min, x_max, bits):
    return (float(x_int) * (x_max - x_min) / ((1 << bits) - 1)) + x_min

def pack_mit(p, v, kp, kd, t):
    def f2u(val, v_min, v_max, bits):
        val = max(min(val, v_max), v_min)
        return int(((val - v_min) * ((1 << bits) - 1)) / (v_max - v_min))
    pi, vi = f2u(p, P_MIN, P_MAX, 16), f2u(v, V_MIN, V_MAX, 12)
    kpi, kdi = f2u(kp, 0, 500, 12), f2u(kd, 0, 5, 12)
    ti = f2u(t, T_MIN, T_MAX, 12)
    d = bytearray(8)
    d[0], d[1] = pi >> 8, pi & 0xFF
    d[2], d[3] = vi >> 4, ((vi & 0xF) << 4) | (kpi >> 8)
    d[4], d[5] = kpi & 0xFF, kdi >> 4
    d[6], d[7] = ((kdi & 0xF) << 4) | (ti >> 8), ti & 0xFF
    return d

motor_ts = {i: 0.0 for i in range(1, 13)}

def unpack_reply(msg, bus_idx, shm):
    key = (bus_idx, msg.arbitration_id)
    if key in MASTER_TO_ID and len(msg.data) >= 6:
        idx = MASTER_TO_ID[key]
        p_int = (msg.data[1] << 8) | msg.data[2]
        t_int = ((msg.data[4] & 0x0F) << 8) | msg.data[5]
        shm.state[idx-1].p = uint_to_float(p_int, P_MIN, P_MAX, 16)
        shm.state[idx-1].t = uint_to_float(t_int, T_MIN, T_MAX, 12)
        motor_ts[idx] = time.time()

if __name__ == "__main__":
    init_can()
    
    # 共享内存初始化
    try:
        shm_obj = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
    except FileExistsError:
        shm_obj = shared_memory.SharedMemory(name=SHM_NAME, create=False)

    
    robot_data = SharedRobotData.from_buffer(shm_obj.buf)
    robot_data.is_running = True
  
    
    # ★ 重要：初始化默认控制参数，防止 logic 端没启动时电机乱晃
    for i in range(12):
        robot_data.cmd[i].p = 0.0
        robot_data.cmd[i].v = 0.0
        robot_data.cmd[i].kp = 0.0
        robot_data.cmd[i].kd = 1.0 # 提供基础阻尼
        robot_data.cmd[i].t = 0.0

    buses = [can.Bus(interface='socketcan', channel='can0', bitrate=1000000, fd=False),
             can.Bus(interface='socketcan', channel='can1', bitrate=1000000, fd=False)]

    def rx_loop(bus, idx):
        while robot_data.is_running:
            try:
                m = bus.recv(timeout=0.01)
                if m: unpack_reply(m, idx, robot_data)
            except: pass
    
    for i, b in enumerate(buses): threading.Thread(target=rx_loop, args=(b, i), daemon=True).start()

    def send_sp(bus, cid, sfx):
        try:
            bus.send(can.Message(arbitration_id=cid, data=bytearray([0xFF]*7+[sfx]), is_extended_id=False), timeout=0.001)
        except: pass

    # 依照 test_motor.py 的成功顺序：先使能，再校零
    print("正在使能电机并校准零点...")
    for c in MOTOR_CONFIG.values(): send_sp(buses[c["bus_idx"]], c["can_id"], 0xFC) # 使能
    time.sleep(0.4)
    for c in MOTOR_CONFIG.values(): send_sp(buses[c["bus_idx"]], c["can_id"], 0xFE) # 零点
    time.sleep(0.1)
    for c in MOTOR_CONFIG.values(): send_sp(buses[c["bus_idx"]], c["can_id"], 0xFE) # 零点
    time.sleep(0.1)
    for c in MOTOR_CONFIG.values(): send_sp(buses[c["bus_idx"]], c["can_id"], 0xFE) # 零点
    time.sleep(0.1)




    print("✔ 硬件底层已就绪！")
    print("显示格式：ID: 位置/扭矩")

    last_disp = 0
    start_t = time.time()
    next_l = start_t + 0.001
    
    try:
        while robot_data.is_running:
            now = time.time()
            # 发送控制指令
            for mid, cfg in MOTOR_CONFIG.items():
                c = robot_data.cmd[mid-1]
                msg_data = pack_mit(c.p, c.v, c.kp, c.kd, c.t)
                try: 
                    buses[cfg["bus_idx"]].send(can.Message(arbitration_id=cfg["can_id"], data=msg_data, is_extended_id=False), timeout=0.0005)
                except: pass
            
            # UI 显示逻辑
            if now - last_disp >= 0.1:
                lines = []
                for grp in [range(1, 7), range(7, 13)]:
                    segs = []
                    for i in grp:
                        m_st = robot_data.state[i-1]
                        d_id = MOTOR_CONFIG[i]["disp_id"]
                        if now - motor_ts[i] < 0.5:
                            segs.append(f"{d_id}:{m_st.p:>5.1f}/{m_st.t:>4.1f}")
                        else:
                            segs.append(f"{d_id}:  ---/ ---")
                    lines.append(" | ".join(segs))
                sys.stdout.write(f"\r\033[K{lines[0]}\n\033[K{lines[1]}\033[F")
                sys.stdout.flush()
                last_disp = now

                # 检查 logic 端是否还在运行（可选）
                # if now - last_logic_heartbeat > 1.0: robot_data.cmd[i].kp = 0

            wait_time = next_l - time.time()
            if wait_time > 0: time.sleep(wait_time)
            next_l += 0.001
            

# 在 dm_hw.py 的末尾修改
    except KeyboardInterrupt:
        print("\n底层驱动捕获到退出信号")
    finally:
        robot_data.is_running = False
        print("正在停止电机...")
        # 禁用所有电机
        for cfg in MOTOR_CONFIG.values(): 
            send_special(buses[cfg["bus_idx"]], cfg["can_id"], 0xFD)
            
        # 关键修改：在删除内存前等待一小会儿，确保逻辑端已经完成清零并安全断开
        time.sleep(0.2) 
        
        shm_obj.close()
        # 只有当你真正想彻底关闭硬件驱动时才执行 unlink
        # 如果你希望 dm_hw 始终运行，不要随便结束这个程序
        shm_obj.unlink() 
        print("✔ 硬件底层资源已完全释放")
