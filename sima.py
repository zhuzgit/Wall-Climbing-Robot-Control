import placo
import numpy as np
import time
import sys
import os
import tempfile
import ctypes
import re
import traceback
import gc
from multiprocessing import shared_memory
import mujoco
import mujoco.viewer
from placo_utils.visualization import robot_viz, get_viewer
import meshcat.geometry as g
import meshcat.transformations as tf

# ==============================================================================
# 模块 1: 共享内存与硬件连接安全逻辑 (IPC - 进程间通信)
# 作用: 仿真脚本(sim.py)是决策大脑，它需要将计算好的位置、速度以 500Hz 的极高频率
#       发送给真实硬件的驱动脚本(moto.py)。通过 Linux 的共享内存(Shared Memory)
#       可以实现微秒级的极速数据传输，避免 Socket/UDP 带来的网络延迟。
# ==============================================================================
class MotorData(ctypes.Structure):
    # 定义单个电机的数据包格式 (与 moto.py 严格对齐)
    _fields_ = [("p", ctypes.c_float), ("v", ctypes.c_float), ("t", ctypes.c_float), 
                ("kp", ctypes.c_float), ("kd", ctypes.c_float)]

class SharedRobotData(ctypes.Structure):
    # 定义整机 12 个电机的数据包格式
    _fields_ = [("cmd", MotorData * 12), ("state", MotorData * 12), ("is_running", ctypes.c_bool)]

SHM_NAME = "spider_robot_shm" 

print("正在初始化共享内存...")
hw_connected = False
try:
    # 尝试连接底层脚本 (moto.py) 创建的内存块
    shm_obj = shared_memory.SharedMemory(name=SHM_NAME)
    hw_connected = True
    print("✔ 已成功连接到硬件底层 (moto.py)")
except FileNotFoundError:
    # 如果没开 moto.py，说明是纯仿真模式，自己建一块内存自己玩
    print("⚠ 未检测到硬件底层，创建仿真专用内存块...")
    shm_obj = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=ctypes.sizeof(SharedRobotData))
except PermissionError:
    # 极度安全的容错机制：如果上一次程序崩溃导致内存变成 root 权限的僵尸文件，自动换个名字建新内存
    print("⚠ 权限拒绝，使用备用内存块...")
    SHM_NAME = "spider_robot_shm_sim_fallback"
    try:
        shm_obj = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=ctypes.sizeof(SharedRobotData))
    except FileExistsError:
        shm_obj = shared_memory.SharedMemory(name=SHM_NAME)

robot_data = SharedRobotData.from_buffer(shm_obj.buf)

# 纯仿真模式下，清空内存脏数据
if not hw_connected:
    for i in range(12):
        robot_data.cmd[i].p = 0.0; robot_data.cmd[i].v = 0.0; robot_data.cmd[i].kp = 0.0; robot_data.cmd[i].kd = 0.0
        robot_data.state[i].p = 0.0; robot_data.state[i].v = 0.0
    robot_data.is_running = True

# 将电机在 URDF 里的名字，映射到实际 CAN 总线的 ID 索引 (0~11)
JOINT_MAPPING = {
    "rotational_motor-1": 0, "armmotor-1": 1, "kneemotor-1": 2,
    "rotational_motor-2": 3, "armmotor-2": 4, "kneemotor-2": 5,
    "rotational_motor-3": 6, "armmotor-3": 7, "kneemotor-3": 8,
    "rotational_motor-4": 9, "armmotor-4": 10, "kneemotor-4": 11
}

# ==============================================================================
# 模块 2: 初始化 Placo (运动学数学大脑) 与 可视化工具
# 作用: Placo 负责处理复杂的逆运动学(IK)，把你想让脚踩在哪个坐标(X,Y,Z)，
#       转化为 12 个电机的旋转角度(q)。这是整个机器人的“小脑”。
# ==============================================================================
robot_ik = placo.RobotWrapper("cr.urdf")
viz = robot_viz(robot_ik)  
viewer = get_viewer()

# 在浏览器 (Meshcat) 中绘制一面半透明的墙壁，供视觉参考
wall_height = 5.0
wall_mat = g.MeshPhongMaterial(color=0x88ccff, opacity=0.4, transparent=True)
wall_geom = g.Box([0.005, 4.0, wall_height])
viewer["virtual_wall"].set_object(wall_geom, wall_mat)
viewer["virtual_wall"].set_transform(tf.translation_matrix([0.0155, 0, wall_height/2.0]))
viewer["grid"].set_object(g.Points(g.PointsGeometry(np.zeros((3,1))), g.PointsMaterial()))

# 初始化 IK 求解器
solver = placo.KinematicsSolver(robot_ik)
solver.enable_joint_limits(True) # 开启物理关节限位限制
# 微小阻尼正则化：当遇到多解或奇异点(Singularity)时，防止矩阵计算除以 0 导致崩溃
solver.add_regularization_task(1e-4) 

MAX_ARROW_LENGTH = 1.0 

# 以下两个函数负责在 UI 中画箭头，表示电磁铁吸力或重力
def draw_3d_vector(name, start, force_vector, color, scale=0.002, max_len=MAX_ARROW_LENGTH, show=True):
    # Meshcat (浏览器) 里的箭头绘制
    force_norm = np.linalg.norm(force_vector)
    if show and force_norm > 0.1:
        visual_length = force_norm * scale
        if visual_length > max_len: visual_length = max_len
        end = start + force_vector * (visual_length / force_norm)
        vertices = np.array([start, end]).astype(np.float32).T
        viewer[f"forces/{name}"].set_object(g.Line(g.PointsGeometry(vertices), g.LineBasicMaterial(color=color, linewidth=12)))
    else:
        viewer[f"forces/{name}"].set_object(g.Line(g.PointsGeometry(np.zeros((3,2))), g.LineBasicMaterial()))

def update_mocap_arrow(model, data, body_name, start_pos, force_vec, max_len=MAX_ARROW_LENGTH, scale=0.002):
    # MuJoCo (物理引擎) 里的箭头绘制（通过更新 mocap 的四元数和大小）
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id == -1: return
    force_norm = np.linalg.norm(force_vec)
    mocap_id = model.body_mocapid[body_id]
    if force_norm < 0.1:
        data.mocap_pos[mocap_id] = [0, 0, -100] 
        return
    visual_length = force_norm * scale
    if visual_length > max_len: visual_length = max_len
    visual_length = max(visual_length, 0.05) 
    half_length = visual_length / 2.0
    shaft_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"geom_{body_name}_shaft")
    head_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"geom_{body_name}_head")
    model.geom_size[shaft_id][1] = half_length
    model.geom_pos[shaft_id][2] = half_length
    model.geom_pos[head_id][2] = visual_length
    data.mocap_pos[mocap_id] = start_pos
    z_axis = np.array([0.0, 0.0, 1.0])
    target_dir = force_vec / force_norm
    v = np.cross(z_axis, target_dir)
    c = np.dot(z_axis, target_dir)
    if c < -0.9999: quat = np.array([0.0, 1.0, 0.0, 0.0])
    else:
        s = np.sqrt((1.0 + c) * 2.0)
        quat = np.array([s / 2.0, v[0] / s, v[1] / s, v[2] / s])
        quat /= np.linalg.norm(quat)
    data.mocap_quat[mocap_id] = quat

# ==============================================================================
# 模块 3: 步态参数配置与 5步折返宏观轨迹生成
# 作用: 定义机器人走路的快慢、抬腿多高、步子多大，以及构建一个“上爬然后下退”的循环。
# ==============================================================================
n_steps = 5             # 连续走几步后折返
step_length = 0.20      # 每步迈多远 (20厘米)
lift_height = 0.03      # 抬腿高度 (3厘米)
epm_thickness = 0.018   # 电磁铁厚度 (留给墙壁的物理余量)
weight = 12.0 * 9.81    # 机器人重力

# 步态周期切片: 卸磁 -> 空中摆动 -> 压向墙面 -> 充磁 -> 支撑身体向后划水
t_demag = 0.2; t_swing = 0.8; t_press = 0.2; t_mag = 0.2; t_stance = 4.6     
cycle_time = t_demag + t_swing + t_press + t_mag + t_stance 

# 定义 4 条腿在时间轴上的起步相位，形成涟漪步态，确保永远有至少 3 条腿在墙上
phase_offsets = {"foot_4": 0.0, "foot_2": 4.5, "foot_1": 3.0, "foot_3": 1.5}

stance = 0.22  # 腿向两侧张开的半跨距
wall_mid_z = 2.0 # 机器人起步时的绝对 Z 轴高度
wall_dist_x = 0.14 # 机器人肚子离墙面的距离 (极关键的悬臂杠杆距离)

# 4 只脚的静态原点目标坐标
foot_nominals = {
    "foot_1": [0.0, -stance, wall_mid_z + stance],
    "foot_4": [0.0,  stance, wall_mid_z + stance],
    "foot_2": [0.0, -stance, wall_mid_z - stance],
    "foot_3": [0.0,  stance, wall_mid_z - stance]
}

v_body = step_length / cycle_time # 计算出机器人的平均爬升速度
macro_period = 2 * n_steps * cycle_time # 完整一次“上+下”大循环的时间
half_period = n_steps * cycle_time # 单程（只上 或 只下）的时间

def get_base_z(t):
    """根据当前时间 t，计算出机器人的肚子(base_link)应该处于多少米的高度"""
    t_mod = t % macro_period
    if t_mod < half_period: # 前半段：上升
        return wall_mid_z + v_body * t_mod  
    else:                   # 后半段：下降
        return wall_mid_z + v_body * half_period - v_body * (t_mod - half_period)

print("正在计算黄金初始姿态...")

# 为机器人的躯干(base_link)创建一个在三维空间中绝对锁死的框架任务
T_world_base = np.eye(4)
T_world_base[:3, :3] = np.array([[0,0,1],[0,-1,0],[1,0,0]])
base_task = solver.add_frame_task("base_link", T_world_base)
base_task.configure("base", "soft", 1.0, 1.0)

# 为 4 只脚创建位置任务 (让 IK 引擎去追求这个位置)
pos_tasks = {}
for name in foot_nominals.keys():
    task = solver.add_position_task(name, np.zeros(3))
    task.configure(f"{name}_pos", "soft", 1.0) 
    pos_tasks[name] = task

# 轻微姿态任务：引导 IK 引擎倾向于把旋转电机留在 0 度附近，防止出现极端麻花解
posture_task = solver.add_joints_task()
for i in range(1, 5):
    posture_task.set_joint(f"rotational_motor-{i}", 0.0)
posture_task.configure("rotational_posture", "soft", 1e-2)

current_base_z = get_base_z(0.0)
T_world_base[:3, 3] = [0.18, 0.0, current_base_z]
base_task.T_world_frame = T_world_base

for name in pos_tasks.keys():
    pos_tasks[name].target_world = np.array([epm_thickness, foot_nominals[name][1], foot_nominals[name][2]])

for j in robot_ik.joint_names():
    robot_ik.set_joint(j, 0.0)

# 迭代求解 300 次，算出起步时完美的关节弯曲状态
for _ in range(300):
    solver.solve(True)
    robot_ik.update_kinematics()

golden_q = robot_ik.state.q.copy()
viz.display(golden_q)

# ==============================================================================
# 模块 4: MuJoCo 物理场景构建 (M 环境：真正的受力炼狱)
# 作用: 实时修改并加载机器人的 URDF 模型，将其丢入拥有重力、碰撞和摩擦力的物理引擎。
# ==============================================================================
with open("cr.urdf", "r") as f: urdf_str = f.read()

def build_collision(match):
    """让物理引擎把视觉网格(visual)识别为可发生物理碰撞的实体(collision)"""
    visual_block = match.group(0)
    geom = re.search(r'<geometry>.*?</geometry>', visual_block, re.DOTALL)
    orig = re.search(r'<origin[^>]*>', visual_block)
    col_inner = (orig.group(0) if orig else "") + (geom.group(0) if geom else "")
    return f"{visual_block}\n    <collision>{col_inner}</collision>" if col_inner else visual_block

urdf_str = re.sub(r'<visual>.*?</visual>', build_collision, urdf_str, flags=re.DOTALL)
urdf_str = re.sub(r'<dynamics[^>]*>', '', urdf_str) 
# 洗白转动惯量，防止奇怪的网格碎片导致物理引擎除零崩溃
urdf_str = re.sub(r'ixx="[^"]+"', 'ixx="0.01"', urdf_str)
urdf_str = re.sub(r'iyy="[^"]+"', 'iyy="0.01"', urdf_str)
urdf_str = re.sub(r'izz="[^"]+"', 'izz="0.01"', urdf_str)
urdf_str = re.sub(r'mass value="0\.0[0-9]*"', 'mass value="0.05"', urdf_str)

mujoco_options = """
  <mujoco>
    <compiler discardvisual="false" fusestatic="false"/>
    <option integrator="implicitfast" cone="elliptic" impratio="100"/>
    <default>
      <geom friction="10.0 1.0 0.0001" margin="0.001" condim="4"/>
    </default>
  </mujoco>
"""
urdf_str = re.sub(r'(<robot[^>]*>)', r'\1\n' + mujoco_options, urdf_str)

# 动态保存并转译 XML
fd, tmp_path = tempfile.mkstemp(suffix=".urdf")
with os.fdopen(fd, 'w') as f: f.write(urdf_str)
tmp_model = mujoco.MjModel.from_xml_path(tmp_path)
os.remove(tmp_path)

fd, tmp_path = tempfile.mkstemp(suffix=".xml")
os.close(fd)
mujoco.mj_saveLastXML(tmp_path, tmp_model)
with open(tmp_path, "r") as f: mjcf_xml = f.read()
os.remove(tmp_path) 

# ★ 关键指令：赋予躯干 freejoint 自由度，让其受到 12kg 重力的坠落拉扯
mjcf_xml = re.sub(r'(<body name="base_link"[^>]*>)', r'\1\n      <freejoint/>', mjcf_xml, count=1)

# 构建物理世界的环境：地面、墙壁以及可视化受力箭头
env_xml = f"""
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="10 10 0.1" rgba="0.8 0.9 0.8 1" friction="10.0 1.0 0.0001"/>
    <geom name="wall" type="box" size="0.02 2.0 2.0" pos="-0.002 0 2.0" rgba="0.5 0.8 1 0.5" friction="10.0 1.0 0.0001"/>
    <body name="mj_gravity_arrow" mocap="true" pos="0 0 0"><geom name="geom_mj_gravity_arrow_shaft" type="cylinder" pos="0 0 0.05" size="0.008 0.05" rgba="1 0.2 0.2 0.8" contype="0" conaffinity="0"/><geom name="geom_mj_gravity_arrow_head" type="sphere" pos="0 0 0.1" size="0.025" rgba="1 0.2 0.2 0.8" contype="0" conaffinity="0"/></body>
    <body name="mj_foot_arrow_1" mocap="true" pos="0 0 0"><geom name="geom_mj_foot_arrow_1_shaft" type="cylinder" pos="0 0 0.05" size="0.006 0.05" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/><geom name="geom_mj_foot_arrow_1_head" type="sphere" pos="0 0 0.1" size="0.02" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/></body>
    <body name="mj_foot_arrow_2" mocap="true" pos="0 0 0"><geom name="geom_mj_foot_arrow_2_shaft" type="cylinder" pos="0 0 0.05" size="0.006 0.05" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/><geom name="geom_mj_foot_arrow_2_head" type="sphere" pos="0 0 0.1" size="0.02" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/></body>
    <body name="mj_foot_arrow_3" mocap="true" pos="0 0 0"><geom name="geom_mj_foot_arrow_3_shaft" type="cylinder" pos="0 0 0.05" size="0.006 0.05" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/><geom name="geom_mj_foot_arrow_3_head" type="sphere" pos="0 0 0.1" size="0.02" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/></body>
    <body name="mj_foot_arrow_4" mocap="true" pos="0 0 0"><geom name="geom_mj_foot_arrow_4_shaft" type="cylinder" pos="0 0 0.05" size="0.006 0.05" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/><geom name="geom_mj_foot_arrow_4_head" type="sphere" pos="0 0 0.1" size="0.02" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/></body>
"""
mjcf_xml = mjcf_xml.replace("<worldbody>", "<worldbody>\n" + env_xml)

# 为模型强行装上物理电机执行器，允许它们爆发出最高 300Nm 的力矩
actuator_xml = "  <actuator>\n"
for j_name in JOINT_MAPPING.keys():
    actuator_xml += f'    <motor joint="{j_name}" name="{j_name}_motor" gear="1" ctrllimited="true" ctrlrange="-300 300"/>\n'
actuator_xml += "  </actuator>\n</mujoco>"
mjcf_xml = mjcf_xml.replace("</mujoco>", actuator_xml)

model = mujoco.MjModel.from_xml_string(mjcf_xml)
data = mujoco.MjData(model)

data.qpos[:3] = [0.18, 0.0, current_base_z]
data.qpos[3:7] = [0.0, 0.70710678, 0.0, 0.70710678]

mujoco_joint_ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_MAPPING.keys()}
actuator_ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_motor") for name in JOINT_MAPPING.keys()}

# 将前面求出来的“黄金初始姿态”注入物理引擎
for j_name in robot_ik.joint_names():
    if j_name == "universe": continue
    mj_id = mujoco_joint_ids.get(j_name, -1)
    if mj_id != -1:
        data.qpos[model.jnt_qposadr[mj_id]] = golden_q[robot_ik.get_joint_v_offset(j_name)]

mujoco.mj_forward(model, data)
model.opt.timestep = 0.002

# ==============================================================================
# 模块 5: 虚实解耦 主控制循环 (核心：防抖与双环境适配)
# 作用: 这里是整个代码跳动的心脏。包含了 100Hz 的大脑路线规划，
#       以及 500Hz 的仿真受力计算与底层真机指令下发。
# ==============================================================================
t = 0.0
dt = 0.002 
ik_update_rate = 5 
t_warmup = 4.0
t_homing = 2.0

# ------------------------------------------------------------------
# ⭐⭐⭐【实机带载修改点 1：刚度参数 (Kp/Kd) 】⭐⭐⭐
# ------------------------------------------------------------------
# 仿真引擎（带载抗重力）：需要高刚度才能吸在墙上不掉下来。
target_kp_sim = 350.0  
target_kd_sim = 15.0

# 真实桌面电机（空载防抖）：
# 注意！当你真正把机器人贴到墙上时，必须把这里的硬件 Kp 修改回 350.0 左右！
# 否则电机将彻底腿软，无法支撑 12kg 的庞大身躯！
target_kp_hw = 25.0  # <--- 实机带载时，修改为 350.0
target_kd_hw = 1.0   # <--- 实机带载时，修改为 15.0
# ------------------------------------------------------------------

step_counter = 0
first_loop = True

initial_p = {}
prev_ik_p = {hw_idx: 0.0 for hw_idx in JOINT_MAPPING.values()}
next_ik_p = {hw_idx: 0.0 for hw_idx in JOINT_MAPPING.values()}
target_ik_v = {hw_idx: 0.0 for hw_idx in JOINT_MAPPING.values()}

# 底层防抖用到的 PT1 一阶低通滤波器状态变量
cmd_p_filter = {hw_idx: 0.0 for hw_idx in JOINT_MAPPING.values()}
lpf_initialized = False

leg_icons = {str(i): "🟪" for i in range(1, 5)}
knee_taus = [0.0, 0.0, 0.0, 0.0]

foot_was_swing = {name: False for name in foot_nominals}
foot_abs_z = {name: foot_nominals[name][2] for name in foot_nominals}
foot_start_z = {name: foot_abs_z[name] for name in foot_nominals}
foot_target_z = {name: foot_abs_z[name] for name in foot_nominals}

golden_mj_qpos = data.qpos.copy()

print("=== 【原版 100% 还原 + 虚实解耦防抖版】 ===")
print("已彻底修复浮点漂移 KeyError！")
print("仿真环境：完全还原您的 sim.py (5步上/下，350刚度对抗重力，不改动任何显示)")
print("真实电机：专为空载环境调校 (低刚度，PT1滤波，取消差分速度)，彻底消灭抽搐！")
print("图例: 🟨卸磁 | ⬛抬腿 | 🟩压腿 | 🟦加磁 | 🟪支撑")
print("-" * 150)

try:
    with mujoco.viewer.launch_passive(model, data) as mj_viewer:
        mj_viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = False
        mj_viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False

        next_sim_time = time.perf_counter()
        
        while mj_viewer.is_running() and (not hw_connected or robot_data.is_running):
            
            # 读取当前 P 环境(数学大脑) 和 M 环境(物理引擎) 躯干的坐标状态供 UI 打印
            p_mat = robot_ik.get_T_world_frame("base_link")
            p_pos = p_mat[:3, 3]
            p_euler = tf.euler_from_matrix(p_mat)
            p_r, p_p, p_y = np.degrees(p_euler)

            m_pos = data.qpos[:3]
            m_quat_mj = data.qpos[3:7] 
            m_quat_tf = [m_quat_mj[1], m_quat_mj[2], m_quat_mj[3], m_quat_mj[0]] 
            m_euler = tf.euler_from_quaternion(m_quat_tf)
            m_r, m_p, m_y = np.degrees(m_euler)
            
            if t < t_warmup:
                viz.display(golden_q)
                status_panel = f"\r[Time: {t:05.2f}s | WAIT] P(R{p_r:+.0f}° P{p_p:+.0f}° Y{p_y:+.0f}°) | M(R{m_r:+.0f}° P{m_p:+.0f}° Y{m_y:+.0f}°) | ⏳ 预热中... "
            else:
                placo_t = t - t_warmup
                
                # 判定当前大循环是处于“上升 U”还是“下降 D”阶段
                t_mod = placo_t % macro_period
                is_up = t_mod < half_period
                if is_up:
                    current_step = int(t_mod / cycle_time) + 1
                    step_dir_str = f"U{current_step}"
                else:
                    current_step = int((t_mod - half_period) / cycle_time) + 1
                    step_dir_str = f"D{current_step}"

                # -------------------------------------------------------------
                # [ 100Hz 循环 ] 数学大脑 (Placo IK) 路线规划层
                # -------------------------------------------------------------
                if step_counter % ik_update_rate == 0:
                    
                    # 设定当前时刻，机器人躯干应该在 Z 轴的多少高度
                    current_base_z = get_base_z(placo_t)
                    T_world_base_target = T_world_base.copy()
                    T_world_base_target[2, 3] = current_base_z
                    base_task.T_world_frame = T_world_base_target
                    
                    active_legs = 0
                    for name in pos_tasks.keys():
                        if (placo_t + phase_offsets[name]) % cycle_time >= t_demag + t_swing + t_press:
                            active_legs += 1
                    friction_z_vis = weight / active_legs if active_legs > 0 else 0.0
                    
                    leg_mj_forces = {}
                    
                    # 遍历 4 条腿，根据步态切片（落地支撑 或 抬腿跨越）给定空间目标坐标
                    for name in pos_tasks.keys():
                        local_t = (placo_t + phase_offsets[name]) % cycle_time
                        is_swing_phase = (t_demag <= local_t < t_demag + t_swing)
                        y = foot_nominals[name][1]
                        
                        if is_swing_phase:
                            if not foot_was_swing[name]:
                                foot_start_z[name] = foot_abs_z[name]
                                landing_t = placo_t + (t_demag + t_swing - local_t)
                                landing_dir = 1.0 if (landing_t % macro_period) < half_period else -1.0
                                stroke_offset = v_body * (t_stance / 2.0)
                                foot_target_z[name] = get_base_z(landing_t) + (foot_nominals[name][2] - wall_mid_z) + landing_dir * stroke_offset
                                
                            progress = (local_t - t_demag) / t_swing
                            # 余弦波平滑处理 Z 轴
                            smooth_p = 0.5 - 0.5 * np.cos(progress * np.pi)
                            # 1-Cosine 钟形曲线处理抬腿 X 轴 (两端速度绝对为0，极其关键的防砸墙爆震手段)
                            x = epm_thickness + lift_height * np.sin(progress * np.pi)
                            foot_abs_z[name] = foot_start_z[name] + smooth_p * (foot_target_z[name] - foot_start_z[name])
                            color, state_icon = 0x000000, "⬛"
                            f_x_mag, f_z_fric_vis = 0.0, 0.0
                        else:
                            x = epm_thickness
                            # 按不同阶段施加电磁吸力大小
                            if local_t < t_demag:
                                f_x_mag = -150.0 * (1 - local_t / t_demag); f_z_fric_vis = 0.0; color, state_icon = 0xffff00, "🟨"
                            elif local_t < t_demag + t_swing + t_press:
                                f_x_mag = -20.0 * ((local_t - t_demag - t_swing) / t_press); f_z_fric_vis = 0.0; color, state_icon = 0x00ff00, "🟩"
                            elif local_t < t_demag + t_swing + t_press + t_mag:
                                f_x_mag = -20.0 - 280.0 * ((local_t - t_demag - t_swing - t_press) / t_mag); f_z_fric_vis = friction_z_vis; color, state_icon = 0x00ffff, "🟦"
                            else:
                                f_x_mag = -300.0; f_z_fric_vis = friction_z_vis; color, state_icon = 0x8A2BE2, "🟪"

                        foot_was_swing[name] = is_swing_phase
                        pos_tasks[name].target_world = np.array([x, y, foot_abs_z[name]])
                        f_vec = np.array([f_x_mag, 0, f_z_fric_vis])
                        
                        f_pos_placo = robot_ik.get_T_world_frame(name)[:3, 3]
                        draw_3d_vector(name, f_pos_placo, f_vec, color)
                        
                        foot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"foot_pyramid-{name[-1]}")
                        if foot_body_id != -1:
                            f_pos_mj = data.xpos[foot_body_id].copy()
                            update_mocap_arrow(model, data, f"mj_foot_arrow_{name[-1]}", f_pos_mj, f_vec)
                        
                        leg_mj_forces[name] = f_x_mag
                        leg_icons[name[-1]] = state_icon 

                    base_pos_placo = robot_ik.get_T_world_frame("base_link")[:3, 3]
                    draw_3d_vector("gravity", base_pos_placo, np.array([0, 0, -weight]), 0xff0000, scale=0.003)
                    base_pos_mj = data.qpos[:3].copy()
                    update_mocap_arrow(model, data, "mj_gravity_arrow", base_pos_mj, np.array([0, 0, -weight]))

                    # 求解 IK 逆向运动学，得到当前的完美的 target 关节角度
                    solver.solve(True); robot_ik.update_kinematics(); viz.display(robot_ik.state.q)
                    
                    # 计算目标速度 target_ik_v，用于后续仿真补偿
                    for j_name, hw_idx in JOINT_MAPPING.items():
                        raw_p = robot_ik.get_joint(j_name)
                        prev_ik_p[hw_idx] = next_ik_p[hw_idx]
                        if first_loop:
                            prev_ik_p[hw_idx] = raw_p
                        next_ik_p[hw_idx] = raw_p
                        
                        target_ik_v[hw_idx] = (next_ik_p[hw_idx] - prev_ik_p[hw_idx]) / (ik_update_rate * dt)
                        target_ik_v[hw_idx] = np.clip(target_ik_v[hw_idx], -10.0, 10.0)

                    if first_loop: first_loop = False

                # -------------------------------------------------------------
                # [ 500Hz 循环 ] 硬件通信层：微小插值与平滑下发 (解耦硬件震颤)
                # -------------------------------------------------------------
                interp_t = (step_counter % ik_update_rate) / float(ik_update_rate)
                boot_weight = min(1.0, placo_t / t_homing)
                
                sim_v_dict = {}

                for j_name, hw_idx in JOINT_MAPPING.items():
                    # 线性插值：把 100Hz 的阶梯跳变，抹平成 500Hz 的平滑直线
                    smooth_p = prev_ik_p[hw_idx] + (next_ik_p[hw_idx] - prev_ik_p[hw_idx]) * interp_t
                    
                    if initial_p.get(hw_idx) is None:
                        # 极端防御型读取：避免出现起步第一帧找不到初值而引发的 KeyError
                        initial_p[hw_idx] = robot_data.state[hw_idx].p
                        
                    target_p = (1 - boot_weight) * initial_p[hw_idx] + boot_weight * smooth_p
                    
                    # 仿真器用的带载前馈速度
                    sim_v_dict[hw_idx] = target_ik_v[hw_idx] * boot_weight
                    
                    # --- 底层电子海绵 PT1 防抖滤波 ---
                    if not lpf_initialized:
                        cmd_p_filter[hw_idx] = target_p
                    
                    # Alpha 设为 0.15，对插值后的微小折角进行终极熨平
                    cmd_p_filter[hw_idx] += 0.15 * (target_p - cmd_p_filter[hw_idx])
                    
                    # 【核心解耦点】：下发给桌面上真实电机的纯净指令
                    robot_data.cmd[hw_idx].p = cmd_p_filter[hw_idx]
                    
                    # ------------------------------------------------------------------
                    # ⭐⭐⭐【实机带载修改点 2：速度前馈 (v) 】⭐⭐⭐
                    # ------------------------------------------------------------------
                    # 在空载桌面测试时，我们必须屏蔽速度 v = 0.0，否则巨大的 Kd 将引发疯狂抖动。
                    # 当机器人真正挂在墙上时，由于重力的压迫导致电机滞后，
                    # 此时你应该把这里恢复为：`robot_data.cmd[hw_idx].v = sim_v_dict[hw_idx]` 
                    # 用 IK 的导数速度推着电机跑，克服负载。
                    robot_data.cmd[hw_idx].v = 0.0  
                    # ------------------------------------------------------------------

                    # 发送极低的防抖刚度给桌面电机
                    robot_data.cmd[hw_idx].kp = target_kp_hw * boot_weight
                    robot_data.cmd[hw_idx].kd = target_kd_hw

                if not lpf_initialized:
                    lpf_initialized = True

                # 抓取当前物理仿真引擎里四个膝盖的扭矩，打印在 UI 上
                knee_taus = []
                for i in range(1, 5):
                    act_id = actuator_ids.get(f"kneemotor-{i}", -1)
                    if act_id != -1 and len(data.ctrl) > act_id:
                        knee_taus.append(data.ctrl[act_id])
                    else:
                        knee_taus.append(0.0)

                leg_str = " ".join([f"L{i}:{leg_icons.get(str(i), '')}" for i in range(1, 5)])
                tau_str = f"TauK[{knee_taus[0]:+4.0f} {knee_taus[1]:+4.0f} {knee_taus[2]:+4.0f} {knee_taus[3]:+4.0f}]"

                status_panel = f"\r[{t:05.2f}s | {step_dir_str}] P(X{p_pos[0]:.2f} Y{p_pos[1]:.2f} Z{p_pos[2]:.2f} | R{p_r:+.0f}° P{p_p:+.0f}° Y{p_y:+.0f}°) " \
                               f"M(X{m_pos[0]:.2f} Y{m_pos[1]:.2f} Z{m_pos[2]:.2f} | R{m_r:+.0f}° P{m_p:+.0f}° Y{m_y:+.0f}°) " \
                               f"| {leg_str} | {tau_str}"

            # -------------------------------------------------------------
            # [ 500Hz 循环 ] MuJoCo 物理仿真层 (M环境：虚拟机器人的炼狱)
            # -------------------------------------------------------------
            if t < t_warmup:
                # 预热期 4 秒，强行冻结仿真机器人在原点不掉下来
                data.qpos[:] = golden_mj_qpos; data.qvel[:] = 0.0; mujoco.mj_forward(model, data)
                if not hw_connected:
                    for j_name, hw_idx in JOINT_MAPPING.items():
                        robot_data.state[hw_idx].p = data.qpos[model.jnt_qposadr[mujoco_joint_ids[j_name]]]
            else:
                mujoco.mj_step1(model, data)
                
                for j_name, hw_idx in JOINT_MAPPING.items():
                    mj_id = mujoco_joint_ids[j_name]
                    act_id = actuator_ids.get(j_name, -1)
                    dof_adr = model.jnt_dofadr[mj_id]
                    
                    # 【核心解耦点】：仿真引擎里的受力计算！
                    # 虚拟机器人使用 Kp=350, Kd=15 的高抗重力刚度，且使用有效的前馈速度。
                    # 它绝不会因为桌面上真实电机的 Kp=25 而变得腿软。
                    sim_kp = target_kp_sim * boot_weight
                    sim_kd = target_kd_sim
                    
                    # 计算内部弹簧阻尼 (PD) 力矩
                    tau_pd = sim_kp * (robot_data.cmd[hw_idx].p - data.qpos[model.jnt_qposadr[mj_id]]) \
                           + sim_kd * (sim_v_dict.get(hw_idx, 0.0) - data.qvel[dof_adr])
                           
                    # 【实机带载修改点 3：前馈重力补偿】
                    # 这里抓取了虚拟机器人的重力补偿项 (tau_grav)。
                    # 在实际开发中，当你要上墙时，你需要把这个 tau_grav 项通过 `robot_data.cmd[hw_idx].t` 
                    # 作为一个前馈电流力矩，主动发给真实的电机驱动器。否则真实机器人全靠 PD 死扛重力会有静差。
                    tau_grav = data.qfrc_bias[dof_adr] 
                    
                    if act_id != -1 and len(data.ctrl) > act_id:
                        data.ctrl[act_id] = np.clip(tau_pd + tau_grav, -300.0, 300.0)
                    
                    # 如果纯仿真(没接真机)，就把仿真当前位置回传给状态层
                    if not hw_connected:
                        robot_data.state[hw_idx].p = data.qpos[model.jnt_qposadr[mj_id]]

                # 向虚拟脚底施加磁吸力与静摩擦力
                if 'leg_mj_forces' in locals():
                    for name, force_x in leg_mj_forces.items():
                        foot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"foot_pyramid-{name[-1]}")
                        if foot_id != -1: data.xfrc_applied[foot_id] = [force_x, 0, 0, 0, 0, 0]
                        
                mujoco.mj_step2(model, data)
                
            mj_viewer.sync()
            
            # 刷新 UI 进度条并限制循环速度以保证真实时间流速
            if step_counter % ik_update_rate == 0 or True:
                sys.stdout.write(status_panel); sys.stdout.flush()
            t += dt; step_counter += 1
            next_sim_time += dt
            while time.perf_counter() < next_sim_time: pass

except Exception as e:
    print(f"\n[ERROR] 发生异常: {e}"); traceback.print_exc()
except KeyboardInterrupt:
    print("\n\n[INFO] 退出信号，安全释放内存...")

finally:
    if 'robot_data' in locals(): del robot_data
    gc.collect() 
    if 'shm_obj' in locals():
        try: shm_obj.buf.release() 
        except Exception: pass
        try:
            if not hw_connected: shm_obj.unlink()
            else: shm_obj.close()
        except Exception as e: pass