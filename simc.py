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

try:
    from pynput import keyboard
except ImportError:
    print("\n[ERROR] 缺少键盘监听库 pynput！")
    print("👉 请在终端执行: sudo pip install pynput")
    sys.exit(1)

# ==========================================
# 1. 共享内存与硬件连接安全逻辑
# ==========================================
class MotorData(ctypes.Structure):
    _fields_ = [("p", ctypes.c_float), ("v", ctypes.c_float), ("t", ctypes.c_float), 
                ("kp", ctypes.c_float), ("kd", ctypes.c_float)]

class SharedRobotData(ctypes.Structure):
    _fields_ = [("cmd", MotorData * 12), ("state", MotorData * 12), ("is_running", ctypes.c_bool)]

SHM_NAME = "spider_robot_shm" 

print("正在初始化共享内存...")
hw_connected = False
try:
    shm_obj = shared_memory.SharedMemory(name=SHM_NAME)
    hw_connected = True
    print("✔ 已成功连接到硬件底层 (moto.py)")
except FileNotFoundError:
    print("⚠ 未检测到硬件底层，创建仿真专用内存块...")
    shm_obj = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=ctypes.sizeof(SharedRobotData))
except PermissionError:
    print("⚠ 权限拒绝，使用备用内存块...")
    SHM_NAME = "spider_robot_shm_sim_fallback"
    try:
        shm_obj = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=ctypes.sizeof(SharedRobotData))
    except FileExistsError:
        shm_obj = shared_memory.SharedMemory(name=SHM_NAME)

robot_data = SharedRobotData.from_buffer(shm_obj.buf)

if not hw_connected:
    for i in range(12):
        robot_data.cmd[i].p = 0.0; robot_data.cmd[i].v = 0.0; robot_data.cmd[i].kp = 0.0; robot_data.cmd[i].kd = 0.0
        robot_data.state[i].p = 0.0; robot_data.state[i].v = 0.0
    robot_data.is_running = True

JOINT_MAPPING = {
    "rotational_motor-1": 0, "armmotor-1": 1, "kneemotor-1": 2,
    "rotational_motor-2": 3, "armmotor-2": 4, "kneemotor-2": 5,
    "rotational_motor-3": 6, "armmotor-3": 7, "kneemotor-3": 8,
    "rotational_motor-4": 9, "armmotor-4": 10, "kneemotor-4": 11
}

# 扩大保护圈，防止崩溃导致的内存僵尸
try:
    # ==========================================
    # 2. 键盘遥控状态机
    # ==========================================
    keys_pressed = set()

    def on_press(key):
        keys_pressed.add(key)

    def on_release(key):
        keys_pressed.discard(key)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    current_base_y = 0.0
    current_base_z = 2.0
    current_base_roll = 0.0  

    MAX_SPEED = 0.06       # 最大平移速度 6 cm/s
    MAX_ROT_SPEED = 0.25   # 最大旋转速度 0.25 rad/s (约 14度/秒)

    # ==========================================
    # 3. 初始化 Placo & 绘制工具
    # ==========================================
    robot_ik = placo.RobotWrapper("cr.urdf")
    viz = robot_viz(robot_ik)  
    viewer = get_viewer()

    wall_height = 5.0
    wall_mat = g.MeshPhongMaterial(color=0x88ccff, opacity=0.4, transparent=True)
    wall_geom = g.Box([0.005, 10.0, wall_height])
    viewer["virtual_wall"].set_object(wall_geom, wall_mat)
    viewer["virtual_wall"].set_transform(tf.translation_matrix([0.0155, 0, wall_height/2.0]))
    viewer["grid"].set_object(g.Points(g.PointsGeometry(np.zeros((3,1))), g.PointsMaterial()))

    solver = placo.KinematicsSolver(robot_ik)
    solver.enable_joint_limits(True)
    solver.add_regularization_task(1e-4) 

    MAX_ARROW_LENGTH = 1.0 

    def draw_3d_vector(name, start, force_vector, color, scale=0.002, max_len=MAX_ARROW_LENGTH, show=True):
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

    # ==========================================
    # 4. 步态参数配置
    # ==========================================
    lift_height = 0.03    
    epm_thickness = 0.018   
    weight = 12.0 * 9.81 

    t_demag = 0.2; t_swing = 0.8; t_press = 0.2; t_mag = 0.2; t_stance = 4.6     
    cycle_time = t_demag + t_swing + t_press + t_mag + t_stance 
    phase_offsets = {"foot_4": 0.0, "foot_2": 4.5, "foot_1": 3.0, "foot_3": 1.5}

    stance = 0.22 
    wall_dist_x = 0.14  

    # ★ 彻底废除上个版本的局部矩阵计算，直接使用经过验证的世界坐标系偏差距
    foot_world_offsets = {
        "foot_1": np.array([0.0, -stance,  stance]),
        "foot_4": np.array([0.0,  stance,  stance]),
        "foot_2": np.array([0.0, -stance, -stance]),
        "foot_3": np.array([0.0,  stance, -stance])
    }

    print("正在计算初始姿态...")

    T_world_base = np.eye(4)
    T_world_base[:3, :3] = np.array([[0,0,1],[0,-1,0],[1,0,0]])
    
    # ★ 核心防线 1：恢复 HARD 约束，绝对禁止 P 环境中身体翻转 180度
    base_task = solver.add_frame_task("base_link", T_world_base)
    base_task.configure("base", "hard", 1.0, 1.0)

    pos_tasks = {}
    for name in foot_world_offsets.keys():
        task = solver.add_position_task(name, np.zeros(3))
        task.configure(f"{name}_pos", "soft", 1.0) 
        pos_tasks[name] = task

    posture_task = solver.add_joints_task()
    for i in range(1, 5):
        posture_task.set_joint(f"rotational_motor-{i}", 0.0)
    posture_task.configure("rotational_posture", "soft", 1e-2)

    T_world_base[:3, 3] = [wall_dist_x, current_base_y, current_base_z]
    base_task.T_world_frame = T_world_base

    for name in pos_tasks.keys():
        init_world_pos = np.array([wall_dist_x, current_base_y, current_base_z]) + foot_world_offsets[name]
        init_world_pos[0] = epm_thickness 
        pos_tasks[name].target_world = init_world_pos

    for j in robot_ik.joint_names():
        robot_ik.set_joint(j, 0.0)

    for _ in range(300):
        solver.solve(True)
        robot_ik.update_kinematics()

    golden_q = robot_ik.state.q.copy()
    viz.display(golden_q)

    # ★ 核心防线 2：黄金姿态锚定，彻底斩断奇异点发生 IK Flip (膝盖反折抽搐)
    full_posture_task = solver.add_joints_task()
    for j_name in JOINT_MAPPING.keys():
        full_posture_task.set_joint(j_name, golden_q[robot_ik.get_joint_v_offset(j_name)])
    full_posture_task.configure("full_posture", "soft", 5e-3)

    # ==============================================================================
    # 5. MuJoCo 物理场景构建 
    # ==============================================================================
    with open("cr.urdf", "r") as f: urdf_str = f.read()

    def build_collision(match):
        visual_block = match.group(0)
        geom = re.search(r'<geometry>.*?</geometry>', visual_block, re.DOTALL)
        orig = re.search(r'<origin[^>]*>', visual_block)
        col_inner = (orig.group(0) if orig else "") + (geom.group(0) if geom else "")
        return f"{visual_block}\n    <collision>{col_inner}</collision>" if col_inner else visual_block

    urdf_str = re.sub(r'<visual>.*?</visual>', build_collision, urdf_str, flags=re.DOTALL)
    urdf_str = re.sub(r'<dynamics[^>]*>', '', urdf_str) 
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

    fd, tmp_path = tempfile.mkstemp(suffix=".urdf")
    with os.fdopen(fd, 'w') as f: f.write(urdf_str)
    tmp_model = mujoco.MjModel.from_xml_path(tmp_path)
    os.remove(tmp_path)

    fd, tmp_path = tempfile.mkstemp(suffix=".xml")
    os.close(fd)
    mujoco.mj_saveLastXML(tmp_path, tmp_model)
    with open(tmp_path, "r") as f: mjcf_xml = f.read()
    os.remove(tmp_path) 

    mjcf_xml = re.sub(r'(<body name="base_link"[^>]*>)', r'\1\n      <freejoint/>', mjcf_xml, count=1)

    env_xml = f"""
        <light pos="0 0 3" dir="0 0 -1" directional="true"/>
        <geom name="floor" type="plane" size="10 10 0.1" rgba="0.8 0.9 0.8 1" friction="10.0 1.0 0.0001"/>
        <geom name="wall" type="box" size="0.02 10.0 5.0" pos="-0.002 0 0" rgba="0.5 0.8 1 0.5" friction="10.0 1.0 0.0001"/>
        <body name="mj_gravity_arrow" mocap="true" pos="0 0 0"><geom name="geom_mj_gravity_arrow_shaft" type="cylinder" pos="0 0 0.05" size="0.008 0.05" rgba="1 0.2 0.2 0.8" contype="0" conaffinity="0"/><geom name="geom_mj_gravity_arrow_head" type="sphere" pos="0 0 0.1" size="0.025" rgba="1 0.2 0.2 0.8" contype="0" conaffinity="0"/></body>
        <body name="mj_foot_arrow_1" mocap="true" pos="0 0 0"><geom name="geom_mj_foot_arrow_1_shaft" type="cylinder" pos="0 0 0.05" size="0.006 0.05" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/><geom name="geom_mj_foot_arrow_1_head" type="sphere" pos="0 0 0.1" size="0.02" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/></body>
        <body name="mj_foot_arrow_2" mocap="true" pos="0 0 0"><geom name="geom_mj_foot_arrow_2_shaft" type="cylinder" pos="0 0 0.05" size="0.006 0.05" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/><geom name="geom_mj_foot_arrow_2_head" type="sphere" pos="0 0 0.1" size="0.02" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/></body>
        <body name="mj_foot_arrow_3" mocap="true" pos="0 0 0"><geom name="geom_mj_foot_arrow_3_shaft" type="cylinder" pos="0 0 0.05" size="0.006 0.05" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/><geom name="geom_mj_foot_arrow_3_head" type="sphere" pos="0 0 0.1" size="0.02" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/></body>
        <body name="mj_foot_arrow_4" mocap="true" pos="0 0 0"><geom name="geom_mj_foot_arrow_4_shaft" type="cylinder" pos="0 0 0.05" size="0.006 0.05" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/><geom name="geom_mj_foot_arrow_4_head" type="sphere" pos="0 0 0.1" size="0.02" rgba="1 0.6 0 0.9" contype="0" conaffinity="0"/></body>
    """
    mjcf_xml = mjcf_xml.replace("<worldbody>", "<worldbody>\n" + env_xml)

    actuator_xml = "  <actuator>\n"
    for j_name in JOINT_MAPPING.keys():
        actuator_xml += f'    <motor joint="{j_name}" name="{j_name}_motor" gear="1" ctrllimited="true" ctrlrange="-300 300"/>\n'
    actuator_xml += "  </actuator>\n</mujoco>"
    mjcf_xml = mjcf_xml.replace("</mujoco>", actuator_xml)

    model = mujoco.MjModel.from_xml_string(mjcf_xml)
    data = mujoco.MjData(model)

    data.qpos[:3] = [0.14, 0.0, 2.0]
    data.qpos[3:7] = [0.0, 0.70710678, 0.0, 0.70710678]

    mujoco_joint_ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_MAPPING.keys()}
    actuator_ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_motor") for name in JOINT_MAPPING.keys()}

    for j_name in robot_ik.joint_names():
        if j_name == "universe": continue
        mj_id = mujoco_joint_ids.get(j_name, -1)
        if mj_id != -1:
            data.qpos[model.jnt_qposadr[mj_id]] = golden_q[robot_ik.get_joint_v_offset(j_name)]

    mujoco.mj_forward(model, data)
    model.opt.timestep = 0.002

    # ==========================================
    # 6. 主循环 (虚实解耦 + 键盘遥控)
    # ==========================================
    t = 0.0
    dt = 0.002 
    ik_update_rate = 5 
    t_warmup = 4.0
    t_homing = 2.0

    target_kp_sim = 350.0  
    target_kd_sim = 15.0

    # ⭐ 实机空载参数 ⭐
    target_kp_hw = 25.0  
    target_kd_hw = 1.0

    step_counter = 0
    first_loop = True

    initial_p = {}
    prev_ik_p = {hw_idx: 0.0 for hw_idx in JOINT_MAPPING.values()}
    next_ik_p = {hw_idx: 0.0 for hw_idx in JOINT_MAPPING.values()}
    target_ik_v = {hw_idx: 0.0 for hw_idx in JOINT_MAPPING.values()}

    cmd_p_filter = {hw_idx: 0.0 for hw_idx in JOINT_MAPPING.values()}
    lpf_initialized = False

    leg_icons = {str(i): "🟪" for i in range(1, 5)}
    knee_taus = [0.0, 0.0, 0.0, 0.0]

    foot_was_swing = {name: False for name in foot_world_offsets}
    
    # 初始化正确的世界落足点
    foot_abs_pos = {}
    for name in foot_world_offsets:
        init_pos = np.array([wall_dist_x, current_base_y, current_base_z]) + foot_world_offsets[name]
        init_pos[0] = epm_thickness 
        foot_abs_pos[name] = init_pos

    foot_start_pos = {name: foot_abs_pos[name].copy() for name in foot_world_offsets}
    foot_target_pos = {name: foot_abs_pos[name].copy() for name in foot_world_offsets}

    golden_mj_qpos = data.qpos.copy()

    print("=== 【全向键盘遥控 + Placo Mujoco联合仿真 + 共享内存电机控制】 ===")
    print("🎮 鼠标点击终端窗口，使用键盘 ↑ ↓ ← → 控制机器人移动！")
    print("🔄 组合键旋转: (↑或↓) + ← 逆时针 | (↑或↓) + → 顺时针")
    print("图例: 🟨卸磁 | ⬛抬腿 | 🟩压腿 | 🟦加磁 | 🟪支撑")
    print("-" * 150)

    with mujoco.viewer.launch_passive(model, data) as mj_viewer:
        mj_viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = False
        mj_viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False

        next_sim_time = time.perf_counter()
        
        while mj_viewer.is_running() and (not hw_connected or robot_data.is_running):
            
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
                status_panel = f"\r[Time: {t:05.2f}s | WAIT] P(R{p_r:+.0f}°) | M(R{m_r:+.0f}°) | ⏳ 预热中... "
            else:
                placo_t = t - t_warmup

                # ==========================================
                # 键盘速度解析器
                # ==========================================
                v_y, v_z, w_roll = 0.0, 0.0, 0.0
                dir_str = "IDLE "
                
                up = keyboard.Key.up in keys_pressed
                down = keyboard.Key.down in keys_pressed
                left = keyboard.Key.left in keys_pressed
                right = keyboard.Key.right in keys_pressed

                if (up or down) and left:
                    w_roll = MAX_ROT_SPEED 
                    dir_str = "CCW ↺"
                elif (up or down) and right:
                    w_roll = -MAX_ROT_SPEED 
                    dir_str = "CW  ↻"
                else:
                    if up and not down: 
                        v_z = MAX_SPEED
                        dir_str = "UP  ↑"
                    elif down and not up: 
                        v_z = -MAX_SPEED
                        dir_str = "DWN ↓"
                    
                    if left and not right: 
                        v_y = -MAX_SPEED
                        dir_str = "LFT ←"
                    elif right and not left: 
                        v_y = MAX_SPEED
                        dir_str = "RGT →"

                if step_counter % ik_update_rate == 0:
                    dt_ik = ik_update_rate * dt
                    
                    current_base_y += v_y * dt_ik
                    current_base_z += v_z * dt_ik
                    current_base_roll += w_roll * dt_ik
                    
                    T_rot = np.eye(4)
                    T_rot[:3, :3] = tf.rotation_matrix(current_base_roll, [1, 0, 0])[:3, :3] @ np.array([[0,0,1],[0,-1,0],[1,0,0]])
                    T_world_base_target = T_rot
                    T_world_base_target[:3, 3] = [wall_dist_x, current_base_y, current_base_z]
                    
                    base_task.T_world_frame = T_world_base_target
                    
                    active_legs = 0
                    for name in pos_tasks.keys():
                        if (placo_t + phase_offsets[name]) % cycle_time >= t_demag + t_swing + t_press:
                            active_legs += 1
                    friction_z_vis = weight / active_legs if active_legs > 0 else 0.0
                    
                    leg_mj_forces = {}
                    
                    for name in pos_tasks.keys():
                        local_t = (placo_t + phase_offsets[name]) % cycle_time
                        is_swing_phase = (t_demag <= local_t < t_demag + t_swing)
                        
                        if is_swing_phase:
                            if not foot_was_swing[name]:
                                foot_start_pos[name] = foot_abs_pos[name].copy()
                                
                                pred_base_y = current_base_y + v_y * (t_stance / 2.0)
                                pred_base_z = current_base_z + v_z * (t_stance / 2.0)
                                pred_base_roll = current_base_roll + w_roll * (t_stance / 2.0)
                                
                                rot_mat_pred = tf.rotation_matrix(pred_base_roll, [1, 0, 0])[:3, :3]
                                offset_rotated = rot_mat_pred @ foot_world_offsets[name]
                                
                                target_world = np.array([wall_dist_x, pred_base_y, pred_base_z]) + offset_rotated
                                target_world[0] = epm_thickness 
                                foot_target_pos[name] = target_world
                                
                            progress = (local_t - t_demag) / t_swing
                            smooth_p = 0.5 - 0.5 * np.cos(progress * np.pi)
                            
                            current_foot_pos = foot_start_pos[name] + smooth_p * (foot_target_pos[name] - foot_start_pos[name])
                            current_foot_pos[0] = epm_thickness + lift_height * (0.5 - 0.5 * np.cos(progress * 2 * np.pi))
                            
                            foot_abs_pos[name] = current_foot_pos
                            
                            color, state_icon = 0x000000, "⬛"
                            f_x_mag, f_z_fric_vis = 0.0, 0.0
                        else:
                            foot_abs_pos[name][0] = epm_thickness
                            
                            if local_t < t_demag:
                                f_x_mag = -150.0 * (1 - local_t / t_demag); f_z_fric_vis = 0.0; color, state_icon = 0xffff00, "🟨"
                            elif local_t < t_demag + t_swing + t_press:
                                f_x_mag = -20.0 * ((local_t - t_demag - t_swing) / t_press); f_z_fric_vis = 0.0; color, state_icon = 0x00ff00, "🟩"
                            elif local_t < t_demag + t_swing + t_press + t_mag:
                                f_x_mag = -20.0 - 280.0 * ((local_t - t_demag - t_swing - t_press) / t_mag); f_z_fric_vis = friction_z_vis; color, state_icon = 0x00ffff, "🟦"
                            else:
                                f_x_mag = -300.0; f_z_fric_vis = friction_z_vis; color, state_icon = 0x8A2BE2, "🟪"

                        foot_was_swing[name] = is_swing_phase
                        pos_tasks[name].target_world = foot_abs_pos[name]
                        
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

                    solver.solve(True); robot_ik.update_kinematics(); viz.display(robot_ik.state.q)
                    
                    for j_name, hw_idx in JOINT_MAPPING.items():
                        raw_p = robot_ik.get_joint(j_name)
                        prev_ik_p[hw_idx] = next_ik_p[hw_idx]
                        if first_loop:
                            prev_ik_p[hw_idx] = raw_p
                            initial_p[hw_idx] = robot_data.state[hw_idx].p
                            
                        next_ik_p[hw_idx] = raw_p
                        
                        target_ik_v[hw_idx] = (next_ik_p[hw_idx] - prev_ik_p[hw_idx]) / (ik_update_rate * dt)
                        target_ik_v[hw_idx] = np.clip(target_ik_v[hw_idx], -10.0, 10.0)

                    if first_loop: first_loop = False

                interp_t = (step_counter % ik_update_rate) / float(ik_update_rate)
                boot_weight = min(1.0, placo_t / t_homing)
                
                for j_name, hw_idx in JOINT_MAPPING.items():
                    smooth_p = prev_ik_p[hw_idx] + (next_ik_p[hw_idx] - prev_ik_p[hw_idx]) * interp_t
                    
                    init_p = initial_p.get(hw_idx, robot_data.state[hw_idx].p)
                    target_p = (1 - boot_weight) * init_p + boot_weight * smooth_p
                    if initial_p.get(hw_idx) is None: initial_p[hw_idx] = init_p
                    
                    if not lpf_initialized:
                        cmd_p_filter[hw_idx] = target_p
                    
                    cmd_p_filter[hw_idx] += 0.15 * (target_p - cmd_p_filter[hw_idx])
                    
                    robot_data.cmd[hw_idx].p = cmd_p_filter[hw_idx]
                    robot_data.cmd[hw_idx].v = 0.0  
                    
                    robot_data.cmd[hw_idx].kp = target_kp_hw * boot_weight
                    robot_data.cmd[hw_idx].kd = target_kd_hw

                if not lpf_initialized:
                    lpf_initialized = True

                knee_taus = []
                for i in range(1, 5):
                    act_id = actuator_ids.get(f"kneemotor-{i}", -1)
                    if act_id != -1 and len(data.ctrl) > act_id:
                        knee_taus.append(data.ctrl[act_id])
                    else:
                        knee_taus.append(0.0)

                leg_str = " ".join([f"L{i}:{leg_icons.get(str(i), '')}" for i in range(1, 5)])
                tau_str = f"TauK[{knee_taus[0]:+3.0f} {knee_taus[1]:+3.0f} {knee_taus[2]:+3.0f} {knee_taus[3]:+3.0f}]"

                status_panel = f"\r[{t:05.2f}s | {dir_str}] P(Y{p_pos[1]:+.2f} Z{p_pos[2]:+.2f} R{p_r:+.0f}°) | M(Y{m_pos[1]:+.2f} Z{m_pos[2]:+.2f} R{m_r:+.0f}°) | {leg_str} | {tau_str}"

            if t < t_warmup:
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
                    
                    sim_kp = target_kp_sim * boot_weight
                    sim_kd = target_kd_sim
                    sim_v = target_ik_v[hw_idx] * boot_weight
                    
                    tau_pd = sim_kp * (robot_data.cmd[hw_idx].p - data.qpos[model.jnt_qposadr[mj_id]]) \
                           + sim_kd * (sim_v - data.qvel[dof_adr])
                           
                    tau_grav = data.qfrc_bias[dof_adr] 
                    
                    if act_id != -1 and len(data.ctrl) > act_id:
                        data.ctrl[act_id] = np.clip(tau_pd + tau_grav, -300.0, 300.0)
                    
                    if not hw_connected:
                        robot_data.state[hw_idx].p = data.qpos[model.jnt_qposadr[mj_id]]

                if 'leg_mj_forces' in locals():
                    for name, force_x in leg_mj_forces.items():
                        foot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"foot_pyramid-{name[-1]}")
                        if foot_id != -1: data.xfrc_applied[foot_id] = [force_x, 0, 0, 0, 0, 0]
                        
                mujoco.mj_step2(model, data)
                
            mj_viewer.sync()
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
