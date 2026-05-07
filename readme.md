# 1. readme

主要功能：

1. 电机初始化 dm.py
2. 电机参数设置 bs.sh
3. 机器人仿真及控制 moto.py sima.py simc.py see.py

# 2. 机器人自动上下5步，循环联合仿真

[SIMA高清视频](https://github.com/zhuzgit/Wall-Climbing-Robot-Control/blob/main/sima.mp4)

<img width="480" height="270" alt="sima" src="https://github.com/user-attachments/assets/2c655f28-9018-401c-920a-c37b3f6a9be7" />

# 3. 机器人键盘控制，循环联合仿真

[SIMC高清视频](https://github.com/zhuzgit/Wall-Climbing-Robot-Control/blob/main/simc.mp4)

<img width="480" height="270" alt="simc" src="https://github.com/user-attachments/assets/b1d9c035-b1a2-4a5d-a41c-aa02d4aba5d0" />

 ## 3.1 电机初始化、控制和开启共享内存  moto.py

    
 $ sudo python moto.py 



>正在配置 CAN 接口...
>
>✔ CAN 接口配置完成
>
>正在使能电机并校准零点...
>
>✔ 硬件底层已就绪！
>
>显示格式：ID: 位置/扭矩
>
>01: ---/ --- | 02: -0.0/-0.0 | 03:  ---/ --- | 04: -0.0/-0.0 | 05:  ---/ --- | 06:  ---/ ---
>
>11: ---/ --- | 02: -0.0/-0.0 | 13: -0.0/-0.0 | 14:  ---/ --- | 15:  ---/ --- | 16:  ---/ ---
>
>^C
>
>底层驱动捕获到退出信号
>
>正在停止电机...



## 3.2 机器人自动上下五步，循环往复  sima.py

$ sudo python sima.py

 <img width="1826" height="338" alt="image" src="https://github.com/user-attachments/assets/03c62da0-8c00-437d-a2d3-aae7c7e0612d" />

>正在初始化共享内存...
>
>✔ 已成功连接到硬件底层 (moto.py)
>
>You can open the visualizer by visiting the following URL:
>
>http://127.0.0.1:7001/static/
>
>Viewer URL: http://127.0.0.1:7001/static/
>
>正在计算黄金初始姿态...
>
>=== 【原版 100% 还原 + 虚实解耦防抖版】 ===
>
>已彻底修复浮点漂移 KeyError！
>
>仿真环境：完全还原您的 sim.py (5步上/下，350刚度对抗重力，不改动任何显示)
>
>真实电机：专为空载环境调校 (低刚度，PT1滤波，取消差分速度)，彻底消灭抽搐！
>
>图例: 🟨卸磁 | ⬛抬腿 | 🟩压腿 | 🟦加磁 | 🟪支撑
>
>>------------------------------------------------------------------------------------------------------------------------------------------------------
>[09.61s | U1] P(X0.18 Y0.02 Z2.19 | R+55° P-90° Y+125°) M(X0.20 Y0.01 Z2.05 | R-139° P+88° Y-141°) | L1:🟪 L2:🟪 L3:🟩 L4:🟪 | TauK[ +23  +20  -15 -104]


[运行时间 | U向上n步 D向下n步] P（Placo环境 坐标和IMU） M（Mujoco环境 坐标和IMU）Ln 第几条腿：颜色代表状态 TauK是四个膝盖电机的实时物理输出扭矩（力矩）

## 3.3 机器人受控运动，simc.py

$ sudo python simc.py

<img width="1382" height="344" alt="image" src="https://github.com/user-attachments/assets/38568043-3a37-486d-8112-e618c1001cd2" />

>正在初始化共享内存...
>
>✔ 已成功连接到硬件底层 (moto.py)
>
>You can open the visualizer by visiting the following URL:
>
>http://127.0.0.1:7001/static/
>
>Viewer URL: http://127.0.0.1:7001/static/
>
>正在计算黄金初始姿态...
>
>=== 【全向键盘遥控 + 完美虚实解耦版】 ===
>
>已彻底修复坐标系扭曲Bug，加入数学光闸与姿态金钟罩！
>
>🎮 鼠标点击终端窗口，使用键盘 ↑ ↓ ← → 控制机器人移动！
>
>🔄 组合键旋转: (↑或↓) + ← 逆时针 | (↑或↓) + → 顺时针
>
>图例: 🟨卸磁 | ⬛抬腿 | 🟩压腿 | 🟦加磁 | 🟪支撑
>
>------------------------------------------------------------------------------------------------------------------------------------------------------
>
>[10.79s | UP  ↑] P(Y+0.00 Z+2.10 R-180°) | M(Y-0.01 Z+2.02 R+50°) | L1:🟪 L2:🟪 L3:🟪 L4:⬛ | TauK[-44 +27 -17 -91]

[时间 命令：上下左右及正逆时针旋转]  P（Placo环境 坐标和IMU） M（Mujoco环境 坐标和IMU）Ln 第几条腿：颜色代表状态 TauK是四个膝盖电机的实时物理输出扭矩（力矩）

# 4. 查看共享内存数据 see.sh see.py

> $ sudo ./see.sh
>
> [SHM_PLOT] 等待共享内存 'spider_robot_shm' 就绪 (请先运行 moto.py)...
>
> ✔ 成功连接共享内存。开始绘制信号: ['cmd_p_00', 'cmd_p_01', 'cmd_p_02', 'cmd_p_03', 'cmd_p_04', 'cmd_p_05', 'cmd_p_06', 'cmd_p_07', 'cmd_p_08', 'cmd_p_09', 'cmd_p_10', 'cmd_p_11']
>
> ^C
>
> [INFO] 检测到手动退出。
>
> 正在清理绘图资源...

see.sh 命令内容是

>sudo python3 see.py --hz 50 --duration 10 --signals cmd_p_00 cmd_p_01 cmd_p_02 cmd_p_03 cmd_p_04 cmd_p_05 cmd_p_06 cmd_p_07 cmd_p_08 cmd_p_09 cmd_p_10 cmd_p_11
>
>--hz 50：让画面以每秒 50 帧的速度丝滑刷新。
>
>--duration 10：画面 X 轴始终显示过去 10 秒钟的历史轨迹。
>
>--signals cmd_p_00 ... cmd_p_11：同时把 12 个电机的“目标位置(P)”全部画在同一张图上！


>see.py附加以下参数来定制您的监控画面：
>
>--hz：绘图刷新率（赫兹）。决定了画面每秒更新多少次。默认为 50。
>
>--duration：X 轴时间窗（秒）。决定了画面上能看到过去多长时间的数据。默认是 10 秒。
>
>--signals：要监控的信号列表。您可以同时监控多个信号，用空格隔开

下发给电机的指令 (Command)：

>cmd_p_xx：目标位置 (Position) —— 大脑期望电机转到多少度。
>
>cmd_v_xx：目标速度 (Velocity) —— 大脑期望电机转多快。
>
>cmd_t_xx：前馈力矩 (Torque) —— 大脑额外给的补偿力气。
>
>cmd_kp_xx：位置刚度 (Kp) —— 大脑设定的“弹簧”硬度。
>
>cmd_kd_xx：速度阻尼 (Kd) —— 大脑设定的“减震器”粘度。

真实电机传回的物理状态 (State)：

>state_p_xx：实际反馈位置 —— 电机当前真实所处的角度。
>
>state_v_xx：实际反馈速度 —— 电机当前真实的转速。
>
>state_t_xx：实际反馈力矩 —— 电机当前真实输出的扭矩（受力大小）。
>
>(注：xx 取值为 00 到 11。例如 00, 01, 02 对应第一条腿的三个电机，依次类推)

# 5. 更改电机ID和波特率 bs.sh
## 5.1 修改参数

>参数设定区
> =========================================================
>CAN_IF="can1"
>
> --- 初始连接参数 (确保当前能通) ---
>
>INIT_BITRATE=1000000
>
>INIT_DBITRATE=1000000 #1M~5M
>
># --- 目标修改参数 ---
>
>CURRENT_ID=3     # 电机当前的 ID (十进制)
>
>TARGET_ID=2      # 准备修改成的目标 ID (十进制)
>
>
># 目标波特率代码: 04 对应 1M, 09 对应 5M
>
>TARGET_BAUD_HEX="04" 

## 5.2 运行
>-------------------------------------------------------
>
>电机当前 ID: 2 (前缀: 0002)
>
>目标修改为 ID: 2
>
>自动计算 Master ID: 0x12
>
>-------------------------------------------------------
>
>发送禁用指令 (确保进入可写模式)...
>
>cansend can1 7FF#FFFFFFFFFFFFFFFD
>
>[1/3] 写入并保存 Master ID (0x12) to ADDR:0x07
>
>cansend can1 7FF#0200550712000000
>
>cansend can1 7FF#0200aa0712000000
>
>[2/3] 写入并保存波特率代码 (04) to ADDR:0x23
>
>cansend can1 7FF#0200552304000000
>
>cansend can1 7FF#0200aa2304000000
>
>[3/3] 写入并保存电机 CAN ID (2) to ADDR:0x08
>
>cansend can1 7FF#0200550802000000
>
>cansend can1 7FF#0200aa0802000000
>
>-------------------------------------------------------
>
>指令发送完成！请【彻底断电】并重启电机。
>
>重启后，请记得将主机的波特率也设置为目标值再重新测试。

# 6. 电机测试 r.sh dm.py

sudo ./r.sh

驱动12个电机，CAN0: 显示 01-06，CAN1: 显示 11-16

MIT模式控制电机正弦摆动并回显位置和扭矩

显示CAN通道(0,1)、数字电机ID(1-6)：位置/扭矩

# 8. note
# 8.1 自动生成依赖关系

### 8.1.1 安装 pipreqs

pip install pipreqs

### 8.1.2 生成 requirements.txt

pipreqs ./ --encoding=utf8  --force

### 8.1.3 自动安装环境

pip install -r requirements.txt

## 8.2 视频压缩

sudo apt install ffmpeg

github限制25M,按比例无损压缩体积（调低 CRF 值，数字越高文件越小）

ffmpeg -i input.mp4 -vcodec libx264 -crf 4 output.mp4


apt list --upgradable

sudo apt -o APT::Get::Always-Include-Phased-Updates=true full-upgrade

## 8.3 设置代理 

export all_proxy="socks5://192.168.99.32:2333"

## 8.4 查看网络接口

ip addr show | grep can

## 8.5 卸载pcan

sudo modprobe -r pcan

sudo modprobe peak_usb

## 8.6 原生测试

cansend can0 001#FFFFFFFFFFFFFC

sudo modprobe pcan

 candump can0
 
  can0  001   [8]  23 00 00 00 09 00 00 00
 
  can0  011   [8]  01 7F 66 7F F7 FD 1E 1C

## 8.7 comtool
pip3 install comtool
