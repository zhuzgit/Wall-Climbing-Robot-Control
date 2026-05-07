#!/bin/bash
#see can data
# candump can1 
#  can1  7FF   [8]  06 00 55 08 09 00 00 00
#  can1  019   [8]  09 00 55 08 09 00 00 00 id is 09 now
#  can1  7FF   [8]  06 00 AA 08 09 00 00 00 no message return
#  can1  7FF   [8]  09 00 AA 08 09 00 00 00 id 09 send 
#  can1  019   [4]  09 00 AA 01             message return ok!     
# =========================================================
# 1. 参数设定区
# =========================================================
CAN_IF="can1"

# --- 初始连接参数 (确保当前能通) ---
INIT_BITRATE=1000000
INIT_DBITRATE=1000000 #1M~5M

# --- 目标修改参数 ---
CURRENT_ID=3     # 电机当前的 ID (十进制)
TARGET_ID=2      # 准备修改成的目标 ID (十进制)

# 目标波特率代码: 04 对应 1M, 09 对应 5M
TARGET_BAUD_HEX="04" 

# =========================================================
# 2. 自动计算 (十六进制)
# =========================================================
# 计算 Master ID = 0x10 + ID (十六进制加法)
# 如果 ID=1, Master ID=0x11; 如果 ID=6, Master ID=0x16
MASTER_ID_HEX=$(printf "%02x" $((0x10 + TARGET_ID)))

# 将十进制 ID 转换为双字节小端十六进制 (用于指令头)
# 电机当前 ID: 5 -> 05 00
CUR_ID_L=$(printf "%02x" $((CURRENT_ID & 0xFF)))
CUR_ID_H=$(printf "%02x" $(( (CURRENT_ID >> 8) & 0xFF )))

# 目标 ID: 1 -> 01 00
TAR_ID_L=$(printf "%02x" $((TARGET_ID & 0xFF)))
TAR_ID_H=$(printf "%02x" $(( (TARGET_ID >> 8) & 0xFF )))

echo "-------------------------------------------------------"
echo "电机当前 ID: $CURRENT_ID (前缀: ${CUR_ID_H}${CUR_ID_L})"
echo "目标修改为 ID: $TARGET_ID"
echo "自动计算 Master ID: 0x$MASTER_ID_HEX"
echo "-------------------------------------------------------"

# =========================================================
# 3. 执行指令序列
# =========================================================

# 步骤 1: 初始化 CAN 接口
sudo ip link set $CAN_IF down
sudo ip link set $CAN_IF type can bitrate $INIT_BITRATE dbitrate $INIT_DBITRATE fd on
sudo ip link set $CAN_IF up
sleep 0.5

# 步骤 1.1: 强制禁用电机 (Disabled Mode)
# 手册指出：保存参数必须在 Disabled Mode 下执行。
# 禁用指令格式：当前ID# FFFFFFFFFFFFFFFD
echo "发送禁用指令 (确保进入可写模式)..."
echo "cansend $CAN_IF 7FF#FFFFFFFFFFFFFFFD"
cansend $CAN_IF 7FF#FFFFFFFFFFFFFFFD
sleep 0.2



# 步骤 2: 修改 Master ID (寄存器 0x07)
# 格式: 7FF# [当前ID_L][当前ID_H] 55 07 [MasterID] 00 00 00
echo "[1/3] 写入并保存 Master ID (0x$MASTER_ID_HEX) to ADDR:0x07"
echo "cansend $CAN_IF 7FF#${CUR_ID_L}${CUR_ID_H}5507${MASTER_ID_HEX}000000"
cansend $CAN_IF 7FF#${CUR_ID_L}${CUR_ID_H}5507${MASTER_ID_HEX}000000
sleep 0.1
echo "cansend $CAN_IF 7FF#${CUR_ID_L}${CUR_ID_H}aa07${MASTER_ID_HEX}000000"
cansend $CAN_IF 7FF#${CUR_ID_L}${CUR_ID_H}aa07${MASTER_ID_HEX}000000
sleep 0.1

# 步骤 3: 修改波特率 (寄存器 0x23)
echo "[2/3] 写入并保存波特率代码 ($TARGET_BAUD_HEX) to ADDR:0x23"
echo "cansend $CAN_IF 7FF#${CUR_ID_L}${CUR_ID_H}5523${TARGET_BAUD_HEX}000000"
cansend $CAN_IF 7FF#${CUR_ID_L}${CUR_ID_H}5523${TARGET_BAUD_HEX}000000
sleep 0.1
echo "cansend $CAN_IF 7FF#${CUR_ID_L}${CUR_ID_H}aa23${TARGET_BAUD_HEX}000000"
cansend $CAN_IF 7FF#${CUR_ID_L}${CUR_ID_H}aa23${TARGET_BAUD_HEX}000000
sleep 0.1

# 步骤 4: 修改电机 CAN ID (寄存器 0x08)
# 格式: 7FF# [当前ID_L][当前ID_H] 55 08 [新ID_L][新ID_H] 00 00
echo "[3/3] 写入并保存电机 CAN ID ($TARGET_ID) to ADDR:0x08"
echo "cansend $CAN_IF 7FF#${CUR_ID_L}${CUR_ID_H}5508${TAR_ID_L}${TAR_ID_H}0000"
cansend $CAN_IF 7FF#${CUR_ID_L}${CUR_ID_H}5508${TAR_ID_L}${TAR_ID_H}0000
sleep 0.1
echo "cansend $CAN_IF 7FF#${TAR_ID_L}${TAR_ID_H}aa08${TAR_ID_L}${TAR_ID_H}0000"
cansend $CAN_IF 7FF#${TAR_ID_L}${TAR_ID_H}aa08${TAR_ID_L}${TAR_ID_H}0000
sleep 0.1


echo "-------------------------------------------------------"
echo "指令发送完成！请【彻底断电】并重启电机。"
echo "重启后，请记得将主机的波特率也设置为目标值再重新测试。"
