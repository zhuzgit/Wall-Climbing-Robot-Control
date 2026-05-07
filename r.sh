sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 type can bitrate 1000000 fd off
sudo ip link set can1 type can bitrate 1000000 fd off
sudo ip link set can0 up
sudo ip link set can1 up
python3 dm.py
