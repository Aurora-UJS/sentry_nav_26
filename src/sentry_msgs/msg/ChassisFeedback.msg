# 底盘 MCU 反馈（串口下行 0x01 → sentry_serial_node 发布）
# MCU 侧陀螺积分 yaw：与上位机 LIO yaw 交叉验证两端估计一致性（标定 correction_gain 用）
std_msgs/Header header

float32 yaw              # MCU 陀螺积分航向 (rad, odom 系，经 LIO 校准)
float32 wz               # 底盘角速度 (rad/s)
float32 vx_body          # 轮系解算机体系速度 (m/s，电控未实现则为 0)
float32 vy_body
bool yaw_initialized     # MCU 端 yaw 是否已用 LIO 样本完成初始化
