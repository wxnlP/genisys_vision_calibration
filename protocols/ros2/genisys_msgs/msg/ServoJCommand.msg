std_msgs/Header header
float64[] target  # 目标关节角 [rad]，长度需 >= 6
float64 dt        # 期望到达时间 [s]，须为正
bool stop         # 最后一条指令置 true，显式停止伺服
