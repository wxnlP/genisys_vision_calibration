std_msgs/Header header
geometry_msgs/Pose target  # 目标末端位姿
float64 dt                 # 期望到达时间 [s]，须为正
bool stop                  # 最后一条指令置 true，显式停止伺服
