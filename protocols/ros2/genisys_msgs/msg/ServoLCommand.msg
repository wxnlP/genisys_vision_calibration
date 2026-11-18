std_msgs/Header header     # 消息头
geometry_msgs/Pose target  # 目标 TCP 位姿（基座坐标系，位置 [m]）
float64 dt                 # 期望到达时间 [s]（正值）
bool stop                  # 是否在本条指令后停止伺服
