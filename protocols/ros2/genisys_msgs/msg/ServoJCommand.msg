std_msgs/Header header  # 消息头
float64[] target        # 目标关节角 [rad]（至少 6 项，取前 6 项）
float64 dt              # 期望到达时间 [s]（正值）
bool stop               # 是否在本条指令后停止伺服
