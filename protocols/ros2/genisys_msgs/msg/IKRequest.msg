uint8 FRAME_UNSPECIFIED = 0  # 未指定坐标系
uint8 FRAME_TCP = 1          # TCP 位姿
uint8 FRAME_FLANGE = 2       # 法兰位姿

uint64 id                    # 调用方指定的查询编号
uint8 frame                  # 目标位姿类型（FRAME_TCP 或 FRAME_FLANGE）
bool do_fk                   # 是否执行正解验证
geometry_msgs/Pose pose      # 目标位姿（基座坐标系，位置 [m]）
