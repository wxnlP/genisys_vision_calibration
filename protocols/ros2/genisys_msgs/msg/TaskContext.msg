std_msgs/Header header                 # 消息头（stamp 为状态生成时刻）

uint64 id                              # 任务编号（从 1 开始单调递增）
float64 duration_ms                    # 规划执行时长 [ms]
builtin_interfaces/Time end_timestamp  # 任务结束时刻（未结束时为 0）

int8 UNKNOWN = -1                      # 无任务
int8 RUNNING = 0                       # 执行中
int8 SAMPLE_ERROR = 1                  # 轨迹采样失败
int8 TOLER_ERROR = 2                   # 未达到到位容差
int8 FINISHED = 3                      # 正常完成
int8 state                             # 任务状态

int32 code                             # 任务结果码（0：正常，-2：采样失败，-3：未达到到位容差）
