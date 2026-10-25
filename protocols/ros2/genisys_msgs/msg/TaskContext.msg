std_msgs/Header header  # stamp：本条状态产生的时刻

uint64 id               # 任务编号，从 1 开始单调递增
float64 duration_ms     # 规划给出的预计执行时长 [ms]
builtin_interfaces/Time end_timestamp  # 任务结束时刻，未结束时为 0

int8 UNKNOWN = -1       # 无任务
int8 RUNNING = 0        # 已受理，正在生产帧
int8 SAMPLE_ERROR = 1   # 采样过程中某一帧算不出来
int8 TOLER_ERROR = 2    # 采样正常，但没有收敛到到位容差
int8 FINISHED = 3       # 正常完成
int8 state

int32 code              # 0 正常；-2 采样错误；-3 容差错误
