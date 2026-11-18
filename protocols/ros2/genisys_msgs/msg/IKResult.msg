uint64 id     # 对应的查询编号
uint8 status  # 逆解状态码（见 IKCheck 响应常量）
bool solved   # 逆解是否成功（status 为 OK）
Joint joint   # 最优解关节角 [rad]（仅 status 为 OK 时有效）
