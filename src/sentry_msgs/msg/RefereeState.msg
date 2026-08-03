# 裁判系统状态（串口下行 0x02 → sentry_serial_node 发布）
# 字段集按「决策层实际消费」最小化起步，扩展前先确认有消费者
std_msgs/Header header

uint8 game_progress          # 0未开始 1准备 2自检 3五秒倒计时 4比赛中 5结算
uint16 stage_remain_time     # 当前阶段剩余时间 (s)

uint16 current_hp            # 本机血量
uint16 maximum_hp
uint16 ammo_17mm             # 剩余允许发弹量

bool rune_available          # 能量机关处于可激活窗口
bool small_rune_active       # 小符已被激活（任一方）
bool big_rune_active

bool enemy_outpost_alive
uint16 enemy_outpost_hp
uint16 friend_outpost_hp
uint16 base_hp               # 己方基地血量
