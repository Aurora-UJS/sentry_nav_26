# sentry_serial 串口协议 v0.1

上位机（导航 NUC）↔ 底盘电控 MCU 的 UART 协议。本文档是双端对接的唯一规格：
上位机实现在 `include/sentry_serial/protocol.hpp`（与本文档绑定，gtest 校验布局），
MCU 端照本文档实现。

## 0. 职责划分（先读这个）

```
导航栈(50Hz) → /cmd_vel_world(odom 系速度) → sentry_serial_node → UART → MCU
                                                                      MCU: 1kHz 陀螺积分 yaw
                                                                           世界系→机体系旋转
                                                                           → 轮速解算
```

**世界系→机体系的旋转在 MCU 做，不在上位机做。** 原因：自转 3 rad/s 时航向每
毫秒变 0.17°，旋转必须发生在航向最新鲜的地方（MCU 陀螺 1kHz）。上位机 LIO yaw
有 50~70ms 龄期，只用作 MCU 陀螺积分的零漂校准源，随速度指令包下发。

仿真里 `chassis_cmd_node`（`sentry_local_planner/src/chassis_cmd_node.cpp`）扮演
MCU 的角色，跑的是与 MCU 相同的算法——仿真与真车同架构同接口。

## 1. 物理层

| 项 | 值 |
|---|---|
| 电平 | 3.3V TTL（C 板 USB CDC 虚拟串口亦可） |
| 波特率 | **921600**（可配 115200/230400/460800） |
| 帧格式 | 8N1 |
| 字节序 | **小端**；float 为 IEEE-754 单精度 |

带宽核算 @921600（≈92 KB/s）：上行 0x10 50Hz×23B ≈ 1.2 KB/s，
下行 0x01 100Hz×22B + 0x02 10Hz×22B ≈ 2.4 KB/s。合计 <4%，余量充足。

## 2. 帧格式

```
偏移   长度   字段
0      1     SOF = 0xA5
1      1     cmd_id
2      1     len            payload 字节数，<= 48
3      len   payload        小端
3+len  2     CRC-16/MCRF4XX 覆盖 [0, 3+len)，小端存放
```

帧总长 = len + 5。**CRC 参数**（与 DJI 裁判系统串口协议 CRC16 相同，电控可复用
现有查表代码）：

| 参数 | 值 |
|---|---|
| poly | 0x1021（反射形式 0x8408） |
| init | 0xFFFF |
| refin / refout | true / true |
| xorout | 0x0000 |
| check("123456789") | **0x6F91** |

参考实现（按位，无查表）：

```c
uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    }
    return crc;
}
```

**测试向量**：帧 `A5 10 12` + 18 字节全零 payload 的 CRC = `0x0C4A`
（帧尾两字节为 `4A 0C`）。双端实现先对齐这个向量再联调。

**解析要求（MCU 端同样建议）**：逐字节状态机（找 SOF → 读 cmd/len → 收满
payload+CRC → 校验），CRC 失败丢弃当前候选帧、回到找 SOF 状态。payload 中出现
0xA5 无需转义——帧边界由 len 决定，误同步由 CRC 兜底。已知取舍：len 字节本身被
噪声破坏时可能连带丢失下一帧，由指令流的连续性（50Hz）与看门狗兜底。

## 3. 包定义

### 0x10 CmdVel（上位机 → MCU，@50Hz，len=18）

| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | float | vx_world | m/s，odom 系 |
| 4 | float | vy_world | m/s，odom 系 |
| 8 | float | wz | rad/s，自转/转向角速度，直接执行 |
| 12 | float | lio_yaw | rad，odom 系，LIO 航向（校准源） |
| 16 | uint8 | lio_yaw_seq | LIO 新样本时 +1（自然回绕） |
| 17 | uint8 | flags | bit0: lio_yaw 有效；无效时忽略 yaw/seq |

**MCU 端处理**：

```
每个陀螺样本(1kHz):
    yaw += wz_gyro * dt                       # 积分（wrap 到 [-π, π]）
收到 0x10:
    if flags.bit0 && seq != last_seq:         # 仅新 LIO 样本校准一次
        last_seq = seq
        if !initialized:  yaw = lio_yaw; initialized = true
        elif |wz_gyro| < 0.5:                 # 门控：自转中 LIO yaw 滞后不可信
            yaw += 1.0 * 0.05 * wrap(lio_yaw - yaw)   # 增益 1.0/s × 标称周期 0.05s
    if initialized:
        vx_body =  cos(yaw)*vx_world + sin(yaw)*vy_world
        vy_body = -sin(yaw)*vx_world + cos(yaw)*vy_world
        执行 (vx_body, vy_body, wz)
    else:
        输出零速                                # yaw 未初始化拒绝执行平移
看门狗: 距上一有效 0x10 超过 200ms → 输出零速
```

（算法与上位机仿真实现 `chassis_yaw_estimator.hpp` 逐行对应；gate 0.5 rad/s、
增益 1.0 与仿真参数一致，联调时两端要同一组值。）

**为什么要 seq**：50Hz 指令包会重复携带同一个 20Hz LIO 样本。若每包都校准，
等效校准频率 ×2.5，收敛时间常数被悄悄缩短。seq 不变则跳过。

### 0x01 ChassisFb（MCU → 上位机，建议 ≥100Hz，len=17）

| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | float | yaw | MCU 积分航向（rad，odom 系，校准后） |
| 4 | float | wz | 底盘角速度 rad/s |
| 8 | float | vx_body | 轮系解算机体系速度 m/s（未实现填 0） |
| 12 | float | vy_body | 同上 |
| 16 | uint8 | flags | bit0: yaw 已初始化 |

上位机发布为 `/chassis/feedback`（`sentry_msgs/ChassisFeedback`）。首要用途：
与 LIO yaw 对比验证两端航向估计一致性（标定 correction_gain / 排查旋转方向）。

### 0x02 Referee（MCU → 上位机，@10Hz，len=17）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | uint8 | game_progress（4=比赛中） |
| 1 | uint16 | stage_remain_time (s) |
| 3 | uint16 | current_hp |
| 5 | uint16 | maximum_hp |
| 7 | uint16 | ammo_17mm（剩余允许发弹量） |
| 9 | uint8 | rune_flags：bit0 可激活窗口 / bit1 小符已激活 / bit2 大符已激活 |
| 10 | uint8 | enemy_outpost_alive（0/1） |
| 11 | uint16 | enemy_outpost_hp |
| 13 | uint16 | friend_outpost_hp |
| 15 | uint16 | base_hp |

上位机发布为 `/referee/state`（`sentry_msgs/RefereeState`）。字段集刻意最小化
（HERO 开源的教训：搬全裁判系统 73 字段，实战消费不足 1/5）。**加字段的前提是
决策层有了消费者**；追加时在包尾扩展并同步改 len 与本表，双端一起升版本。

裁判系统原始数据的敌我判定在 MCU 侧做完（上位机只见 friend/enemy 语义，不出现
红蓝 id 判断）。

## 4. cmd_id 分配与演进规则

| 范围 | 用途 |
|---|---|
| 0x01~0x0F | 下行（MCU → 上位机） |
| 0x10~0x1F | 上行（上位机 → MCU） |
| 0x00, 0xF0~0xFF | 保留 |

- 收到未知 cmd_id：跳过该帧（按 len 消费），不视为错误——允许一端先升级
- 收到已知 cmd_id 但 len 不符：丢弃并计数——版本不匹配的信号
- 上位机侧统计（tx/rx/CRC 错/长度错/重连）每 5s 打一行 INFO 日志

## 5. 联调步骤建议

1. 双端先对齐 CRC 测试向量（§2 两个值）
2. MCU 只发 0x01（yaw 未初始化 flags=0），上位机 `ros2 topic echo /chassis/feedback`
3. 上位机 `ros2 topic pub /cmd_vel_world`（低速），验证 MCU 收包、看门狗（停发→刹停）
4. 接入 LIO 后验证 yaw 初始化与校准：原地慢速平移，`/chassis/feedback` 的 yaw
   应收敛到 LIO yaw；随后自转，验证门控生效（自转中 yaw 靠积分不跳变）
5. 无硬件时上位机可用 `socat -d -d pty,raw,echo=0 pty,raw,echo=0` 建虚拟串口对，
   一端指给 `port` 参数、另一端用脚本回放帧
