#pragma once
/**
 * sentry_serial 协议层：帧格式、CRC、包定义、逐字节解析状态机（纯逻辑，无 ROS 依赖）
 *
 * 帧格式（小端，完整规格见 PROTOCOL.md）：
 *   [0]        SOF = 0xA5
 *   [1]        cmd_id
 *   [2]        len          (payload 字节数, <= kMaxPayload)
 *   [3..3+len) payload
 *   [3+len,+2) CRC-16/MCRF4XX  (覆盖 [0, 3+len)，小端)
 *
 * 设计约束：
 *   - 解析器逐字节喂入，对断帧/粘包/线路噪声鲁棒：任何校验失败只丢当前候选帧，
 *     从下一个 0xA5 重新同步（payload 中出现 0xA5 靠 CRC 兜底拒绝）
 *   - 包结构体 pack(1) + 小端直拷：x86 / ARM(Jetson, STM32) 均为小端，
 *     跨端序平台移植时需改用逐字段序列化
 */

#include <cstdint>
#include <cstring>

namespace sentry_serial
{

// ---------------------------------------------------------------- 常量

constexpr uint8_t kSof = 0xA5;
constexpr size_t kMaxPayload = 48;              // 单帧 payload 上限
constexpr size_t kFrameOverhead = 5;            // SOF + cmd + len + crc16
constexpr size_t kMaxFrame = kMaxPayload + kFrameOverhead;

// cmd_id 分配：0x0x 下行（MCU → 上位机），0x1x 上行（上位机 → MCU）
// 0x00 与 0xF0 以上保留
enum CmdId : uint8_t
{
    CMD_CHASSIS_FB = 0x01,   // 底盘反馈（MCU yaw / wz / 轮系速度）
    CMD_REFEREE    = 0x02,   // 裁判系统状态
    CMD_VEL        = 0x10,   // 世界系速度指令 + LIO yaw 校准源
};

// ---------------------------------------------------------------- CRC

/**
 * CRC-16/MCRF4XX: poly=0x1021(reflected 0x8408), init=0xFFFF,
 * refin=true, refout=true, xorout=0x0000。check("123456789") = 0x6F91。
 * 与 DJI 裁判系统串口协议 CRC16 同参数，电控侧可复用现有查表实现。
 */
inline uint16_t crc16(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF)
{
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    }
    return crc;
}

// ---------------------------------------------------------------- 包定义

#pragma pack(push, 1)

/** 0x10 上行 @50Hz：cmd_vel_world 直转 + LIO yaw 校准源 */
struct CmdVelPacket
{
    float vx_world;       // m/s, odom 系
    float vy_world;       // m/s, odom 系
    float wz;             // rad/s（自转/对齐转向，MCU 直接执行）
    float lio_yaw;        // rad, odom 系（MCU 陀螺积分的校准源）
    uint8_t lio_yaw_seq;  // 新 LIO 样本时 +1（回绕）；MCU 仅在 seq 变化时执行一次校准
                          // —— 否则 50Hz 重复样本会把校准时间常数缩短 2.5 倍
    uint8_t flags;        // bit0: lio_yaw 有效（LIO 未初始化时为 0，MCU 须忽略 yaw 字段）
};
static_assert(sizeof(CmdVelPacket) == 18, "CmdVelPacket layout drift");

constexpr uint8_t kCmdVelFlagYawValid = 0x01;

/** 0x01 下行 @>=100Hz：MCU 侧状态回传（两端 yaw 估计交叉验证） */
struct ChassisFbPacket
{
    float yaw;        // MCU 陀螺积分航向 (rad, odom 系, 经 LIO 校准)
    float wz;         // rad/s
    float vx_body;    // 轮系解算机体系速度 (m/s, 未实现填 0)
    float vy_body;
    uint8_t flags;    // bit0: yaw 已初始化（未初始化时 MCU 拒绝执行速度指令）
};
static_assert(sizeof(ChassisFbPacket) == 17, "ChassisFbPacket layout drift");

constexpr uint8_t kChassisFbFlagYawInit = 0x01;

/** 0x02 下行 @10Hz：裁判系统状态（字段集与 sentry_msgs/RefereeState 一一对应） */
struct RefereePacket
{
    uint8_t game_progress;        // 4 = 比赛中
    uint16_t stage_remain_time;   // s
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t ammo_17mm;
    uint8_t rune_flags;           // bit0 可激活窗口, bit1 小符已激活, bit2 大符已激活
    uint8_t enemy_outpost_alive;  // 0/1
    uint16_t enemy_outpost_hp;
    uint16_t friend_outpost_hp;
    uint16_t base_hp;
};
static_assert(sizeof(RefereePacket) == 17, "RefereePacket layout drift");

constexpr uint8_t kRuneFlagAvailable = 0x01;
constexpr uint8_t kRuneFlagSmallActive = 0x02;
constexpr uint8_t kRuneFlagBigActive = 0x04;

#pragma pack(pop)

// ---------------------------------------------------------------- 编码

/** 组帧到 out（容量须 >= sizeof(T) + kFrameOverhead），返回帧总长。 */
template <typename T>
inline size_t encodeFrame(uint8_t cmd_id, const T &pkt, uint8_t *out)
{
    static_assert(sizeof(T) <= kMaxPayload, "payload too large");
    out[0] = kSof;
    out[1] = cmd_id;
    out[2] = static_cast<uint8_t>(sizeof(T));
    std::memcpy(out + 3, &pkt, sizeof(T));
    const uint16_t c = crc16(out, 3 + sizeof(T));
    out[3 + sizeof(T)] = static_cast<uint8_t>(c & 0xFF);
    out[4 + sizeof(T)] = static_cast<uint8_t>(c >> 8);
    return sizeof(T) + kFrameOverhead;
}

// ---------------------------------------------------------------- 解析

/** 一帧解析结果：payload 指向解析器内部缓冲，仅在下一次 feed 前有效。 */
struct Frame
{
    uint8_t cmd_id = 0;
    uint8_t len = 0;
    const uint8_t *payload = nullptr;

    /** payload 长度与 T 一致时拷出，否则返回 false（协议版本不匹配的兜底）。 */
    template <typename T>
    bool to(T &out) const
    {
        if (len != sizeof(T))
            return false;
        std::memcpy(&out, payload, sizeof(T));
        return true;
    }
};

/**
 * 逐字节解析状态机。用法：
 *   Frame f;
 *   for (byte : chunk) if (parser.feed(byte, f)) handle(f);
 */
class FrameParser
{
public:
    struct Stats
    {
        uint64_t frames_ok = 0;
        uint64_t crc_errors = 0;
        uint64_t len_errors = 0;    // len > kMaxPayload，视为伪 SOF
        uint64_t bytes_dropped = 0; // 同步状态下丢弃的非 SOF 字节
    };

    /** 喂入一个字节；恰好完成一帧时返回 true 并填充 out。 */
    bool feed(uint8_t byte, Frame &out)
    {
        switch (state_)
        {
        case State::SOF:
            if (byte == kSof)
            {
                buf_[0] = byte;
                pos_ = 1;
                state_ = State::HEADER;
            }
            else
            {
                ++stats_.bytes_dropped;
            }
            return false;

        case State::HEADER: // cmd_id, len
            buf_[pos_++] = byte;
            if (pos_ == 3)
            {
                if (buf_[2] > kMaxPayload)
                {
                    ++stats_.len_errors;
                    reset();
                    return false;
                }
                need_ = 3 + buf_[2] + 2;
                state_ = State::BODY;
            }
            return false;

        case State::BODY: // payload + crc
            buf_[pos_++] = byte;
            if (pos_ < need_)
                return false;

            {
                const size_t crc_off = need_ - 2;
                const uint16_t rx = static_cast<uint16_t>(buf_[crc_off]) |
                                    (static_cast<uint16_t>(buf_[crc_off + 1]) << 8);
                const bool ok = (crc16(buf_, crc_off) == rx);
                if (ok)
                {
                    ++stats_.frames_ok;
                    out.cmd_id = buf_[1];
                    out.len = buf_[2];
                    out.payload = buf_ + 3;
                }
                else
                {
                    ++stats_.crc_errors;
                }
                reset();
                return ok;
            }
        }
        return false; // unreachable
    }

    const Stats &stats() const { return stats_; }

private:
    enum class State { SOF, HEADER, BODY };

    void reset()
    {
        state_ = State::SOF;
        pos_ = 0;
        need_ = 0;
    }

    State state_ = State::SOF;
    uint8_t buf_[kMaxFrame] = {};
    size_t pos_ = 0;
    size_t need_ = 0;
    Stats stats_;
};

} // namespace sentry_serial
