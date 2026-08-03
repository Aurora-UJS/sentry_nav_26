// 协议层回归：CRC 向量 / 序列化往返 / 断帧粘包噪声鲁棒性
#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include <sentry_serial/protocol.hpp>

using namespace sentry_serial;

namespace
{

// 逐字节喂入，收集所有完整帧的 (cmd_id, payload 拷贝)
struct Collected
{
    uint8_t cmd_id;
    std::vector<uint8_t> payload;
};

std::vector<Collected> feedAll(FrameParser &p, const std::vector<uint8_t> &bytes)
{
    std::vector<Collected> out;
    Frame f;
    for (uint8_t b : bytes)
        if (p.feed(b, f))
            out.push_back({f.cmd_id, {f.payload, f.payload + f.len}});
    return out;
}

CmdVelPacket sampleCmdVel()
{
    CmdVelPacket pkt{};
    pkt.vx_world = 1.5f;
    pkt.vy_world = -0.75f;
    pkt.wz = 3.0f;
    pkt.lio_yaw = -2.5f;
    pkt.lio_yaw_seq = 42;
    pkt.flags = kCmdVelFlagYawValid;
    return pkt;
}

std::vector<uint8_t> encodedCmdVel()
{
    uint8_t buf[kMaxFrame];
    const size_t n = encodeFrame(CMD_VEL, sampleCmdVel(), buf);
    return {buf, buf + n};
}

} // namespace

// ---------------------------------------------------------------- CRC

TEST(Crc16, StandardCheckValue)
{
    // CRC-16/MCRF4XX check value（独立 Python 实现交叉验证）
    const uint8_t data[] = "123456789";
    EXPECT_EQ(crc16(data, 9), 0x6F91);
}

TEST(Crc16, KnownFrameVector)
{
    // 帧头 A5 10 12 + 18 字节全零 payload 的 CRC，与文档 PROTOCOL.md 样例一致
    std::vector<uint8_t> frame = {0xA5, 0x10, 0x12};
    frame.insert(frame.end(), 18, 0x00);
    EXPECT_EQ(crc16(frame.data(), frame.size()), 0x0C4A);
}

// ---------------------------------------------------------------- 布局

TEST(Layout, PacketSizes)
{
    // 与 PROTOCOL.md 的字段表绑定；改动布局必须同步文档与 MCU 端
    EXPECT_EQ(sizeof(CmdVelPacket), 18u);
    EXPECT_EQ(sizeof(ChassisFbPacket), 17u);
    EXPECT_EQ(sizeof(RefereePacket), 17u);
}

// ---------------------------------------------------------------- 往返

TEST(RoundTrip, CmdVel)
{
    FrameParser p;
    auto got = feedAll(p, encodedCmdVel());
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].cmd_id, CMD_VEL);

    Frame f{got[0].cmd_id, static_cast<uint8_t>(got[0].payload.size()), got[0].payload.data()};
    CmdVelPacket rx;
    ASSERT_TRUE(f.to(rx));
    const CmdVelPacket tx = sampleCmdVel();
    EXPECT_EQ(std::memcmp(&tx, &rx, sizeof(tx)), 0);
}

TEST(RoundTrip, Referee)
{
    RefereePacket tx{};
    tx.game_progress = 4;
    tx.stage_remain_time = 351;
    tx.current_hp = 385;
    tx.maximum_hp = 400;
    tx.ammo_17mm = 320;
    tx.rune_flags = kRuneFlagAvailable | kRuneFlagBigActive;
    tx.enemy_outpost_alive = 1;
    tx.enemy_outpost_hp = 1200;
    tx.friend_outpost_hp = 800;
    tx.base_hp = 5000;

    uint8_t buf[kMaxFrame];
    const size_t n = encodeFrame(CMD_REFEREE, tx, buf);

    FrameParser p;
    auto got = feedAll(p, {buf, buf + n});
    ASSERT_EQ(got.size(), 1u);
    Frame f{got[0].cmd_id, static_cast<uint8_t>(got[0].payload.size()), got[0].payload.data()};
    RefereePacket rx;
    ASSERT_TRUE(f.to(rx));
    EXPECT_EQ(std::memcmp(&tx, &rx, sizeof(tx)), 0);
}

TEST(RoundTrip, SizeMismatchRejected)
{
    // 协议版本不匹配兜底：len 与目标结构体不一致时 to() 拒绝
    FrameParser p;
    auto got = feedAll(p, encodedCmdVel());
    ASSERT_EQ(got.size(), 1u);
    Frame f{got[0].cmd_id, static_cast<uint8_t>(got[0].payload.size()), got[0].payload.data()};
    ChassisFbPacket wrong; // 17 != 18
    EXPECT_FALSE(f.to(wrong));
}

// ---------------------------------------------------------------- 流鲁棒性

TEST(Stream, ByteByByteEquivalentToChunk)
{
    // feedAll 本身就是逐字节；此处验证同一帧分两半喂与整帧喂结果一致
    const auto frame = encodedCmdVel();
    FrameParser p;
    const std::vector<uint8_t> first(frame.begin(), frame.begin() + 7);
    const std::vector<uint8_t> second(frame.begin() + 7, frame.end());
    EXPECT_TRUE(feedAll(p, first).empty());
    EXPECT_EQ(feedAll(p, second).size(), 1u);
}

TEST(Stream, TwoConcatenatedFramesDistinctPayloads)
{
    // 两帧不同内容粘连：帧边界错一个字节都会让第二帧 payload 错位，必须断言内容
    CmdVelPacket a = sampleCmdVel();
    CmdVelPacket b = sampleCmdVel();
    b.vx_world = -9.25f;
    b.lio_yaw_seq = 43;

    uint8_t buf[kMaxFrame];
    std::vector<uint8_t> bytes;
    size_t n = encodeFrame(CMD_VEL, a, buf);
    bytes.insert(bytes.end(), buf, buf + n);
    n = encodeFrame(CMD_VEL, b, buf);
    bytes.insert(bytes.end(), buf, buf + n);

    FrameParser p;
    auto got = feedAll(p, bytes);
    ASSERT_EQ(got.size(), 2u);
    CmdVelPacket rx0, rx1;
    Frame f0{got[0].cmd_id, static_cast<uint8_t>(got[0].payload.size()), got[0].payload.data()};
    Frame f1{got[1].cmd_id, static_cast<uint8_t>(got[1].payload.size()), got[1].payload.data()};
    ASSERT_TRUE(f0.to(rx0));
    ASSERT_TRUE(f1.to(rx1));
    EXPECT_EQ(std::memcmp(&a, &rx0, sizeof(a)), 0);
    EXPECT_EQ(std::memcmp(&b, &rx1, sizeof(b)), 0);
}

TEST(RoundTrip, MaxPayloadBoundary)
{
    // len == kMaxPayload(48) 的满载帧：守卫从 > 回归成 >= 或 buf_ 容量差 1 都只有它能暴露
    uint8_t payload[kMaxPayload];
    for (size_t i = 0; i < sizeof(payload); ++i)
        payload[i] = static_cast<uint8_t>(i * 7 + 1);

    uint8_t buf[kMaxFrame];
    buf[0] = kSof;
    buf[1] = 0x20;
    buf[2] = kMaxPayload;
    std::memcpy(buf + 3, payload, sizeof(payload));
    const uint16_t c = crc16(buf, 3 + kMaxPayload);
    buf[3 + kMaxPayload] = static_cast<uint8_t>(c & 0xFF);
    buf[4 + kMaxPayload] = static_cast<uint8_t>(c >> 8);

    FrameParser p;
    auto got = feedAll(p, {buf, buf + kMaxFrame});
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(p.stats().len_errors, 0u);
    ASSERT_EQ(got[0].payload.size(), kMaxPayload);
    EXPECT_EQ(std::memcmp(got[0].payload.data(), payload, kMaxPayload), 0);
}

TEST(Stream, CrcByteCorruptionDropped)
{
    // CRC 自身首/末字节被破坏都必须整帧拒绝：只比低字节/只比高字节的退化实现在此现形
    for (size_t off : {size_t(21), size_t(22)}) // len=18 帧的 CRC 两个字节
    {
        auto bad = encodedCmdVel();
        ASSERT_EQ(bad.size(), 23u);
        bad[off] ^= 0xFF;
        FrameParser p;
        EXPECT_TRUE(feedAll(p, bad).empty()) << "corrupted CRC byte at " << off;
        EXPECT_EQ(p.stats().crc_errors, 1u);
    }
}

TEST(Stream, MidFrameDesyncRecovers)
{
    // 半截帧（线路中断）后跟两个完整帧：解析器把第一个完整帧的头吃进旧帧 payload，
    // CRC 兜底拒绝后，第二个完整帧必须恢复解析（丢帧上限 = 1）
    const auto frame = encodedCmdVel();
    std::vector<uint8_t> bytes(frame.begin(), frame.begin() + 10); // 只传了 10 字节就断
    bytes.insert(bytes.end(), frame.begin(), frame.end());
    bytes.insert(bytes.end(), frame.begin(), frame.end());

    FrameParser p;
    auto got = feedAll(p, bytes);
    EXPECT_GE(got.size(), 1u);
    EXPECT_EQ(p.stats().crc_errors + p.stats().len_errors, 1u);
    // 恢复后最后一帧内容必须完好
    CmdVelPacket rx;
    Frame f{got.back().cmd_id, static_cast<uint8_t>(got.back().payload.size()),
            got.back().payload.data()};
    ASSERT_TRUE(f.to(rx));
    const CmdVelPacket tx = sampleCmdVel();
    EXPECT_EQ(std::memcmp(&tx, &rx, sizeof(tx)), 0);
}

TEST(Stream, CrcCorruptionDroppedThenRecovers)
{
    auto bad = encodedCmdVel();
    bad[5] ^= 0xFF; // 破坏 payload
    auto good = encodedCmdVel();
    bad.insert(bad.end(), good.begin(), good.end());

    FrameParser p;
    auto got = feedAll(p, bad);
    ASSERT_EQ(got.size(), 1u); // 坏帧丢弃，后续帧正常
    EXPECT_EQ(p.stats().crc_errors, 1u);
    EXPECT_EQ(p.stats().frames_ok, 1u);
}

TEST(Stream, GarbagePrefixThenFrame)
{
    std::vector<uint8_t> bytes = {0x00, 0xFF, 0x3C, 0x7E}; // 线路噪声
    const auto frame = encodedCmdVel();
    bytes.insert(bytes.end(), frame.begin(), frame.end());

    FrameParser p;
    EXPECT_EQ(feedAll(p, bytes).size(), 1u);
    EXPECT_EQ(p.stats().bytes_dropped, 4u);
}

TEST(Stream, OversizeLenRejectedThenRecovers)
{
    // 伪 SOF + 超限 len：应记 len_error 并回到同步态，不吞掉后续真帧
    std::vector<uint8_t> bytes = {kSof, 0x10, static_cast<uint8_t>(kMaxPayload + 1)};
    const auto frame = encodedCmdVel();
    bytes.insert(bytes.end(), frame.begin(), frame.end());

    FrameParser p;
    EXPECT_EQ(feedAll(p, bytes).size(), 1u);
    EXPECT_EQ(p.stats().len_errors, 1u);
}

TEST(Stream, PayloadContainingSofParsesFine)
{
    // payload 内出现 0xA5 不得干扰帧边界（状态机按 len 计数，不搜 SOF）
    CmdVelPacket pkt{};
    std::memset(&pkt, 0xA5, sizeof(pkt));
    uint8_t buf[kMaxFrame];
    const size_t n = encodeFrame(CMD_VEL, pkt, buf);

    FrameParser p;
    auto got = feedAll(p, {buf, buf + n});
    ASSERT_EQ(got.size(), 1u);
    Frame f{got[0].cmd_id, static_cast<uint8_t>(got[0].payload.size()), got[0].payload.data()};
    CmdVelPacket rx;
    ASSERT_TRUE(f.to(rx));
    EXPECT_EQ(std::memcmp(&pkt, &rx, sizeof(pkt)), 0);
}

TEST(Stream, ZeroLenFrameSupported)
{
    // len=0 的心跳帧（预留用法）应能通过
    uint8_t buf[kFrameOverhead];
    buf[0] = kSof;
    buf[1] = 0x7F;
    buf[2] = 0;
    const uint16_t c = crc16(buf, 3);
    buf[3] = static_cast<uint8_t>(c & 0xFF);
    buf[4] = static_cast<uint8_t>(c >> 8);

    FrameParser p;
    auto got = feedAll(p, {buf, buf + sizeof(buf)});
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].cmd_id, 0x7F);
    EXPECT_TRUE(got[0].payload.empty());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
