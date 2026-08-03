/**
 * sentry_serial_node: 真车串口桥 —— 仿真侧 gz bridge + chassis_cmd_node 的真车替身
 *
 * 上行（→ MCU, 0x10 @导航栈 50Hz 驱动）：
 *   /cmd_vel_world (Twist, odom 系) + LIO yaw（/Odometry 提取，带新鲜度 seq）
 *   —— MCU 端跑与 chassis_yaw_estimator.hpp 相同的算法（1kHz 陀螺积分 + 门控校准），
 *      世界系→机体系旋转在 MCU 执行（旋转必须发生在航向最新鲜的地方）
 *   —— 上位机停发（节点挂/线断）= 不再有帧，MCU 端 200ms 看门狗刹停，
 *      与仿真 chassis_cmd_node 的 cmd_timeout 行为等价
 *
 * 下行（← MCU）：
 *   0x01 底盘反馈 → /chassis/feedback（两端 yaw 估计交叉验证/标定 correction_gain）
 *   0x02 裁判系统 → /referee/state（决策层数据源）
 *
 * 线程模型：ROS 回调（单线程 executor）负责写；独立读线程负责 poll/read/重连。
 * write 与 close/open 共用 io_mutex_ —— 仅有 atomic fd 时，load 与 write 之间
 * 读线程可能 close 且 fd 号被复用（POSIX 最小编号复用），帧会杂散写入无关 fd。
 * 读线程 read 不持锁（read 期间 fd 不会被别的线程关闭：close 只发生在读线程自身）。
 */

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sentry_msgs/msg/chassis_feedback.hpp>
#include <sentry_msgs/msg/referee_state.hpp>

#include <sentry_serial/protocol.hpp>

namespace
{

speed_t baudConstant(int baud)
{
    switch (baud)
    {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
    }
}

} // namespace

class SentrySerialNode : public rclcpp::Node
{
public:
    SentrySerialNode() : Node("sentry_serial")
    {
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        baud_ = static_cast<int>(this->declare_parameter<int>("baud", 921600));
        reconnect_period_ = this->declare_parameter<double>("reconnect_period", 1.0);

        const auto cmd_topic = this->declare_parameter<std::string>("cmd_world_topic", "/cmd_vel_world");
        const auto odom_topic = this->declare_parameter<std::string>("odom_topic", "/Odometry");
        const auto fb_topic = this->declare_parameter<std::string>("feedback_topic", "/chassis/feedback");
        const auto ref_topic = this->declare_parameter<std::string>("referee_topic", "/referee/state");

        if (baudConstant(baud_) == 0)
        {
            RCLCPP_FATAL(this->get_logger(),
                         "Unsupported baud %d (supported: 115200/230400/460800/921600)", baud_);
            throw std::invalid_argument("unsupported baud");
        }

        fb_pub_ = this->create_publisher<sentry_msgs::msg::ChassisFeedback>(
            fb_topic, rclcpp::SensorDataQoS());
        ref_pub_ = this->create_publisher<sentry_msgs::msg::RefereeState>(ref_topic, 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                const auto &q = msg->pose.pose.orientation;
                // 与 chassis_cmd_node 同一 yaw 提取式
                lio_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                ++lio_yaw_seq_;
                lio_yaw_valid_ = true;
            });

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_topic, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { sendCmdVel(*msg); });

        stats_timer_ = this->create_wall_timer(std::chrono::seconds(5),
                                               [this]() { logStats(); });

        running_ = true;
        io_thread_ = std::thread([this]() { ioLoop(); });

        RCLCPP_INFO(this->get_logger(), "sentry_serial: %s @ %d, %s -> UART, UART -> %s + %s",
                    port_.c_str(), baud_, cmd_topic.c_str(), fb_topic.c_str(), ref_topic.c_str());
    }

    ~SentrySerialNode() override
    {
        running_ = false;
        if (io_thread_.joinable())
            io_thread_.join();
        closePort();
    }

private:
    // ------------------------------------------------------------ 上行

    void sendCmdVel(const geometry_msgs::msg::Twist &cmd)
    {
        sentry_serial::CmdVelPacket pkt{};
        pkt.vx_world = static_cast<float>(cmd.linear.x);
        pkt.vy_world = static_cast<float>(cmd.linear.y);
        pkt.wz = static_cast<float>(cmd.angular.z);
        pkt.lio_yaw = static_cast<float>(lio_yaw_);
        pkt.lio_yaw_seq = lio_yaw_seq_;
        pkt.flags = lio_yaw_valid_ ? sentry_serial::kCmdVelFlagYawValid : 0;

        uint8_t buf[sizeof(pkt) + sentry_serial::kFrameOverhead];
        const size_t n = sentry_serial::encodeFrame(sentry_serial::CMD_VEL, pkt, buf);

        // 持锁 write：防止 load 与 write 之间读线程 close + fd 号复用（杂散写）
        std::lock_guard<std::mutex> lock(io_mutex_);
        const int fd = fd_.load();
        if (fd < 0)
        {
            ++tx_dropped_;
            return;
        }
        if (::write(fd, buf, n) != static_cast<ssize_t>(n))
            ++tx_errors_;
        else
            ++tx_frames_;
    }

    // ------------------------------------------------------------ 下行

    void handleFrame(const sentry_serial::Frame &f)
    {
        switch (f.cmd_id)
        {
        case sentry_serial::CMD_CHASSIS_FB:
        {
            sentry_serial::ChassisFbPacket p;
            if (!f.to(p))
            {
                rx_size_mismatch_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            sentry_msgs::msg::ChassisFeedback msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "odom";
            msg.yaw = p.yaw;
            msg.wz = p.wz;
            msg.vx_body = p.vx_body;
            msg.vy_body = p.vy_body;
            msg.yaw_initialized = (p.flags & sentry_serial::kChassisFbFlagYawInit) != 0;
            fb_pub_->publish(msg);
            break;
        }
        case sentry_serial::CMD_REFEREE:
        {
            sentry_serial::RefereePacket p;
            if (!f.to(p))
            {
                rx_size_mismatch_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            sentry_msgs::msg::RefereeState msg;
            msg.header.stamp = this->now();
            msg.game_progress = p.game_progress;
            msg.stage_remain_time = p.stage_remain_time;
            msg.current_hp = p.current_hp;
            msg.maximum_hp = p.maximum_hp;
            msg.ammo_17mm = p.ammo_17mm;
            msg.rune_available = (p.rune_flags & sentry_serial::kRuneFlagAvailable) != 0;
            msg.small_rune_active = (p.rune_flags & sentry_serial::kRuneFlagSmallActive) != 0;
            msg.big_rune_active = (p.rune_flags & sentry_serial::kRuneFlagBigActive) != 0;
            msg.enemy_outpost_alive = p.enemy_outpost_alive != 0;
            msg.enemy_outpost_hp = p.enemy_outpost_hp;
            msg.friend_outpost_hp = p.friend_outpost_hp;
            msg.base_hp = p.base_hp;
            ref_pub_->publish(msg);
            break;
        }
        default:
            rx_unknown_cmd_.fetch_add(1, std::memory_order_relaxed);
            break;
        }
    }

    // ------------------------------------------------------------ 串口 IO（读线程独占 open/close/read）

    void ioLoop()
    {
        uint8_t chunk[512];
        sentry_serial::Frame frame;

        while (running_)
        {
            if (fd_.load() < 0)
            {
                if (!openPort())
                {
                    // 打开失败：按重连周期重试（可中断的睡眠）
                    const auto deadline = std::chrono::steady_clock::now() +
                                          std::chrono::duration<double>(reconnect_period_);
                    while (running_ && std::chrono::steady_clock::now() < deadline)
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
            }

            struct pollfd pfd{};
            pfd.fd = fd_.load();
            pfd.events = POLLIN;
            const int pr = ::poll(&pfd, 1, 100);
            if (pr < 0)
            {
                if (errno == EINTR)
                    continue;
                reconnect("poll error");
                continue;
            }
            if (pr == 0)
                continue;
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
            {
                reconnect("port error/hangup");
                continue;
            }

            const ssize_t n = ::read(pfd.fd, chunk, sizeof(chunk));
            if (n < 0)
            {
                if (errno == EAGAIN || errno == EINTR)
                    continue;
                reconnect("read error");
                continue;
            }
            if (n == 0) // USB CDC 拔线常表现为 EOF
            {
                reconnect("EOF (device unplugged?)");
                continue;
            }
            for (ssize_t i = 0; i < n; ++i)
                if (parser_.feed(chunk[i], frame))
                    handleFrame(frame);

            // 统计快照到 atomic 镜像：logStats 在 executor 线程读，parser_ 本体只归读线程
            const auto &s = parser_.stats();
            rx_frames_.store(s.frames_ok, std::memory_order_relaxed);
            rx_crc_errors_.store(s.crc_errors, std::memory_order_relaxed);
            rx_len_errors_.store(s.len_errors, std::memory_order_relaxed);
        }
    }

    bool openPort()
    {
        const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "open %s failed: %s", port_.c_str(), std::strerror(errno));
            return false;
        }

        termios tio{};
        if (tcgetattr(fd, &tio) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
            ::close(fd);
            return false;
        }
        cfmakeraw(&tio);
        tio.c_cflag |= CLOCAL | CREAD;
        // cfmakeraw 置 CS8 但不清停止位/流控/软流控残留（端口可能被 minicom 等
        // 工具配过）：显式清掉才是真 8N1。CRTSCTS 残留且未接线时 write 会永久
        // EAGAIN 且不触发重连——上行静默断流
        tio.c_cflag &= ~(CSTOPB | CRTSCTS);
        tio.c_iflag &= ~(IXON | IXOFF | IXANY);
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        const speed_t spd = baudConstant(baud_);
        cfsetispeed(&tio, spd);
        cfsetospeed(&tio, spd);
        if (tcsetattr(fd, TCSANOW, &tio) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
            ::close(fd);
            return false;
        }
        tcflush(fd, TCIOFLUSH);

        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            fd_.store(fd);
        }
        reconnects_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_INFO(this->get_logger(), "serial opened: %s @ %d", port_.c_str(), baud_);
        return true;
    }

    void reconnect(const char *reason)
    {
        RCLCPP_WARN(this->get_logger(), "serial reconnecting: %s", reason);
        closePort();
    }

    void closePort()
    {
        std::lock_guard<std::mutex> lock(io_mutex_);
        const int fd = fd_.exchange(-1);
        if (fd >= 0)
            ::close(fd);
    }

    void logStats()
    {
        const uint64_t reopen = reconnects_.load(std::memory_order_relaxed);
        RCLCPP_INFO(this->get_logger(),
                    "tx %lu (drop %lu err %lu) | rx %lu (crc %lu len %lu size %lu unk %lu) | reopen %lu",
                    static_cast<unsigned long>(tx_frames_),
                    static_cast<unsigned long>(tx_dropped_),
                    static_cast<unsigned long>(tx_errors_),
                    static_cast<unsigned long>(rx_frames_.load(std::memory_order_relaxed)),
                    static_cast<unsigned long>(rx_crc_errors_.load(std::memory_order_relaxed)),
                    static_cast<unsigned long>(rx_len_errors_.load(std::memory_order_relaxed)),
                    static_cast<unsigned long>(rx_size_mismatch_.load(std::memory_order_relaxed)),
                    static_cast<unsigned long>(rx_unknown_cmd_.load(std::memory_order_relaxed)),
                    static_cast<unsigned long>(reopen > 0 ? reopen - 1 : 0));
    }

    // 参数
    std::string port_;
    int baud_ = 921600;
    double reconnect_period_ = 1.0;

    // LIO yaw 快照（odom 回调写、cmd 回调读；单线程 executor 下不并发）
    double lio_yaw_ = 0.0;
    uint8_t lio_yaw_seq_ = 0;
    bool lio_yaw_valid_ = false;

    // IO
    std::atomic<int> fd_{-1};
    std::atomic<bool> running_{false};
    std::mutex io_mutex_; // 保护 write 与 close/open 之间的 fd 生命周期
    std::thread io_thread_;
    sentry_serial::FrameParser parser_; // 只归读线程

    // 统计：tx_* 仅 executor 线程访问；其余跨线程，atomic(relaxed)
    uint64_t tx_frames_ = 0, tx_dropped_ = 0, tx_errors_ = 0;
    std::atomic<uint64_t> rx_frames_{0}, rx_crc_errors_{0}, rx_len_errors_{0};
    std::atomic<uint64_t> rx_size_mismatch_{0}, rx_unknown_cmd_{0}, reconnects_{0};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sentry_msgs::msg::ChassisFeedback>::SharedPtr fb_pub_;
    rclcpp::Publisher<sentry_msgs::msg::RefereeState>::SharedPtr ref_pub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SentrySerialNode>());
    rclcpp::shutdown();
    return 0;
}
