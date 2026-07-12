/**
 * chassis_cmd_node: 仿真侧"底盘固件"——世界系速度指令 → 机体系 /cmd_vel
 *
 * 架构原则：世界系→机体系的旋转必须在航向最新鲜的地方执行。
 *   - 导航栈 (50Hz) 输出 odom 系速度到 /cmd_vel_world，不做机体系旋转
 *   - 本节点在每个底盘 IMU 样本上（仿真 ~250Hz+）用陀螺积分 yaw 旋转后发 /cmd_vel
 *   - LIO yaw 仅做零漂校准，|wz| 门控（自转时 LIO yaw 有 50-70ms 龄期滞后，不可信）
 *
 * 真车部署：本节点整体由电控 MCU 固件替代（1kHz 陀螺积分 + 同一套门控校准），
 * 上位机只发世界系速度。仿真与真车保持同一架构、同一接口。
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>

#include <sentry_local_planner/chassis_yaw_estimator.hpp>

class ChassisCmdNode : public rclcpp::Node
{
public:
    ChassisCmdNode() : Node("chassis_cmd")
    {
        this->declare_parameter<std::string>("cmd_world_topic", "/cmd_vel_world");
        this->declare_parameter<std::string>("cmd_body_topic", "/cmd_vel");
        this->declare_parameter<std::string>("imu_topic", "/chassis/imu");
        this->declare_parameter<std::string>("odom_topic", "/Odometry");
        this->declare_parameter<double>("gate_wz", 0.5);
        this->declare_parameter<double>("correction_gain", 1.0);
        this->declare_parameter<double>("cmd_timeout", 0.2);

        sentry_planner::ChassisYawEstimator::Config ecfg;
        ecfg.gate_wz = this->get_parameter("gate_wz").as_double();
        ecfg.correction_gain = this->get_parameter("correction_gain").as_double();
        est_ = sentry_planner::ChassisYawEstimator(ecfg);
        cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            this->get_parameter("cmd_body_topic").as_string(), 10);

        cmd_world_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            this->get_parameter("cmd_world_topic").as_string(), 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmd_world_ = *msg;
                last_cmd_time_ = this->now();
                has_cmd_ = true;
            });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").as_string(), 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                const auto &q = msg->pose.pose.orientation;
                double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                est_.onLioYaw(yaw);
            });

        // 每个 IMU 样本: 积分 yaw + 旋转最新世界系指令后转发（旋转执行点在最高频处）
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic").as_string(),
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                est_.onGyro(stamp, msg->angular_velocity.z);
                publishBodyCmd();
            });

        RCLCPP_INFO(this->get_logger(),
                    "Chassis cmd node: %s (world) -> %s (body), gate_wz=%.2f",
                    this->get_parameter("cmd_world_topic").as_string().c_str(),
                    this->get_parameter("cmd_body_topic").as_string().c_str(),
                    ecfg.gate_wz);
    }

private:
    void publishBodyCmd()
    {
        if (!est_.initialized() || !has_cmd_)
            return;

        geometry_msgs::msg::Twist cmd;
        // 指令超时看门狗：上游停发（节点挂掉/网络断）时输出零而不是保持旧指令
        if ((this->now() - last_cmd_time_).seconds() < cmd_timeout_)
        {
            double yaw = est_.yaw();
            double c = std::cos(-yaw), s = std::sin(-yaw);
            cmd.linear.x = c * cmd_world_.linear.x - s * cmd_world_.linear.y;
            cmd.linear.y = s * cmd_world_.linear.x + c * cmd_world_.linear.y;
            cmd.angular.z = cmd_world_.angular.z;
        }
        cmd_pub_->publish(cmd);
    }

    sentry_planner::ChassisYawEstimator est_;
    geometry_msgs::msg::Twist cmd_world_;
    rclcpp::Time last_cmd_time_;
    double cmd_timeout_ = 0.2;
    bool has_cmd_ = false;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_world_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisCmdNode>());
    rclcpp::shutdown();
    return 0;
}
