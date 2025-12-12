/*
 * ano_bridge_node.cpp
 * 匿名科创 V7 协议桥接节点 for ROS2 Humble
 * 功能：
 * 1. 订阅 /cmd_vel -> 发送 0x41 协议 (控制指令)
 * 2. 订阅 /odometry/filtered -> 发送 0x33 协议 (速度闭环)
 * 3. 读取串口 -> 解析 0x01, 0x04 (IMU) -> 发布 /imu/data
 * 4. 读取串口 -> 解析 0x51 (光流) -> 发布 /optical_flow/vel
 * * 参考文档：匿名通信协议V7.pdf
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Linux Serial Headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include <vector>
#include <chrono>
#include <cstring>
#include <cmath>

using namespace std::chrono_literals;
 // 协议常量定义 [cite: 1067]
const uint8_t FRAME_HEAD = 0xAA;
const uint8_t TARGET_ADDR = 0x05; // 飞控地址 [cite: 1414]
const uint8_t MY_ADDR = 0xAF;     // 上位机地址 [cite: 1414]

class AnoBridgeNode : public rclcpp::Node
{
public:
    AnoBridgeNode() : Node("ano_bridge_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 500000); // 凌霄飞控通常默认 500000
        this->declare_parameter<std::string>("imu_frame_id", "imu_link");
        this->declare_parameter<std::string>("flow_frame_id", "flow_link");

        // 初始化串口
        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();
        if (!init_serial(port, baud)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            rclcpp::shutdown();
            return;
        }

        // 创建发布者
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        flow_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/optical_flow/vel", 10);

        // 创建订阅者
        // 1. 订阅 Nav2 的控制指令 -> 转为 0x41 发送给飞控
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&AnoBridgeNode::cmd_vel_callback, this, std::placeholders::_1));

        // 2. 订阅 EKF 融合后的里程计 -> 转为 0x33 发送给飞控 (速度闭环)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&AnoBridgeNode::odom_callback, this, std::placeholders::_1));

        // 创建读取串口的定时器 (500Hz polling)
        read_timer_ = this->create_wall_timer(2ms, std::bind(&AnoBridgeNode::read_serial_data, this));

        RCLCPP_INFO(this->get_logger(), "ANO Bridge Node Started. Port: %s, Baud: %d", port.c_str(), baud);
    }

    ~AnoBridgeNode() {
        if (serial_fd_ != -1) close(serial_fd_);
    }

private:
    int serial_fd_ = -1;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr flow_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr read_timer_;

    // 接收缓冲区
    std::vector<uint8_t> rx_buffer_;
    enum ParseState { WAIT_HEAD, WAIT_ADDR, WAIT_ID, WAIT_LEN, WAIT_DATA, WAIT_SC, WAIT_AC };
    ParseState state_ = WAIT_HEAD;
    uint8_t p_addr_, p_id_, p_len_;
    std::vector<uint8_t> p_data_;
    uint8_t p_sc_, p_ac_;

    // 暂存 IMU 数据，因为协议中加速度(0x01)和四元数(0x04)是分开的
    sensor_msgs::msg::Imu imu_msg_;
    bool has_acc_ = false;

    // --- 串口初始化 ---
    bool init_serial(const std::string& port, int baud_rate) {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) return false;

        struct termios options;
        tcgetattr(serial_fd_, &options);
        
        // 设置波特率
        speed_t baud;
        switch(baud_rate) {
            case 115200: baud = B115200; break;
            case 460800: baud = B460800; break;
            case 500000: baud = B500000; break;
            case 921600: baud = B921600; break;
            default: baud = B500000; break; // Default fallback
        }
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
        options.c_cflag &= ~PARENB;          // No parity
        options.c_cflag &= ~CSTOPB;          // 1 stop bit
        options.c_cflag &= ~CSIZE;           // Mask character size bits
        options.c_cflag |= CS8;              // 8 data bits
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
        options.c_oflag &= ~OPOST;           // Raw output

        tcsetattr(serial_fd_, TCSANOW, &options);
        return true;
    }

    // --- 数据发送辅助函数 ---
    void send_frame(uint8_t id, const std::vector<uint8_t>& data) {
        if (serial_fd_ == -1) return;

        std::vector<uint8_t> frame;
        frame.push_back(FRAME_HEAD);    // Head [cite: 1067]
        frame.push_back(TARGET_ADDR);   // Addr [cite: 1067]
        frame.push_back(id);            // ID   [cite: 1067]
        frame.push_back(data.size());   // Len  [cite: 1067]

        uint8_t sc = 0, ac = 0;
        
        // 计算 Head 到 Len 的校验
        for (size_t i = 0; i < frame.size(); i++) {
            sc += frame[i];
            ac += sc;
        }

        // 添加数据并计算校验
        for (uint8_t byte : data) {
            frame.push_back(byte);
            sc += byte;
            ac += sc;
        }

        frame.push_back(sc); // SC [cite: 1070]
        frame.push_back(ac); // AC [cite: 1072]

        write(serial_fd_, frame.data(), frame.size());
    }

    // --- ROS 回调函数 ---

 // 1. 处理 Nav2 的控制指令 -> 发送 0x41 [cite: 1284]
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
     // Nav2 输出 m/s, 协议要求 cm/s [cite: 1291]
        int16_t speed_x = static_cast<int16_t>(msg->linear.x * 100.0); // 机头方向
        int16_t speed_y = static_cast<int16_t>(msg->linear.y * 100.0); // 机身左侧
        int16_t speed_z = 0; // 高度通常由飞控定高，或者可以在这里处理 msg->linear.z

     // 角速度 rad/s -> deg/s (飞控单位) [cite: 1290]
        int16_t yaw_dps = static_cast<int16_t>(msg->angular.z * 57.29578);

     // 构造 0x41 数据体 (14 bytes) [cite: 1285]
        std::vector<uint8_t> data(14, 0);
        
        // 0x41 帧定义: ROL, PIT, THR, YAW_DPS, SPD_X, SPD_Y, SPD_Z (均为 int16)
     // 在速度控制模式下，我们只控制 SPD_X, SPD_Y, YAW_DPS [cite: 1292]
        // ROL, PIT, THR 填 0
        
        // Little Endian 填充
        // Byte 0-5: ROL, PIT, THR (0)
        
        // Byte 6-7: YAW DPS
        std::memcpy(&data[6], &yaw_dps, 2);

        // Byte 8-9: SPD X
        std::memcpy(&data[8], &speed_x, 2);

        // Byte 10-11: SPD Y
        std::memcpy(&data[10], &speed_y, 2);

        // Byte 12-13: SPD Z
        std::memcpy(&data[12], &speed_z, 2);

        send_frame(0x41, data);
    }

 // 2. 处理 EKF 融合数据 -> 发送 0x33 [cite: 1266]
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // EKF 的 twist 是基于 child_frame_id (base_link) 的，即相对于机身的速度
     // 协议要求 cm/s [cite: 1269]
        
        int16_t odom_vx = static_cast<int16_t>(msg->twist.twist.linear.x * 100.0);
        int16_t odom_vy = static_cast<int16_t>(msg->twist.twist.linear.y * 100.0);
        int16_t odom_vz = static_cast<int16_t>(msg->twist.twist.linear.z * 100.0);

     // 构造 0x33 数据体 (6 bytes) [cite: 1268]
        // SPEED X, SPEED Y, SPEED Z (int16)
        std::vector<uint8_t> data(6);
        std::memcpy(&data[0], &odom_vx, 2);
        std::memcpy(&data[2], &odom_vy, 2);
        std::memcpy(&data[4], &odom_vz, 2);

        send_frame(0x33, data);
    }

    // --- 串口读取与协议解析 ---
    void read_serial_data() {
        if (serial_fd_ == -1) return;
        
        uint8_t buf[64];
        int n = read(serial_fd_, buf, sizeof(buf));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                process_byte(buf[i]);
            }
        }
    }

    void process_byte(uint8_t byte) {
        switch (state_) {
            case WAIT_HEAD:
                if (byte == FRAME_HEAD) state_ = WAIT_ADDR; // [cite: 1067]
                break;
            case WAIT_ADDR:
                p_addr_ = byte;
                state_ = WAIT_ID;
                break;
            case WAIT_ID:
                p_id_ = byte;
                state_ = WAIT_LEN;
                break;
            case WAIT_LEN:
                p_len_ = byte;
                p_data_.clear();
                if (p_len_ == 0) state_ = WAIT_SC;
                else state_ = WAIT_DATA;
                break;
            case WAIT_DATA:
                p_data_.push_back(byte);
                if (p_data_.size() == p_len_) state_ = WAIT_SC;
                break;
            case WAIT_SC:
                p_sc_ = byte;
                state_ = WAIT_AC;
                break;
            case WAIT_AC:
                p_ac_ = byte;
                if (check_sum()) {
                    handle_packet();
                }
                state_ = WAIT_HEAD;
                break;
        }
    }

 // 校验和计算 [cite: 1070]
    bool check_sum() {
        uint8_t sc = 0, ac = 0;
        std::vector<uint8_t> raw;
        raw.push_back(FRAME_HEAD);
        raw.push_back(p_addr_);
        raw.push_back(p_id_);
        raw.push_back(p_len_);
        raw.insert(raw.end(), p_data_.begin(), p_data_.end());

        for (uint8_t b : raw) {
            sc += b;
            ac += sc;
        }
        return (sc == p_sc_ && ac == p_ac_);
    }

    // 处理有效数据包
    void handle_packet() {
     // 1. 处理 IMU 加速度/角速度 (0x01) [cite: 1120]
        if (p_id_ == 0x01 && p_len_ == 13) {
            int16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
            std::memcpy(&acc_x, &p_data_[0], 2);
            std::memcpy(&acc_y, &p_data_[2], 2);
            std::memcpy(&acc_z, &p_data_[4], 2);
            std::memcpy(&gyr_x, &p_data_[6], 2);
            std::memcpy(&gyr_y, &p_data_[8], 2);
            std::memcpy(&gyr_z, &p_data_[10], 2);

            // 注意：此处需要根据你的飞控具体量程进行换算
            // 假设原始数据直接输出（或者根据 MP6050 标准单位）
            // 这里为了演示，暂时只进行坐标系转换，实际需乘以比例系数
            imu_msg_.linear_acceleration.x = acc_x; 
            imu_msg_.linear_acceleration.y = acc_y;
            imu_msg_.linear_acceleration.z = acc_z;
            imu_msg_.angular_velocity.x = gyr_x;
            imu_msg_.angular_velocity.y = gyr_y;
            imu_msg_.angular_velocity.z = gyr_z;
            has_acc_ = true;
        }

     // 2. 处理 IMU 姿态四元数 (0x04) [cite: 1140]
        else if (p_id_ == 0x04 && p_len_ == 9) {
            int16_t v0, v1, v2, v3;
            std::memcpy(&v0, &p_data_[0], 2);
            std::memcpy(&v1, &p_data_[2], 2);
            std::memcpy(&v2, &p_data_[4], 2);
            std::memcpy(&v3, &p_data_[6], 2);

         // 协议：Data * 10000 [cite: 1143]
            imu_msg_.orientation.w = v0 / 10000.0;
            imu_msg_.orientation.x = v1 / 10000.0;
            imu_msg_.orientation.y = v2 / 10000.0;
            imu_msg_.orientation.z = v3 / 10000.0;

            // 发布完整的 IMU 消息
            if (has_acc_) {
                imu_msg_.header.stamp = this->now();
                imu_msg_.header.frame_id = this->get_parameter("imu_frame_id").as_string();
                imu_pub_->publish(imu_msg_);
                has_acc_ = false; // Reset
            }
        }

     // 3. 处理光流数据 (0x51) [cite: 1296]
        else if (p_id_ == 0x51) {
            uint8_t mode = p_data_[0];
            
         // 我们只用 Mode 1: 融合后的地面速度 [cite: 1303]
            if (mode == 1 && p_len_ >= 5) {
                int16_t dx, dy;
                std::memcpy(&dx, &p_data_[2], 2); // cm/s
                std::memcpy(&dy, &p_data_[4], 2); // cm/s

                geometry_msgs::msg::TwistWithCovarianceStamped flow_msg;
                flow_msg.header.stamp = this->now();
                flow_msg.header.frame_id = this->get_parameter("flow_frame_id").as_string();
                
                // 转换为 m/s
                flow_msg.twist.twist.linear.x = dx / 100.0;
                flow_msg.twist.twist.linear.y = dy / 100.0;
                
                // 设置简单的协方差，表示这是观测值
                flow_msg.twist.covariance[0] = 0.01; // x
                flow_msg.twist.covariance[7] = 0.01; // y
                
                flow_pub_->publish(flow_msg);
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnoBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}