#include "ano_drone_bringup/ano_hardware_interface_node.hpp"
#include <chrono>
#include <iostream>

namespace ano_drone_bringup
{

AnoHardwareInterfaceNode::AnoHardwareInterfaceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ano_hardware_interface", options)
{
  RCLCPP_INFO(this->get_logger(), "初始化匿名V7协议硬件接口节点...");
  
  // 初始化参数
  initParameters();
  
  // 初始化串口
  if (!initSerialPort()) {
    RCLCPP_ERROR(this->get_logger(), "串口初始化失败，节点将继续运行但无法与STM32通信");
  }
  
  // 初始化ROS2话题发布器和订阅器
  initPublishersAndSubscribers();
  
  // 启动串口读取线程
  startSerialReadThread();
  
  RCLCPP_INFO(this->get_logger(), "匿名V7协议硬件接口节点初始化完成");
  RCLCPP_INFO(this->get_logger(), "串口: %s, 波特率: %d", serial_port_.c_str(), baud_rate_);
  RCLCPP_INFO(this->get_logger(), "IMU话题: %s", imu_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "位姿话题: %s", pose_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "速度控制话题: %s", velocity_topic_.c_str());
}

AnoHardwareInterfaceNode::~AnoHardwareInterfaceNode()
{
  // 停止串口读取线程
  thread_running_ = false;
  if (serial_read_thread_.joinable()) {
    serial_read_thread_.join();
  }
  
  // 关闭串口
  if (serial_port_ && serial_port_->isOpen()) {
    serial_port_->close();
  }
  
  RCLCPP_INFO(this->get_logger(), "节点销毁，接收帧数: %lu, 发送帧数: %lu, 校验和错误: %lu",
              rx_frame_count_.load(), tx_frame_count_.load(), checksum_error_count_.load());
}

void AnoHardwareInterfaceNode::initParameters()
{
  // 声明参数
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 500000);
  this->declare_parameter<bool>("use_custom_msg", true);
  this->declare_parameter<std::string>("imu_topic", "imu/data");
  this->declare_parameter<std::string>("pose_topic", "ekf/pose");
  this->declare_parameter<std::string>("velocity_topic", "cmd_vel");
  
  // 获取参数值
  this->get_parameter("serial_port", serial_port_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("use_custom_msg", use_custom_msg_);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("pose_topic", pose_topic_);
  this->get_parameter("velocity_topic", velocity_topic_);
}

bool AnoHardwareInterfaceNode::initSerialPort()
{
  try {
    serial_port_ = std::make_unique<serial::Serial>();
    serial_port_->setPort(serial_port_);
    serial_port_->setBaudrate(baud_rate_);
    serial_port_->setBytesize(serial::eightbits);
    serial_port_->setParity(serial::parity_none);
    serial_port_->setStopbits(serial::stopbits_one);
    serial_port_->setFlowcontrol(serial::flowcontrol_none);
    
    // 设置超时
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    serial_port_->setTimeout(timeout);
    
    serial_port_->open();
    
    if (serial_port_->isOpen()) {
      serial_connected_ = true;
      RCLCPP_INFO(this->get_logger(), "串口 %s 打开成功，波特率: %d", serial_port_.c_str(), baud_rate_);
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "串口 %s 打开失败", serial_port_.c_str());
      return false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "串口初始化异常: %s", e.what());
    return false;
  }
}

void AnoHardwareInterfaceNode::initPublishersAndSubscribers()
{
  // 创建发布器
  if (use_custom_msg_) {
    imu_data_pub_ = this->create_publisher<ano_drone_bringup::msg::AnoImuData>(
      imu_topic_, rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "创建自定义IMU数据发布器: %s", imu_topic_.c_str());
  } else {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "创建标准IMU发布器: %s", imu_topic_.c_str());
  }
  
  // 创建订阅器
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AnoHardwareInterfaceNode::poseCallback, this, std::placeholders::_1));
  
  velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    velocity_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AnoHardwareInterfaceNode::velocityCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "订阅位姿话题: %s", pose_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "订阅速度控制话题: %s", velocity_topic_.c_str());
}

void AnoHardwareInterfaceNode::startSerialReadThread()
{
  thread_running_ = true;
  serial_read_thread_ = std::thread(&AnoHardwareInterfaceNode::serialReadThreadFunc, this);
  RCLCPP_INFO(this->get_logger(), "串口读取线程已启动");
}

void AnoHardwareInterfaceNode::serialReadThreadFunc()
{
  rx_buffer_.resize(RX_BUFFER_SIZE);
  
  while (thread_running_ && rclcpp::ok()) {
    if (!serial_connected_ || !serial_port_ || !serial_port_->isOpen()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    
    try {
      // 读取串口数据
      size_t bytes_read = serial_port_->read(rx_buffer_.data(), rx_buffer_.size());
      
      if (bytes_read > 0) {
        // 解析接收到的数据
        parseSerialData(rx_buffer_.data(), bytes_read);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "串口读取异常: %s", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "串口读取线程退出");
}

void AnoHardwareInterfaceNode::parseSerialData(const uint8_t* data, size_t length)
{
  for (size_t i = 0; i < length; ++i) {
    uint8_t byte = data[i];
    
    switch (parse_state_) {
      case ParseState::WAITING_FOR_HEADER:
        if (byte == 0xAA) {
          parse_state_ = ParseState::READING_FRAME_ID;
          current_data_.clear();
        }
        break;
        
      case ParseState::READING_FRAME_ID:
        if (byte == 0xFF) {  // 目标地址
          parse_state_ = ParseState::READING_DATA_LEN;
        } else {
          // 不是有效的目标地址，重新等待帧头
          parse_state_ = ParseState::WAITING_FOR_HEADER;
        }
        break;
        
      case ParseState::READING_DATA_LEN:
        current_frame_id_ = byte;
        parse_state_ = ParseState::READING_DATA_LEN;
        // 下一个字节是数据长度
        expected_bytes_ = 1;
        parse_state_ = ParseState::READING_DATA_LEN;
        break;
        
      case ParseState::READING_DATA_LEN:
        current_data_len_ = byte;
        if (current_data_len_ > 0) {
          current_data_.reserve(current_data_len_);
          parse_state_ = ParseState::READING_DATA;
          expected_bytes_ = current_data_len_;
        } else {
          // 数据长度为0，直接读取校验和
          parse_state_ = ParseState::READING_CHECKSUM_SUM;
          expected_bytes_ = 2;  // SUM和ADD
        }
        break;
        
      case ParseState::READING_DATA:
        current_data_.push_back(byte);
        if (current_data_.size() >= current_data_len_) {
          parse_state_ = ParseState::READING_CHECKSUM_SUM;
          expected_bytes_ = 2;  // SUM和ADD
        }
        break;
        
      case ParseState::READING_CHECKSUM_SUM:
        // 这里简化处理，直接读取两个校验和字节
        if (current_data_len_ > 0) {
          uint8_t sum = byte;
          // 需要读取下一个字节ADD
          if (i + 1 < length) {
            uint8_t add = data[i + 1];
            i++;  // 消耗ADD字节
            
            // 验证校验和
            if (verify_v7_checksum(current_data_.data(), current_data_len_, sum, add)) {
              // 处理完整的帧
              processV7Frame(current_frame_id_, current_data_.data(), current_data_len_);
              rx_frame_count_++;
            } else {
              checksum_error_count_++;
              RCLCPP_WARN(this->get_logger(), "帧ID 0x%02X 校验和错误", current_frame_id_);
            }
          }
        }
        // 重置状态机
        parse_state_ = ParseState::WAITING_FOR_HEADER;
        break;
        
      default:
        parse_state_ = ParseState::WAITING_FOR_HEADER;
        break;
    }
  }
}

void AnoHardwareInterfaceNode::processV7Frame(uint8_t frame_id, const uint8_t* data, uint8_t data_len)
{
  switch (frame_id) {
    case 0x01:  // IMU/光流数据帧
      if (data_len == sizeof(AnoImuFlowData)) {
        processImuFlowFrame(data);
      } else {
        RCLCPP_WARN(this->get_logger(), "0x01帧数据长度不匹配: 期望 %zu, 实际 %u",
                   sizeof(AnoImuFlowData), data_len);
      }
      break;
      
    default:
      RCLCPP_DEBUG(this->get_logger(), "收到未知帧ID: 0x%02X, 长度: %u", frame_id, data_len);
      break;
  }
}

void AnoHardwareInterfaceNode::processImuFlowFrame(const uint8_t* data)
{
  const AnoImuFlowData* imu_data = reinterpret_cast<const AnoImuFlowData*>(data);
  
  // 发布IMU数据
  publishImuData(*imu_data);
  
  // 调试信息
  static size_t count = 0;
  if (++count % 100 == 0) {
    RCLCPP_DEBUG(this->get_logger(), 
                "收到IMU数据: accel(%.2f, %.2f, %.2f), gyro(%.2f, %.2f, %.2f), 姿态(%.2f, %.2f, %.2f)",
                imu_data->accel_x, imu_data->accel_y, imu_data->accel_z,
                imu_data->gyro_x, imu_data->gyro_y, imu_data->gyro_z,
                imu_data->roll, imu_data->pitch, imu_data->yaw);
  }
}

void AnoHardwareInterfaceNode::publishImuData(const AnoImuFlowData& imu_data)
{
  auto now = this->now();
  
  if (use_custom_msg_ && imu_data_pub_) {
    // 发布自定义消息
    auto msg = std::make_unique<ano_drone_bringup::msg::AnoImuData>();
    msg->stamp = now;
    
    // 填充IMU数据
    msg->accel_x = imu_data.accel_x;
    msg->accel_y = imu_data.accel_y;
    msg->accel_z = imu_data.accel_z;
    msg->gyro_x = imu_data.gyro_x;
    msg->gyro_y = imu_data.gyro_y;
    msg->gyro_z = imu_data.gyro_z;
    msg->mag_x = imu_data.mag_x;
    msg->mag_y = imu_data.mag_y;
    msg->mag_z = imu_data.mag_z;
    
    // 姿态角
    msg->roll = imu_data.roll;
    msg->pitch = imu_data.pitch;
    msg->yaw = imu_data.yaw;
    
    // 四元数
    msg->qw = imu_data.qw;
    msg->qx = imu_data.qx;
    msg->qy = imu_data.qy;
    msg->qz = imu_data.qz;
    
    // 光流数据
    msg->flow_x = imu_data.flow_x;
    msg->flow_y = imu_data.flow_y;
    msg->flow_quality = imu_data.flow_quality;
    
    // 高度数据
    msg->baro_altitude = imu_data.baro_altitude;
    msg->ultrasonic_alt = imu_data.ultrasonic_alt;
    
    // 系统状态
    msg->imu_health = imu_data.imu_health;
    msg->flow_health = imu_data.flow_health;
    
    imu_data_pub_->publish(std::move(msg));
  } else if (imu_pub_) {
    // 发布标准IMU消息
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    msg->header.stamp = now;
    msg->header.frame_id = "imu_link";
    
    // 线性加速度 (m/s²)
    msg->linear_acceleration.x = imu_data.accel_x;
    msg->linear_acceleration.y = imu_data.accel_y;
    msg->linear_acceleration.z = imu_data.accel_z;
    
    // 角速度 (rad/s)
    msg->angular_velocity.x = imu_data.gyro_x;
    msg->angular_velocity.y = imu_data.gyro_y;
    msg->angular_velocity.z = imu_data.gyro_z;
    
    // 四元数姿态
    msg->orientation.w = imu_data.qw;
    msg->orientation.x = imu_data.qx;
    msg->orientation.y = imu_data.qy;
    msg->orientation.z = imu_data.qz;
    
    imu_pub_->publish(std::move(msg));
  }
}

void AnoHardwareInterfaceNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 发送EKF位姿数据到STM32
  sendEKFData(*msg);
  
  // 调试信息
  static size_t count = 0;
  if (++count % 10 == 0) {
    RCLCPP_DEBUG(this->get_logger(), 
                "收到位姿: 位置(%.2f, %.2f, %.2f), 姿态(%.2f, %.2f, %.2f, %.2f)",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                msg->pose.orientation.w, msg->pose.orientation.x,
                msg->pose.orientation.y, msg->pose.orientation.z);
  }
}

void AnoHardwareInterfaceNode::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 发送速度控制指令到STM32
  sendVelocityCmd(*msg);
  
  // 调试信息
  static size_t count = 0;
  if (++count % 10 == 0) {
    RCLCPP_DEBUG(this->get_logger(), 
                "收到速度指令: 线速度(%.2f, %.2f, %.2f), 角速度(%.2f, %.2f, %.2f)",
                msg->linear.x, msg->linear.y, msg->linear.z,
                msg->angular.x, msg->angular.y, msg->angular.z);
  }
}

void AnoHardwareInterfaceNode::sendEKFData(const geometry_msgs::msg::PoseStamped& pose)
{
  AnoEKFData ekf_data;
  
  // 填充位置数据
  ekf_data.position_x = static_cast<float>(pose.pose.position.x);
  ekf_data.position_y = static_cast<float>(pose.pose.position.y);
  ekf_data.position_z = static_cast<float>(pose.pose.position.z);
  
  // 速度数据（这里简化处理，实际应从EKF获取）
  ekf_data.velocity_x = 0.0f;
  ekf_data.velocity_y = 0.0f;
  ekf_data.velocity_z = 0.0f;
  
  // 四元数数据
  ekf_data.quaternion_w = static_cast<float>(pose.pose.orientation.w);
  ekf_data.quaternion