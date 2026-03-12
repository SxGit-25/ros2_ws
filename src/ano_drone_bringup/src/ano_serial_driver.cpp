#include "ano_drone_bringup/ano_serial_driver.hpp"

#include <chrono>
#include <string>
#include <thread>

namespace ano_drone_bringup
{

AnoSerialDriverNode::AnoSerialDriverNode(const rclcpp::NodeOptions & options)
: Node("ano_serial_driver", options),
  serial_port_("/dev/ttyAMA0"),
  baud_rate_(115200),
  imu_topic_("imu/data"),
  pose_topic_("command/pose"),
  twist_topic_("command/twist"),
  rx_buffer_(V7_FRAME_MAX_LENGTH),
  rx_buffer_index_(0),
  rx_state_(RxState::WAITING_FOR_HEADER)
{
  // 声明并读取串口参数
  this->declare_parameter<std::string>("serial_port", serial_port_);
  this->declare_parameter<int>("baud_rate", baud_rate_);
  
  // 声明并读取ROS话题参数
  this->declare_parameter<std::string>("imu_topic", imu_topic_);
  this->declare_parameter<std::string>("pose_topic", pose_topic_);
  this->declare_parameter<std::string>("twist_topic", twist_topic_);

  this->get_parameter("serial_port", serial_port_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("pose_topic", pose_topic_);
  this->get_parameter("twist_topic", twist_topic_);

  RCLCPP_INFO(this->get_logger(), "Serial configuration: port=%s, baud=%d",
              serial_port_.c_str(), baud_rate_);
  RCLCPP_INFO(this->get_logger(), "Topic configuration: imu=%s, pose=%s, twist=%s",
              imu_topic_.c_str(), pose_topic_.c_str(), twist_topic_.c_str());

  try
  {
    // 初始化串口对象
    serial_port_ = std::make_unique<serial::Serial>(
      serial_port_,
      static_cast<uint32_t>(baud_rate_),
      serial::Timeout::simpleTimeout(1000)
    );

    if (serial_port_->isOpen())
    {
      RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Serial port initialization failed: %s", e.what());
  }

  // 初始化ROS 2 Publisher和Subscriber
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
    imu_topic_, 10);
  
  pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, 10,
    std::bind(&AnoSerialDriverNode::poseCallback, this, std::placeholders::_1));
  
  twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    twist_topic_, 10,
    std::bind(&AnoSerialDriverNode::twistCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "ROS 2 publishers and subscribers initialized");

  // 启动串口读取线程
  serial_read_thread_ = std::thread(&AnoSerialDriverNode::serialReadLoop, this);
}

AnoSerialDriverNode::~AnoSerialDriverNode()
{
  // 等待线程结束
  if (serial_read_thread_.joinable())
  {
    serial_read_thread_.join();
  }

  // 关闭串口
  if (serial_port_ && serial_port_->isOpen())
  {
    serial_port_->close();
    RCLCPP_INFO(this->get_logger(), "Serial port closed");
  }
}

void AnoSerialDriverNode::serialReadLoop()
{
  RCLCPP_INFO(this->get_logger(), "Serial read thread started");

  // 局部变量用于状态机
  uint8_t expected_data_length = 0;
  uint8_t received_sum = 0;
  uint8_t received_add = 0;
  uint8_t calculated_sum = 0;
  uint8_t calculated_add = 0;

  while (rclcpp::ok())
  {
    if (!serial_port_ || !serial_port_->isOpen())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    // 尝试读取串口数据
    size_t bytes_available = serial_port_->available();
    if (bytes_available == 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    // 读取数据
    std::vector<uint8_t> read_buffer(bytes_available);
    size_t bytes_read = serial_port_->read(read_buffer.data(), bytes_available);
    
    if (bytes_read == 0)
    {
      continue;
    }

    // 处理每个接收到的字节
    for (size_t i = 0; i < bytes_read; ++i)
    {
      uint8_t byte = read_buffer[i];

      switch (rx_state_)
      {
        case RxState::WAITING_FOR_HEADER:
          if (byte == V7_FRAME_HEADER)
          {
            // 找到帧头，重置缓冲区
            rx_buffer_index_ = 0;
            rx_buffer_[rx_buffer_index_++] = byte;
            rx_state_ = RxState::WAITING_FOR_ADDRESS;
          }
          break;

        case RxState::WAITING_FOR_ADDRESS:
          rx_buffer_[rx_buffer_index_++] = byte;
          if (byte == V7_TARGET_ADDRESS)
          {
            rx_state_ = RxState::WAITING_FOR_ID;
          }
          else
          {
            // 地址不匹配，重新寻找帧头
            rx_state_ = RxState::WAITING_FOR_HEADER;
          }
          break;

        case RxState::WAITING_FOR_ID:
          rx_buffer_[rx_buffer_index_++] = byte;
          rx_state_ = RxState::WAITING_FOR_LENGTH;
          break;

        case RxState::WAITING_FOR_LENGTH:
          rx_buffer_[rx_buffer_index_++] = byte;
          expected_data_length = byte;
          
          if (expected_data_length == 0)
          {
            // 没有数据，直接等待校验和
            rx_state_ = RxState::WAITING_FOR_CHECKSUM1;
          }
          else if (expected_data_length <= V7_FRAME_MAX_LENGTH - V7_FRAME_MIN_LENGTH)
          {
            rx_state_ = RxState::WAITING_FOR_DATA;
          }
          else
          {
            // 数据长度过长，重新寻找帧头
            rx_state_ = RxState::WAITING_FOR_HEADER;
          }
          break;

        case RxState::WAITING_FOR_DATA:
          rx_buffer_[rx_buffer_index_++] = byte;
          if (rx_buffer_index_ >= (4 + expected_data_length))  // 帧头+地址+ID+长度+数据
          {
            rx_state_ = RxState::WAITING_FOR_CHECKSUM1;
          }
          break;

        case RxState::WAITING_FOR_CHECKSUM1:
          received_sum = byte;
          rx_state_ = RxState::WAITING_FOR_CHECKSUM2;
          break;

        case RxState::WAITING_FOR_CHECKSUM2:
          received_add = byte;
          
          // 计算校验和
          calculateV7Checksum(rx_buffer_.data(), 4 + expected_data_length, calculated_sum, calculated_add);
          
          // 验证校验和
          if (received_sum == calculated_sum && received_add == calculated_add)
          {
            // 校验通过，处理数据帧
            uint8_t frame_id = rx_buffer_[2];  // ID在缓冲区索引2的位置
            
            if (frame_id == V7_IMU_DATA_ID && expected_data_length == sizeof(ano_imu_data_t))
            {
              // 解析IMU数据
              const ano_imu_data_t* imu_data = reinterpret_cast<const ano_imu_data_t*>(&rx_buffer_[4]);
              
              // 打印原始数据
              RCLCPP_DEBUG(this->get_logger(),
                         "Received IMU data - acc_x: %d, gyr_x: %d",
                         imu_data->acc_x, imu_data->gyr_x);
              
              // 创建并发布ROS 2 Imu消息
              auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
              
              // 设置时间戳
              imu_msg->header.stamp = this->now();
              imu_msg->header.frame_id = "imu_link";
              
              // 单位转换：加速度 (m/s²) = val * 0.001 * 9.80665
              const double acc_scale = 0.001 * 9.80665;
              imu_msg->linear_acceleration.x = static_cast<double>(imu_data->acc_x) * acc_scale;
              imu_msg->linear_acceleration.y = static_cast<double>(imu_data->acc_y) * acc_scale;
              imu_msg->linear_acceleration.z = static_cast<double>(imu_data->acc_z) * acc_scale;
              
              // 单位转换：角速度 (rad/s) = val * 0.01 * (M_PI / 180.0)
              const double gyro_scale = 0.01 * (M_PI / 180.0);
              imu_msg->angular_velocity.x = static_cast<double>(imu_data->gyr_x) * gyro_scale;
              imu_msg->angular_velocity.y = static_cast<double>(imu_data->gyr_y) * gyro_scale;
              imu_msg->angular_velocity.z = static_cast<double>(imu_data->gyr_z) * gyro_scale;
              
              // 协方差矩阵（假设数据质量良好）
              imu_msg->linear_acceleration_covariance[0] = 0.01;
              imu_msg->linear_acceleration_covariance[4] = 0.01;
              imu_msg->linear_acceleration_covariance[8] = 0.01;
              
              imu_msg->angular_velocity_covariance[0] = 0.001;
              imu_msg->angular_velocity_covariance[4] = 0.001;
              imu_msg->angular_velocity_covariance[8] = 0.001;
              
              // 发布消息
              imu_publisher_->publish(std::move(imu_msg));
              
              RCLCPP_DEBUG(this->get_logger(), "Published IMU message");
            }
            else if (frame_id == V7_EKF_POSE_ID && expected_data_length == sizeof(ano_ekf_pose_t))
            {
              // 可以在这里添加EKF数据解析
              RCLCPP_INFO(this->get_logger(), "Received EKF pose data (ID: 0x%02X)", frame_id);
            }
            else if (frame_id == V7_VELOCITY_CMD_ID && expected_data_length == sizeof(ano_velocity_cmd_t))
            {
              // 可以在这里添加速度命令解析
              RCLCPP_INFO(this->get_logger(), "Received velocity command (ID: 0x%02X)", frame_id);
            }
            else
            {
              RCLCPP_WARN(this->get_logger(),
                         "Unknown frame ID: 0x%02X with length: %d",
                         frame_id, expected_data_length);
            }
          }
          else
          {
            RCLCPP_WARN(this->get_logger(),
                       "Checksum mismatch! Received: SUM=0x%02X, ADD=0x%02X, Calculated: SUM=0x%02X, ADD=0x%02X",
                       received_sum, received_add, calculated_sum, calculated_add);
          }
          
          // 无论校验是否通过，都重新开始寻找下一帧
          rx_state_ = RxState::WAITING_FOR_HEADER;
          break;
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Serial read thread exiting");
}

void AnoSerialDriverNode::sendV7Frame(uint8_t id, const uint8_t* data, uint8_t len)
{
  if (!serial_port_ || !serial_port_->isOpen())
  {
    RCLCPP_WARN(this->get_logger(), "Serial port not open, cannot send frame");
    return;
  }

  if (len > 255)
  {
    RCLCPP_ERROR(this->get_logger(), "Data length too large: %d", len);
    return;
  }

  // 构建帧缓冲区：帧头 + 地址 + ID + 长度 + 数据 + 校验和1 + 校验和2
  std::vector<uint8_t> frame_buffer(4 + len + 2);  // 4字节头部 + 数据长度 + 2字节校验和
  
  // 填充帧头部分
  frame_buffer[0] = V7_FRAME_HEADER;      // 帧头
  frame_buffer[1] = V7_TARGET_ADDRESS;    // 目标地址
  frame_buffer[2] = id;                   // 功能码
  frame_buffer[3] = len;                  // 数据长度
  
  // 复制数据
  if (len > 0 && data != nullptr)
  {
    std::copy(data, data + len, frame_buffer.begin() + 4);
  }
  
  // 计算校验和（从帧头开始到数据结束）
  uint8_t sum = 0, add = 0;
  calculateV7Checksum(frame_buffer.data(), 4 + len, sum, add);
  
  // 添加校验和
  frame_buffer[4 + len] = sum;
  frame_buffer[4 + len + 1] = add;
  
  // 发送数据
  try
  {
    size_t bytes_written = serial_port_->write(frame_buffer);
    if (bytes_written == frame_buffer.size())
    {
      RCLCPP_DEBUG(this->get_logger(),
                  "Sent V7 frame: ID=0x%02X, Len=%d, SUM=0x%02X, ADD=0x%02X",
                  id, len, sum, add);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
                 "Partial write: %zu of %zu bytes",
                 bytes_written, frame_buffer.size());
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send V7 frame: %s", e.what());
  }
}

void AnoSerialDriverNode::calculateV7Checksum(const uint8_t* buffer, size_t length, uint8_t& sum, uint8_t& add)
{
  sum = 0;
  add = 0;
  
  for (size_t i = 0; i < length; ++i)
  {
    sum += buffer[i];  // 自然溢出
    add += sum;        // 自然溢出
  }
}

void AnoSerialDriverNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!serial_port_ || !serial_port_->isOpen())
  {
    RCLCPP_WARN(this->get_logger(), "Serial port not open, cannot send pose command");
    return;
  }

  RCLCPP_DEBUG(this->get_logger(),
              "Received PoseStamped: x=%.3f, y=%.3f, z=%.3f",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  // 填充ano_ekf_pose_t结构体
  ano_ekf_pose_t ekf_pose;
  
  // 单位转换：位置 (m) 转换为 STM32 的 0.01cm，即 static_cast<int32_t>(val * 10000.0)
  ekf_pose.pos_x = static_cast<int32_t>(msg->pose.position.x * 10000.0);
  ekf_pose.pos_y = static_cast<int32_t>(msg->pose.position.y * 10000.0);
  ekf_pose.pos_z = static_cast<int32_t>(msg->pose.position.z * 10000.0);
  
  // 从四元数提取偏航角 (yaw)
  // 简化处理：这里假设偏航角为0，实际应用中应从四元数计算
  ekf_pose.yaw = 0;
  
  // 速度字段设为0（因为PoseStamped不包含速度信息）
  ekf_pose.vel_x = 0;
  ekf_pose.vel_y = 0;
  ekf_pose.vel_z = 0;
  
  // 状态标志位
  ekf_pose.state = 0x01;  // 有效数据标志
  
  // 发送0x05帧（根据要求）
  sendV7Frame(0x05, reinterpret_cast<const uint8_t*>(&ekf_pose), sizeof(ekf_pose));
  
  RCLCPP_DEBUG(this->get_logger(), "Sent pose command to STM32");
}

void AnoSerialDriverNode::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!serial_port_ || !serial_port_->isOpen())
  {
    RCLCPP_WARN(this->get_logger(), "Serial port not open, cannot send twist command");
    return;
  }

  RCLCPP_DEBUG(this->get_logger(),
              "Received Twist: linear=(%.3f, %.3f, %.3f), angular=(%.3f, %.3f, %.3f)",
              msg->linear.x, msg->linear.y, msg->linear.z,
              msg->angular.x, msg->angular.y, msg->angular.z);

  // 填充ano_velocity_cmd_t结构体
  ano_velocity_cmd_t velocity_cmd;
  
  // 单位转换：角速度 (rad/s) 转换为 0.01°/s，即 static_cast<int16_t>(val * (180.0 / M_PI) * 100.0)
  const double angular_scale = (180.0 / M_PI) * 100.0;
  velocity_cmd.roll_rate = static_cast<int16_t>(msg->angular.x * angular_scale);
  velocity_cmd.pitch_rate = static_cast<int16_t>(msg->angular.y * angular_scale);
  velocity_cmd.yaw_rate = static_cast<int16_t>(msg->angular.z * angular_scale);
  
  // 单位转换：线速度 (m/s) 转换为 STM32 的 0.01cm/s，即 static_cast<int16_t>(val * 10000.0)
  const double linear_scale = 10000.0;
  velocity_cmd.vel_x = static_cast<int16_t>(msg->linear.x * linear_scale);
  velocity_cmd.vel_y = static_cast<int16_t>(msg->linear.y * linear_scale);
  velocity_cmd.vel_z = static_cast<int16_t>(msg->linear.z * linear_scale);
  
  // 状态标志位和保留字节
  velocity_cmd.state = 0x01;     // 有效数据标志
  velocity_cmd.reserved = 0x00;  // 保留字节
  
  // 发送0x41帧（根据要求）
  sendV7Frame(0x41, reinterpret_cast<const uint8_t*>(&velocity_cmd), sizeof(velocity_cmd));
  
  RCLCPP_DEBUG(this->get_logger(), "Sent velocity command to STM32");
}

}  // namespace ano_drone_bringup

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ano_drone_bringup::AnoSerialDriverNode)