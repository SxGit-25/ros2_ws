#ifndef ANO_DRONE_BRINGUP__ANO_SERIAL_DRIVER_HPP_
#define ANO_DRONE_BRINGUP__ANO_SERIAL_DRIVER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ano_drone_bringup
{

// V7协议常量定义
constexpr uint8_t V7_FRAME_HEADER = 0xAA;
constexpr uint8_t V7_TARGET_ADDRESS = 0xFF;
constexpr uint8_t V7_IMU_DATA_ID = 0x01;
constexpr uint8_t V7_EKF_POSE_ID = 0x02;
constexpr uint8_t V7_VELOCITY_CMD_ID = 0x03;

// V7帧最小长度：帧头(1) + 地址(1) + ID(1) + 长度(1) + 校验和(2) = 6字节
constexpr size_t V7_FRAME_MIN_LENGTH = 6;
// V7帧最大长度：最小长度 + 最大数据长度(假设最大256字节)
constexpr size_t V7_FRAME_MAX_LENGTH = 6 + 256;

#pragma pack(push, 1)
/**
 * @brief IMU数据包结构体 (13字节)
 * 
 * 严格按照1字节对齐，使用固定宽度整数类型
 */
struct ano_imu_data_t
{
  int16_t acc_x;   // X轴加速度
  int16_t acc_y;   // Y轴加速度
  int16_t acc_z;   // Z轴加速度
  int16_t gyr_x;   // X轴角速度
  int16_t gyr_y;   // Y轴角速度
  int16_t gyr_z;   // Z轴角速度
  uint8_t state;   // 状态标志位
};
static_assert(sizeof(ano_imu_data_t) == 13, "ano_imu_data_t size must be 13 bytes");

/**
 * @brief EKF位姿数据包结构体 (21字节)
 * 
 * 严格按照1字节对齐，使用固定宽度整数类型
 */
struct ano_ekf_pose_t
{
  int32_t pos_x;   // X轴位置
  int32_t pos_y;   // Y轴位置
  int32_t pos_z;   // Z轴位置
  int16_t vel_x;   // X轴速度
  int16_t vel_y;   // Y轴速度
  int16_t vel_z;   // Z轴速度
  int16_t yaw;     // 偏航角
  uint8_t state;   // 状态标志位
};
static_assert(sizeof(ano_ekf_pose_t) == 21, "ano_ekf_pose_t size must be 21 bytes");

/**
 * @brief 速度控制命令结构体 (14字节)
 * 
 * 严格按照1字节对齐，使用固定宽度整数类型
 */
struct ano_velocity_cmd_t
{
  int16_t roll_rate;   // 滚转角速度
  int16_t pitch_rate;  // 俯仰角速度
  int16_t yaw_rate;    // 偏航角速度
  int16_t vel_x;       // X轴速度
  int16_t vel_y;       // Y轴速度
  int16_t vel_z;       // Z轴速度
  uint8_t state;       // 状态标志位
  uint8_t reserved;    // 保留字节
};
static_assert(sizeof(ano_velocity_cmd_t) == 14, "ano_velocity_cmd_t size must be 14 bytes");
#pragma pack(pop)

/**
 * @brief ANO串口驱动节点类
 * 
 * 负责与STM32通过串口通信，读取IMU、EKF数据，发送控制命令
 */
class AnoSerialDriverNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   * 
   * @param options ROS 2节点选项
   */
  explicit AnoSerialDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief 析构函数
   */
  ~AnoSerialDriverNode();

private:
  /**
   * @brief 串口读取线程函数
   */
  void serialReadLoop();

  /**
   * @brief 发送V7协议帧
   *
   * @param id 功能码
   * @param data 数据指针
   * @param len 数据长度
   */
  void sendV7Frame(uint8_t id, const uint8_t* data, uint8_t len);

  /**
   * @brief 计算V7协议校验和
   *
   * @param buffer 数据缓冲区
   * @param length 数据长度（从帧头开始到数据结束）
   * @param sum 输出校验和1
   * @param add 输出校验和2
   */
  void calculateV7Checksum(const uint8_t* buffer, size_t length, uint8_t& sum, uint8_t& add);

  /**
   * @brief PoseStamped消息回调函数
   *
   * @param msg 接收到的位姿消息
   */
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Twist消息回调函数
   *
   * @param msg 接收到的速度消息
   */
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // 串口配置参数
  std::string serial_port_;
  int baud_rate_;

  // ROS 2话题名称参数
  std::string imu_topic_;
  std::string pose_topic_;
  std::string twist_topic_;

  // 串口对象
  std::unique_ptr<serial::Serial> serial_port_;

  // 串口读取线程
  std::thread serial_read_thread_;

  // ROS 2 Publisher和Subscriber
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;

  // V7协议接收缓冲区
  std::vector<uint8_t> rx_buffer_;
  size_t rx_buffer_index_;

  // 接收状态机状态
  enum class RxState {
    WAITING_FOR_HEADER,
    WAITING_FOR_ADDRESS,
    WAITING_FOR_ID,
    WAITING_FOR_LENGTH,
    WAITING_FOR_DATA,
    WAITING_FOR_CHECKSUM1,
    WAITING_FOR_CHECKSUM2
  };
  RxState rx_state_;
};

}  // namespace ano_drone_bringup

#endif  // ANO_DRONE_BRINGUP__ANO_SERIAL_DRIVER_HPP_