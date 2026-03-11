#ifndef ANO_DRONE_BRINGUP__ANO_HARDWARE_INTERFACE_NODE_HPP_
#define ANO_DRONE_BRINGUP__ANO_HARDWARE_INTERFACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "ano_drone_bringup/msg/ano_imu_data.hpp"
#include "ano_drone_bringup/ano_protocol.hpp"

#include <serial/serial.h>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <vector>

namespace ano_drone_bringup
{

/**
 * @brief 匿名V7协议硬件接口节点
 * 
 * 功能：
 * 1. 通过串口与STM32飞控通信
 * 2. 接收0x01帧（IMU/光流数据）并发布为ROS2话题
 * 3. 订阅位姿和控制指令话题，打包为0x05帧和0x41帧发送给STM32
 */
class AnoHardwareInterfaceNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   * @param options ROS2节点选项
   */
  explicit AnoHardwareInterfaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief 析构函数
   */
  ~AnoHardwareInterfaceNode();

private:
  /**
   * @brief 初始化参数
   */
  void initParameters();

  /**
   * @brief 初始化串口
   * @return true 初始化成功，false 初始化失败
   */
  bool initSerialPort();

  /**
   * @brief 初始化ROS2话题发布器和订阅器
   */
  void initPublishersAndSubscribers();

  /**
   * @brief 启动串口读取线程
   */
  void startSerialReadThread();

  /**
   * @brief 串口读取线程函数
   */
  void serialReadThreadFunc();

  /**
   * @brief 解析接收到的串口数据
   * @param data 接收到的数据
   * @param length 数据长度
   */
  void parseSerialData(const uint8_t* data, size_t length);

  /**
   * @brief 处理完整的V7协议帧
   * @param frame_id 帧ID
   * @param data 帧数据
   * @param data_len 数据长度
   */
  void processV7Frame(uint8_t frame_id, const uint8_t* data, uint8_t data_len);

  /**
   * @brief 处理0x01帧（IMU/光流数据）
   * @param data 帧数据
   */
  void processImuFlowFrame(const uint8_t* data);

  /**
   * @brief 发布IMU数据到ROS2话题
   * @param imu_data IMU数据结构体
   */
  void publishImuData(const AnoImuFlowData& imu_data);

  /**
   * @brief 位姿话题回调函数
   * @param msg 位姿消息
   */
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief 速度控制话题回调函数
   * @param msg 速度控制消息
   */
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief 发送0x05帧（EKF位姿数据）到STM32
   * @param pose 位姿数据
   */
  void sendEKFData(const geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief 发送0x41帧（速度控制指令）到STM32
   * @param twist 速度控制数据
   */
  void sendVelocityCmd(const geometry_msgs::msg::Twist& twist);

  /**
   * @brief 通过串口发送数据
   * @param data 数据指针
   * @param length 数据长度
   * @return true 发送成功，false 发送失败
   */
  bool sendSerialData(const uint8_t* data, size_t length);

  // ROS2参数
  std::string serial_port_;      // 串口号，如 /dev/ttyUSB0
  int baud_rate_;                // 波特率，如 500000
  bool use_custom_msg_;          // 是否使用自定义消息

  // 串口对象
  std::unique_ptr<serial::Serial> serial_port_;
  std::atomic<bool> serial_connected_{false};

  // 串口读取线程
  std::thread serial_read_thread_;
  std::atomic<bool> thread_running_{false};

  // 数据缓冲区
  std::vector<uint8_t> rx_buffer_;
  static constexpr size_t RX_BUFFER_SIZE = 1024;

  // 协议解析状态机
  enum class ParseState {
    WAITING_FOR_HEADER,
    READING_FRAME_ID,
    READING_DATA_LEN,
    READING_DATA,
    READING_CHECKSUM_SUM,
    READING_CHECKSUM_ADD
  };
  ParseState parse_state_{ParseState::WAITING_FOR_HEADER};
  uint8_t current_frame_id_{0};
  uint8_t current_data_len_{0};
  std::vector<uint8_t> current_data_;
  uint8_t expected_bytes_{0};

  // ROS2发布器
  rclcpp::Publisher<ano_drone_bringup::msg::AnoImuData>::SharedPtr imu_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // ROS2订阅器
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;

  // 话题名称参数
  std::string imu_topic_;
  std::string pose_topic_;
  std::string velocity_topic_;

  // 互斥锁保护串口发送
  std::mutex serial_mutex_;

  // 统计信息
  std::atomic<uint64_t> rx_frame_count_{0};
  std::atomic<uint64_t> tx_frame_count_{0};
  std::atomic<uint64_t> checksum_error_count_{0};
};

}  // namespace ano_drone_bringup

#endif  // ANO_DRONE_BRINGUP__ANO_HARDWARE_INTERFACE_NODE_HPP_