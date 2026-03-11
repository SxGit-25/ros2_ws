#ifndef ANO_DRONE_BRINGUP__ANO_PROTOCOL_HPP_
#define ANO_DRONE_BRINGUP__ANO_PROTOCOL_HPP_

#include <cstdint>
#include <cstring>

namespace ano_drone_bringup
{

/**
 * @brief 匿名V7协议帧头定义
 */
#pragma pack(push, 1)  // 1字节对齐

/**
 * @brief 匿名V7协议帧头结构体
 */
struct AnoFrameHeader
{
  uint8_t frame_head;      // 帧头 0xAA
  uint8_t target_addr;     // 目标地址 0xFF
  uint8_t frame_id;        // 功能码/帧ID
  uint8_t data_len;        // 数据长度
} __attribute__((__packed__));

/**
 * @brief 0x01帧 IMU/光流数据结构体 (从STM32接收)
 * 数据长度: 根据实际协议定义，这里假设为72字节
 */
struct AnoImuFlowData
{
  // IMU数据
  float accel_x;          // 加速度 X (m/s²)
  float accel_y;          // 加速度 Y (m/s²)
  float accel_z;          // 加速度 Z (m/s²)
  float gyro_x;           // 角速度 X (rad/s)
  float gyro_y;           // 角速度 Y (rad/s)
  float gyro_z;           // 角速度 Z (rad/s)
  float mag_x;            // 磁力计 X (uT)
  float mag_y;            // 磁力计 Y (uT)
  float mag_z;            // 磁力计 Z (uT)
  
  // 姿态角 (欧拉角)
  float roll;             // 横滚角 (rad)
  float pitch;            // 俯仰角 (rad)
  float yaw;              // 偏航角 (rad)
  
  // 四元数
  float qw;
  float qx;
  float qy;
  float qz;
  
  // 光流数据
  float flow_x;           // 光流 X 方向速度 (pixel/s)
  float flow_y;           // 光流 Y 方向速度 (pixel/s)
  uint8_t flow_quality;   // 光流质量 (0-255)
  
  // 高度数据
  float baro_altitude;    // 气压计高度 (m)
  float ultrasonic_alt;   // 超声波高度 (m)
  
  // 系统状态
  uint8_t imu_health;     // IMU健康状态 (0:异常, 1:正常)
  uint8_t flow_health;    // 光流健康状态 (0:异常, 1:正常)
  
  // 预留字段
  uint8_t reserved[2];
} __attribute__((__packed__));

/**
 * @brief 0x05帧 EKF位姿数据结构体 (发送给STM32)
 * 数据长度: 21字节 (根据需求文档)
 */
struct AnoEKFData
{
  float position_x;       // X位置 (m)
  float position_y;       // Y位置 (m)
  float position_z;       // Z位置 (m)
  float velocity_x;       // X速度 (m/s)
  float velocity_y;       // Y速度 (m/s)
  float velocity_z;       // Z速度 (m/s)
  float quaternion_w;     // 四元数 w
  float quaternion_x;     // 四元数 x
  float quaternion_y;     // 四元数 y
  float quaternion_z;     // 四元数 z
  uint8_t pose_valid;     // 位姿有效性 (0:无效, 1:有效)
} __attribute__((__packed__));

/**
 * @brief 0x41帧 速度控制指令结构体 (发送给STM32)
 * 数据长度: 14字节 (根据需求文档)
 */
struct AnoVelocityCmd
{
  float vx_cmd;           // X方向速度指令 (m/s)
  float vy_cmd;           // Y方向速度指令 (m/s)
  float vz_cmd;           // Z方向速度指令 (m/s)
  float yaw_rate_cmd;     // 偏航角速度指令 (rad/s)
  uint8_t cmd_valid;      // 指令有效性 (0:无效, 1:有效)
  uint8_t control_mode;   // 控制模式 (0:位置, 1:速度, 2:姿态)
} __attribute__((__packed__));

/**
 * @brief 完整的V7协议帧结构体 (用于发送)
 */
template<typename DataType>
struct AnoV7Frame
{
  AnoFrameHeader header;
  DataType data;
  uint8_t checksum_sum;
  uint8_t checksum_add;
} __attribute__((__packed__));

#pragma pack(pop)  // 恢复默认对齐

/**
 * @brief 计算V7协议校验和
 * @param data 数据指针
 * @param len 数据长度
 * @param sum 输出SUM校验和
 * @param add 输出ADD校验和
 */
inline void calculate_v7_checksum(const uint8_t* data, uint8_t len, uint8_t& sum, uint8_t& add)
{
  sum = 0;
  add = 0;
  
  for (uint8_t i = 0; i < len; ++i) {
    sum += data[i];
    add += sum;
  }
}

/**
 * @brief 验证V7协议校验和
 * @param data 数据指针
 * @param len 数据长度
 * @param received_sum 接收到的SUM校验和
 * @param received_add 接收到的ADD校验和
 * @return true 校验通过，false 校验失败
 */
inline bool verify_v7_checksum(const uint8_t* data, uint8_t len, uint8_t received_sum, uint8_t received_add)
{
  uint8_t calc_sum, calc_add;
  calculate_v7_checksum(data, len, calc_sum, calc_add);
  return (calc_sum == received_sum) && (calc_add == received_add);
}

/**
 * @brief 构建完整的V7协议帧
 * @tparam DataType 数据类型
 * @param frame_id 帧ID
 * @param data 数据
 * @param buffer 输出缓冲区
 * @return 帧长度
 */
template<typename DataType>
size_t build_v7_frame(uint8_t frame_id, const DataType& data, uint8_t* buffer)
{
  // 构建帧头
  AnoFrameHeader header;
  header.frame_head = 0xAA;
  header.target_addr = 0xFF;
  header.frame_id = frame_id;
  header.data_len = sizeof(DataType);
  
  // 计算校验和
  uint8_t sum, add;
  uint8_t* data_ptr = reinterpret_cast<uint8_t*>(const_cast<DataType*>(&data));
  calculate_v7_checksum(data_ptr, sizeof(DataType), sum, add);
  
  // 复制到缓冲区
  size_t offset = 0;
  memcpy(buffer + offset, &header, sizeof(header));
  offset += sizeof(header);
  memcpy(buffer + offset, &data, sizeof(data));
  offset += sizeof(data);
  buffer[offset++] = sum;
  buffer[offset++] = add;
  
  return offset;
}

}  // namespace ano_drone_bringup

#endif  // ANO_DRONE_BRINGUP__ANO_PROTOCOL_HPP_