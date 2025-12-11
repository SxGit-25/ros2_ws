/*
 * ano_protocol.h
 * 匿名科创 V7 通信协议定义头文件
 * 基于文档: 匿名通信协议V7.00 (2020.04.30)
 * * 适用于: STM32飞控 <-> ROS2 上位机 (树莓派)
 */

#ifndef ANO_PROTOCOL_H
#define ANO_PROTOCOL_H

#include <cstdint>

// ==========================================
[cite_start]// 1. 基础协议常量 [cite: 1309]
// ==========================================
namespace AnoProtocol {

    static const uint8_t FRAME_HEAD = 0xAA;     // 帧头
    
    [cite_start]// 硬件地址定义 [cite: 1656]
    static const uint8_t ADDR_HOST  = 0xAF;     // 上位机 (树莓派)
    static const uint8_t ADDR_FC    = 0x05;     // 拓空者/凌霄飞控
    static const uint8_t ADDR_BROADCAST = 0xFF; // 广播地址

    // 功能码 (Frame ID) 定义
    // --- 接收类 (RX, From Flight Controller) ---
    static const uint8_t ID_IMU_RAW     = 0x01; [cite_start]// 惯性传感器数据 [cite: 1361]
    static const uint8_t ID_EULER       = 0x03; [cite_start]// 欧拉角姿态 [cite: 1375]
    static const uint8_t ID_QUAT        = 0x04; [cite_start]// 四元数姿态 [cite: 1381]
    static const uint8_t ID_VELOCITY    = 0x07; [cite_start]// 飞行速度数据 [cite: 1414]
    static const uint8_t ID_OPTICAL_FLOW= 0x51; [cite_start]// 光流数据 [cite: 1537]
    static const uint8_t ID_BATTERY     = 0x0D; [cite_start]// 电压电流 [cite: 1445]

    // --- 发送类 (TX, To Flight Controller) ---
    static const uint8_t ID_EXT_VEL     = 0x33; [cite_start]// 通用速度传感器 (用于EKF闭环) [cite: 1507]
    static const uint8_t ID_CONTROL     = 0x41; [cite_start]// 实时控制帧 (用于Nav2控制) [cite: 1526]

    // ==========================================
    // 2. 数据载荷结构体 (Payload)
    [cite_start]// 注意：协议采用小端模式 (Little Endian) [cite: 1310]
    // ==========================================

    #pragma pack(push, 1) // 强制1字节对齐

    /**
     * [cite_start]@brief ID: 0x01 惯性传感器数据 [cite: 1364]
     * 长度: 13 bytes
     */
    struct Frame0x01_IMU {
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
        int16_t gyr_x;
        int16_t gyr_y;
        int16_t gyr_z;
        uint8_t shock_sta; // 震动状态
    };

    /**
     * [cite_start]@brief ID: 0x04 四元数格式姿态 [cite: 1384]
     * 长度: 9 bytes
     * [cite_start]说明: 数据传输时扩大了 10000 倍 [cite: 1385]
     */
    struct Frame0x04_Quat {
        int16_t v0; // w * 10000
        int16_t v1; // x * 10000
        int16_t v2; // y * 10000
        int16_t v3; // z * 10000
        uint8_t fusion_sta;
    };

    /**
     * [cite_start]@brief ID: 0x51 匿名光流数据 (Mode 1: 融合光流) [cite: 1545]
     * 长度: 5 bytes (最小长度)
     * 注意: 只解析 Mode=1 的情况，单位 cm/s
     */
    struct Frame0x51_Flow {
        uint8_t mode;      // 必须为 1
        uint8_t state;     // 0:无效, 1:有效
        int16_t dx_speed;  // 地面速度 X, cm/s
        int16_t dy_speed;  // 地面速度 Y, cm/s
        uint8_t quality;   // (可选) 质量 0-255
    };

    /**
     * [cite_start]@brief ID: 0x33 通用速度型传感器数据 (TX) [cite: 1510]
     * 长度: 6 bytes
     * 说明: 用于将 EKF 算出的速度传回飞控，单位 cm/s
     * [cite_start]坐标系: 捷联载体坐标系 (X机头，Y机左) [cite: 1512]
     */
    struct Frame0x33_ExtVel {
        int16_t speed_x; // cm/s
        int16_t speed_y; // cm/s
        int16_t speed_z; // cm/s
    };

    /**
     * [cite_start]@brief ID: 0x41 实时控制帧 (TX) [cite: 1529]
     * 长度: 14 bytes
     * 说明: 用于 Nav2 控制飞控飞行
     */
    struct Frame0x41_Control {
        int16_t ctrl_rol;     // 姿态角控制 (放大100倍)
        int16_t ctrl_pit;     // 姿态角控制 (放大100倍)
        int16_t ctrl_thr;     // 油门 0-1000
        int16_t ctrl_yaw_dps; [cite_start]// 偏航角速度 deg/s [cite: 1532]
        int16_t ctrl_spd_x;   [cite_start]// 期望速度 X cm/s [cite: 1533]
        int16_t ctrl_spd_y;   // 期望速度 Y cm/s
        int16_t ctrl_spd_z;   // 期望速度 Z cm/s
    };

    #pragma pack(pop) // 恢复默认对齐

    // ==========================================
    // 3. 辅助转换函数 (内联)
    // ==========================================
    
    // 浮点转定点 (用于发送)
    inline int16_t float_to_cm_s(double val_m_s) {
        return static_cast<int16_t>(val_m_s * 100.0f);
    }

    // 定点转浮点 (用于接收四元数)
    inline double parse_quat_element(int16_t raw) {
        return static_cast<double>(raw) / 10000.0;
    }
}

#endif // ANO_PROTOCOL_H