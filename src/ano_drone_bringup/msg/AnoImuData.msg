# 匿名V7协议 0x01帧 IMU/光流数据
# 时间戳
builtin_interfaces/Time stamp

# IMU数据
float32 accel_x          # 加速度 X (m/s²)
float32 accel_y          # 加速度 Y (m/s²)
float32 accel_z          # 加速度 Z (m/s²)
float32 gyro_x           # 角速度 X (rad/s)
float32 gyro_y           # 角速度 Y (rad/s)
float32 gyro_z           # 角速度 Z (rad/s)
float32 mag_x            # 磁力计 X (uT)
float32 mag_y            # 磁力计 Y (uT)
float32 mag_z            # 磁力计 Z (uT)

# 姿态角 (欧拉角)
float32 roll             # 横滚角 (rad)
float32 pitch            # 俯仰角 (rad)
float32 yaw              # 偏航角 (rad)

# 四元数
float32 qw
float32 qx
float32 qy
float32 qz

# 光流数据
float32 flow_x           # 光流 X 方向速度 (pixel/s)
float32 flow_y           # 光流 Y 方向速度 (pixel/s)
float32 flow_quality     # 光流质量 (0-255)

# 高度数据
float32 baro_altitude    # 气压计高度 (m)
float32 ultrasonic_alt   # 超声波高度 (m)

# 系统状态
uint8 imu_health         # IMU健康状态 (0:异常, 1:正常)
uint8 flow_health        # 光流健康状态 (0:异常, 1:正常)