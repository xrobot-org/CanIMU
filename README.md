# CanIMU

CAN/串口IMU通信模块 / CAN/UART IMU Communication Module

## 硬件需求 / Required Hardware

imu_can, imu_data_uart, ramfs, database

## 构造参数 / Constructor Arguments

- accl_topic:            "imu_accl"
- gyro_topic:            "imu_gyro"
- quat_topic:            "imu_quat"
- eulr_topic:            "imu_eulr"
- task_stack_depth_uart: 384
- task_stack_depth_can:  384

## 依赖 / Depends

无（No dependencies）
