# CanIMU

CAN 与 UART IMU 数据转发模块 / Module that forwards IMU data over classic CAN and UART

## 1. 模块作用 / Purpose

构造后，CanIMU 订阅四个 IMU Topic（加速度、角速度、四元数、欧拉角），各保留最近的样本，并以 `fb_cycle` 毫秒为周期通过经典 CAN 和 UART 发出。`fb_cycle` 是配置的字段，该配置保存在 Database 的键 `can_imu` 中，另含 CAN ID（`id`）与各输出开关；构造时读取该键的值，键缺失时以默认配置创建。配置可用 RamFS 命令 `set_imu` 在运行时修改，修改立即生效。

模块创建两个 MEDIUM 优先级线程：

- `can_imu_uart`：把 UART 配置为 1 Mbps 8N1，每周期取出最新的 Topic 数据，开启 UART 输出时发送一帧。
- `can_imu_can`：每周期按开关发送 CAN 帧。

RamFS 命令 `set_imu`：

- `set_imu`：打印当前 CAN 与 UART 开关、已开启的数据、周期、ID 和用法。
- `set_imu set_delay <ms>`：设置发送周期 `fb_cycle`，限制在 1 到 1000 ms。
- `set_imu set_can_id <id>`：设置 ID（0–255），即 CAN 基础 ID，同时写入 UART 帧的 `id`。
- `set_imu enable|disable accl|gyro|quat|eulr|can|uart`：开关单项输出。

每条设置命令都写入 Database。默认配置：`id` 为 `0x30`，`fb_cycle` 为 1 ms，CAN 与 UART 开启，CAN 上发送角速度和欧拉角。

After construction, CanIMU subscribes to four IMU Topics (acceleration, angular velocity, quaternion, Euler angles), keeps the latest sample of each and sends them every `fb_cycle` ms over classic CAN and UART. `fb_cycle` is a field of the configuration, which is stored in the Database under the key `can_imu` and also holds the CAN ID (`id`) and the output switches; on construction the value of this key is read, and a missing key is created with the default configuration. The configuration can be changed at run time with the RamFS command `set_imu`; a change takes effect immediately.

The Module creates two MEDIUM-priority threads:

- `can_imu_uart`: configures the UART to 1 Mbps 8N1, takes the latest Topic data every period, and sends one frame when UART output is enabled.
- `can_imu_can`: sends the enabled CAN frames every period.

The RamFS command `set_imu`:

- `set_imu`: print the current CAN and UART switches, the enabled data, the period, the ID and the usage.
- `set_imu set_delay <ms>`: set the send period `fb_cycle`, limited to 1 to 1000 ms.
- `set_imu set_can_id <id>`: set the ID (0–255), which is the CAN base ID and is also written to the `id` field of the UART frame.
- `set_imu enable|disable accl|gyro|quat|eulr|can|uart`: switch one output.

Every setting command writes the Database. The default configuration is: `id` of `0x30`, `fb_cycle` of 1 ms, CAN and UART enabled, angular velocity and Euler angles sent on CAN.

## 2. 输出帧格式 / Output Frame Formats

UART 帧小端、紧凑排列，共 59 字节：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `prefix` | `uint8_t` | `0xA5` |
| `id` | `uint8_t` | 配置的 ID |
| `time` | `uint32_t` | 发送时刻，单位 ms |
| `quat[4]` | `float` | w, x, y, z |
| `gyro[3]` | `float` | 单位 rad/s |
| `accl[3]` | `float` | 单位 g |
| `eulr[3]` | `float` | roll, pitch, yaw，单位 rad |
| `crc8` | `uint8_t` | 前面所有字节的 LibXR CRC8 |

CAN 帧为标准帧，DLC 8，ID 为配置的 ID 加偏移。三轴数据为三个 21 位无符号字段，从最低位起依次打包，由 `LibXR::FloatEncoder<21>` 把给定区间线性映射到 0 到 2^21-1。

| 偏移 | 内容 | 编码 |
| --- | --- | --- |
| +0 | 加速度 x, y, z | ±24 g |
| +1 | 角速度 x, y, z | ±2000 °/s，以 rad/s 表示 |
| +3 | 欧拉角 pitch, roll, yaw | ±π rad |
| +4 | 四元数 w, x, y, z | 4 个 `int16_t`，值乘以 32767 |

The UART frame is little-endian and packed, 59 bytes in total:

| Field | Type | Meaning |
| --- | --- | --- |
| `prefix` | `uint8_t` | `0xA5` |
| `id` | `uint8_t` | Configured ID |
| `time` | `uint32_t` | Send time in ms |
| `quat[4]` | `float` | w, x, y, z |
| `gyro[3]` | `float` | In rad/s |
| `accl[3]` | `float` | In g |
| `eulr[3]` | `float` | roll, pitch, yaw, in rad |
| `crc8` | `uint8_t` | LibXR CRC8 over all preceding bytes |

CAN frames are standard frames with DLC 8 and ID equal to the configured ID plus an offset. The three-axis data are three 21-bit unsigned fields packed from the least significant bit, mapped linearly from the given range to 0 to 2^21-1 by `LibXR::FloatEncoder<21>`.

| Offset | Content | Encoding |
| --- | --- | --- |
| +0 | Acceleration x, y, z | ±24 g |
| +1 | Angular velocity x, y, z | ±2000 °/s, expressed in rad/s |
| +3 | Euler angles pitch, roll, yaw | ±π rad |
| +4 | Quaternion w, x, y, z | 4 × `int16_t`, value multiplied by 32767 |

## 3. 构造接口 / Constructor

```cpp
explicit CanIMU(LibXR::CAN& can_bus,
                LibXR::UART& uart,
                LibXR::Database& database,
                LibXR::RamFS& ramfs,
                const Param& param = {.accl_topic = "imu_accl",
                                      .gyro_topic = "imu_gyro",
                                      .quat_topic = "imu_quat",
                                      .eulr_topic = "imu_eulr",
                                      .task_stack_depth_uart = 1536,
                                      .task_stack_depth_can = 1536});
```

依赖：

- `can_bus`：发送 IMU 数据的 `LibXR::CAN`，取自 BSP 的硬件注册（`XR_REGISTER`）。
- `uart`：发送 IMU 数据的 `LibXR::UART`，取自 BSP 的硬件注册。
- `database`：保存输出配置的 `LibXR::Database`，取自 BSP 的硬件注册。
- `ramfs`：注册 `set_imu` 命令的 `LibXR::RamFS`，取自 BSP 的硬件注册。

配置参数（`Param`）：

- `accl_topic`、`gyro_topic`、`quat_topic`、`eulr_topic`：订阅的 Topic 名称，默认 `"imu_accl"`、`"imu_gyro"`、`"imu_quat"`、`"imu_eulr"`。
- `task_stack_depth_uart`、`task_stack_depth_can`：两个线程的栈深，默认 1536。

Dependencies:

- `can_bus`: the `LibXR::CAN` that sends the IMU data, taken from the BSP's Registration (`XR_REGISTER`).
- `uart`: the `LibXR::UART` that sends the IMU data, taken from the BSP's Registration.
- `database`: the `LibXR::Database` that stores the output configuration, taken from the BSP's Registration.
- `ramfs`: the `LibXR::RamFS` that receives the `set_imu` command, taken from the BSP's Registration.

Configuration parameters (`Param`):

- `accl_topic`, `gyro_topic`, `quat_topic`, `eulr_topic`: names of the subscribed Topics, default `"imu_accl"`, `"imu_gyro"`, `"imu_quat"`, `"imu_eulr"`.
- `task_stack_depth_uart`, `task_stack_depth_can`: stack depth of the two threads, default 1536.

## 4. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `accl_topic`（默认 `imu_accl`） | 订阅 | `Eigen::Matrix<float, 3, 1>` | 加速度，单位 g |
| `gyro_topic`（默认 `imu_gyro`） | 订阅 | `Eigen::Matrix<float, 3, 1>` | 角速度，单位 rad/s |
| `quat_topic`（默认 `imu_quat`） | 订阅 | `LibXR::Quaternion<float>` | 姿态四元数 |
| `eulr_topic`（默认 `imu_eulr`） | 订阅 | `LibXR::EulerAngle<float>` | 欧拉角，单位 rad |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `accl_topic` (default `imu_accl`) | Subscribe | `Eigen::Matrix<float, 3, 1>` | Acceleration in g |
| `gyro_topic` (default `imu_gyro`) | Subscribe | `Eigen::Matrix<float, 3, 1>` | Angular velocity in rad/s |
| `quat_topic` (default `imu_quat`) | Subscribe | `LibXR::Quaternion<float>` | Attitude quaternion |
| `eulr_topic` (default `imu_eulr`) | Subscribe | `LibXR::EulerAngle<float>` | Euler angles in rad |

## 5. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/CanIMU` 写入的实例，依赖填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/CanIMU`, with the dependencies set to names registered by the BSP with `XR_REGISTER` (Registration):

```yaml
modules:
  - module: xrobot-org/CanIMU
    id: canimu_0
    args:
      - can_bus: can1
      - uart: uart_imu
      - database: database
      - ramfs: ramfs
      - param:
          accl_topic: "imu_accl"
          gyro_topic: "imu_gyro"
          quat_topic: "imu_quat"
          eulr_topic: "imu_eulr"
          task_stack_depth_uart: 1536
          task_stack_depth_can: 1536
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。数据来自发布上述 Topic 的 IMU 与姿态解算模块。

硬件：一路经典 CAN 和一路 UART，由 BSP 通过 `XR_REGISTER` 注册，另需 Database 与 RamFS。

Dependencies: LibXR. The data comes from the IMU and attitude Modules that publish the Topics above.

Hardware: one classic CAN bus and one UART, registered by the BSP with `XR_REGISTER`, plus a Database and a RamFS.
