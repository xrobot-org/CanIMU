# CanIMU

CAN/串口 IMU 数据转发模块。
CAN/UART IMU data forwarding module.

模块订阅四个 IMU Topic（加速度、角速度、四元数、欧拉角），各保留最近的样本，并以
`fb_cycle` 毫秒为周期通过经典 CAN 和 UART 发出。输出开关、周期和 CAN ID 保存在 Database
键 `can_imu` 中，可用 RamFS 命令 `set_imu` 在运行时修改，立即生效。

The Module subscribes to four IMU topics (acceleration, angular rate, quaternion,
Euler angles), keeps the latest sample of each and sends them every `fb_cycle` ms
over classic CAN and UART. Output switches, period and CAN ID live in the Database
key `can_imu` and can be changed at run time with the RamFS command `set_imu`; changes
take effect immediately.

## 订阅的 Topic / Subscribed topics

| Topic（默认 / default） | 类型 / Type |
| --- | --- |
| `accl_topic`（`imu_accl`） | `Eigen::Matrix<float, 3, 1>`，g |
| `gyro_topic`（`imu_gyro`） | `Eigen::Matrix<float, 3, 1>`，rad/s |
| `quat_topic`（`imu_quat`） | `LibXR::Quaternion<float>` |
| `eulr_topic`（`imu_eulr`） | `LibXR::EulerAngle<float>`，rad |

## 线程 / Threads

- `can_imu_uart`（MEDIUM）：把 UART 配置为 1 Mbps 8N1，每周期取出最新的 Topic 数据，
  若开启 UART 输出则发送一帧。/ Configures the UART to 1 Mbps 8N1, takes the latest
  topic data every period and sends one frame when UART output is enabled.
- `can_imu_can`（MEDIUM）：每周期按开关发送 CAN 帧。/ Sends the enabled CAN frames every
  period.

## UART 帧 / UART frame

小端、紧凑排列，共 59 字节。/ Little-endian, packed, 59 bytes.

| 字段 / Field | 类型 / Type | 说明 / Meaning |
| --- | --- | --- |
| `prefix` | `uint8_t` | `0xA5` |
| `id` | `uint8_t` | 配置的 ID / configured ID |
| `time` | `uint32_t` | 发送时刻，ms / send time in ms |
| `quat[4]` | `float` | w, x, y, z |
| `gyro[3]` | `float` | rad/s |
| `accl[3]` | `float` | g |
| `eulr[3]` | `float` | roll, pitch, yaw，rad |
| `crc8` | `uint8_t` | 前面所有字节的 LibXR CRC8 / LibXR CRC8 over all preceding bytes |

## CAN 帧 / CAN frames

标准帧，DLC 8，ID = 配置的 ID + 偏移。三轴数据为三个 21 位无符号字段（从最低位起依次打包），
由 `LibXR::FloatEncoder<21>` 把给定区间线性映射到 0 到 2^21-1。
Standard frames, DLC 8, ID = configured ID + offset. Three-axis data are three
21-bit unsigned fields packed from the least significant bit, mapped linearly from
the given range to 0 .. 2^21-1 by `LibXR::FloatEncoder<21>`.

| 偏移 / Offset | 内容 / Content | 编码 / Encoding |
| --- | --- | --- |
| +0 | 加速度 / acceleration x, y, z | ±24 g |
| +1 | 角速度 / angular rate x, y, z | ±2000 °/s，以 rad/s 表示 / in rad/s |
| +3 | 欧拉角 / Euler angles pitch, roll, yaw | ±π rad |
| +4 | 四元数 / quaternion w, x, y, z | 4 × `int16_t`，值 × 32767 / value × 32767 |

## RamFS 命令 / RamFS command

- `set_imu`：打印当前 CAN/UART 开关、已开启的数据、周期、ID 和用法。/ Print the CAN/UART
  switches, enabled data, period, ID and usage.
- `set_imu set_delay <ms>`：设置发送周期，限制在 1-1000 ms。/ Set the send period,
  clamped to 1-1000 ms.
- `set_imu set_can_id <id>`：设置 ID（CAN 基础 ID，也写入 UART 帧的 `id`）。/ Set the ID
  (CAN base ID, also written to the UART frame `id`).
- `set_imu enable|disable accl|gyro|quat|eulr|can|uart`：开关单项输出。/ Switch one output.

每条设置命令都会写入 Database。默认配置：ID `0x30`，周期 1 ms，CAN 和 UART 开启，CAN 上
只发送角速度和欧拉角。
Every setting command writes the Database. Default configuration: ID `0x30`, period
1 ms, CAN and UART enabled, angular rate and Euler angles sent on CAN.

## 依赖 / Dependencies

无其他模块依赖，仅使用 LibXR。数据来自发布上述 Topic 的 IMU / 姿态解算模块。
No other Modules; LibXR only. The data come from whatever IMU / attitude Modules
publish the topics above.

## 构造接口 / Constructor

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

依赖 / Dependencies:

- `can_bus`：发送 IMU 数据的 `LibXR::CAN`。/ CAN bus for the IMU frames.
- `uart`：发送 IMU 数据的 `LibXR::UART`。/ UART for the IMU frames.
- `database`：保存输出配置的 `LibXR::Database`。/ Stores the output configuration.
- `ramfs`：注册 `set_imu` 命令的 `LibXR::RamFS`。/ Receives the `set_imu` command.

配置 / Configuration (`Param`):

- `accl_topic` / `gyro_topic` / `quat_topic` / `eulr_topic`：订阅的 Topic 名称，默认见上表。
  / Names of the subscribed topics, defaults above.
- `task_stack_depth_uart` / `task_stack_depth_can`：两个线程的栈深，默认 1536。
  / Stack depth of the two threads, default 1536.

## 使用 / Use

```sh
xrobot module add xrobot-org/CanIMU
xrobot setup
xrobot instance add xrobot-org/CanIMU
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把依赖项填为 BSP 中用 `XR_REGISTER` 注册的对象名：
`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set the dependencies to the names of objects
the BSP registers with `XR_REGISTER`:

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
          accl_topic: '"imu_accl"'
          gyro_topic: '"imu_gyro"'
          quat_topic: '"imu_quat"'
          eulr_topic: '"imu_eulr"'
          task_stack_depth_uart: '1536'
          task_stack_depth_can: '1536'
```

BSP 侧 / BSP side:

```cpp
XR_REGISTER(can1, LibXR::CAN);
XR_REGISTER(uart_imu, LibXR::UART);
XR_REGISTER(database, LibXR::Database);
XR_REGISTER(ramfs, LibXR::RamFS);
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。
Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/xrobot-org/CanIMU`
（在 BSP 中）打印 manifest 和当前的构造函数。
`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/CanIMU` in a BSP, prints the manifest and the
current constructor.
