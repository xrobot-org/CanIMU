#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: CAN/串口IMU通信模块 CAN/UART IMU Communication Module
constructor_args:
  - accl_topic: "imu_accl"
  - gyro_topic: "imu_gyro"
  - quat_topic: "imu_quat"
  - eulr_topic: "imu_eulr"
  - task_stack_depth_uart: 1536
  - task_stack_depth_can: 1536
template_args: []
required_hardware: imu_can imu_data_uart ramfs database
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstring>

#include "app_framework.hpp"
#include "can.hpp"
#include "crc.hpp"
#include "database.hpp"
#include "float_encoder.hpp"
#include "message.hpp"
#include "timebase.hpp"
#include "uart.hpp"

class CanIMU : public LibXR::Application {
 public:
  explicit CanIMU(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                  const char* accl_topic, const char* gyro_topic,
                  const char* quat_topic, const char* eulr_topic,
                  uint32_t task_stack_depth_uart, uint32_t task_stack_depth_can)
      : accl_topic_name_(accl_topic),
        gyro_topic_name_(gyro_topic),
        quat_topic_name_(quat_topic),
        eulr_topic_name_(eulr_topic),
        can_(hw.template FindOrExit<LibXR::CAN>({"imu_can"})),
        uart_(hw.template FindOrExit<LibXR::UART>({"imu_data_uart"})),
        config_(*hw.template FindOrExit<LibXR::Database>({"database"}),
                "can_imu",
                Configuration{0x30, 1, true, true, true, false, false, true}),
        cmd_file_(LibXR::RamFS::CreateFile("set_imu", CommandFunc, this)) {
    app.Register(*this);

    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    RegisterTopicCallbacks();

    thread_uart_.Create(this, ThreadUart, "can_imu_uart", task_stack_depth_uart,
                        LibXR::Thread::Priority::MEDIUM);
    thread_can_.Create(this, ThreadCan, "can_imu_can", task_stack_depth_can,
                       LibXR::Thread::Priority::MEDIUM);
  }

  void OnMonitor() override {
    // Optional: Add self-check, debug output, frequency monitor, etc.
  }

  static int CommandFunc(CanIMU* imu, int argc, char** argv) {
    if (argc == 1) {
      if (imu->config_.data_.can_enabled) {
        LibXR::STDIO::Printf<"can mode\r\ndata:">();
        if (imu->config_.data_.accl_enabled) {
          LibXR::STDIO::Printf<"accl,">();
        }
        if (imu->config_.data_.gyro_enabled) {
          LibXR::STDIO::Printf<"gyro,">();
        }
        if (imu->config_.data_.quat_enabled) {
          LibXR::STDIO::Printf<"quat,">();
        }
        if (imu->config_.data_.eulr_enabled) {
          LibXR::STDIO::Printf<"eulr,">();
        }
        LibXR::STDIO::Printf<"\r\n">();
      } else {
        LibXR::STDIO::Printf<"can output disabled.\r\n">();
      }

      if (imu->config_.data_.uart_enabled) {
        LibXR::STDIO::Printf<"uart output enabled.\r\n">();
      } else {
        LibXR::STDIO::Printf<"uart output disabled.\r\n">();
      }

      LibXR::STDIO::Printf<"feedback delay:%u\r\n">(
          static_cast<unsigned>(imu->config_.data_.fb_cycle));
      LibXR::STDIO::Printf<"id:%u\r\n\r\nUsage:\r\n">(
          static_cast<unsigned>(imu->config_.data_.id));
      LibXR::STDIO::Printf<"\tset_delay  [time]  设置发送延时ms\r\n">();
      LibXR::STDIO::Printf<"\tset_can_id [id]    设置can id\r\n">();
      LibXR::STDIO::Printf<"\tenable/disable     "
          "[accl/gyro/quat/eulr/can/uart]\r\n">();
    } else if (argc == 3 && strcmp(argv[1], "set_delay") == 0) {
      int delay = std::stoi(argv[2]);

      if (delay > 1000) {
        delay = 1000;
      }

      if (delay < 1) {
        delay = 1;
      }

      imu->config_.data_.fb_cycle = delay;

      LibXR::STDIO::Printf<"delay:%d\r\n">(delay);

      imu->config_.Set(imu->config_.data_);
    } else if (argc == 3 && strcmp(argv[1], "enable") == 0) {
      if (strcmp(argv[2], "accl") == 0) {
        imu->config_.data_.accl_enabled = true;
      } else if (strcmp(argv[2], "gyro") == 0) {
        imu->config_.data_.gyro_enabled = true;
      } else if (strcmp(argv[2], "quat") == 0) {
        imu->config_.data_.quat_enabled = true;
      } else if (strcmp(argv[2], "eulr") == 0) {
        imu->config_.data_.eulr_enabled = true;
      } else if (strcmp(argv[2], "can") == 0) {
        imu->config_.data_.can_enabled = true;
      } else if (strcmp(argv[2], "uart") == 0) {
        imu->config_.data_.uart_enabled = true;
      } else {
        LibXR::STDIO::Printf<"命令错误\r\n">();
        return -1;
      }

      imu->config_.Set(imu->config_.data_);
    } else if (argc == 3 && strcmp(argv[1], "disable") == 0) {
      if (strcmp(argv[2], "accl") == 0) {
        imu->config_.data_.accl_enabled = false;
      } else if (strcmp(argv[2], "gyro") == 0) {
        imu->config_.data_.gyro_enabled = false;
      } else if (strcmp(argv[2], "quat") == 0) {
        imu->config_.data_.quat_enabled = false;
      } else if (strcmp(argv[2], "eulr") == 0) {
        imu->config_.data_.eulr_enabled = false;
      } else if (strcmp(argv[2], "can") == 0) {
        imu->config_.data_.can_enabled = false;
      } else if (strcmp(argv[2], "uart") == 0) {
        imu->config_.data_.uart_enabled = false;
      } else {
        LibXR::STDIO::Printf<"命令错误\r\n">();
        return -1;
      }

      imu->config_.Set(imu->config_.data_);
    } else if (argc == 3 && strcmp(argv[1], "set_can_id") == 0) {
      int id = std::stoi(argv[2]);

      imu->config_.data_.id = id;

      LibXR::STDIO::Printf<"can_id:%d\r\n">(id);

      imu->config_.Set(imu->config_.data_);
    } else {
      LibXR::STDIO::Printf<"命令错误\r\n">();
    }

    return 0;
  }

 private:
  const char* accl_topic_name_;
  const char* gyro_topic_name_;
  const char* quat_topic_name_;
  const char* eulr_topic_name_;

  struct __attribute__((packed)) Configuration {
    uint32_t id;
    uint32_t fb_cycle;
    bool can_enabled;
    bool uart_enabled;
    bool eulr_enabled;
    bool quat_enabled;
    bool accl_enabled;
    bool gyro_enabled;
  };

  struct __attribute__((packed)) Data {
    uint8_t prefix = 0xA5;
    uint8_t id = 0x30;  // Default ID
    uint32_t time = 0;
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f};
    float accl[3] = {0.0f};
    float eulr[3] = {0.0f};
    uint8_t crc8 = 0;
  };

  union CanData3 {
    struct __attribute__((packed)) {
      int32_t data1 : 21;
      int32_t data2 : 21;
      int32_t data3 : 21;
      int32_t res : 1;
    };

    struct __attribute__((packed)) {
      uint32_t data1_unsigned : 21;
      uint32_t data2_unsigned : 21;
      uint32_t data3_unsigned : 21;
      uint32_t res_unsigned : 1;
    };
  };

  struct __attribute__((packed)) CanData4 {
    union {
      int16_t data[4];
      uint16_t data_unsigned[4];
    };
  };

  enum class CanPackID : uint32_t { ACCL = 0, GYRO = 1, EULR = 3, QUAT = 4 };

  LibXR::CAN* can_;
  LibXR::UART* uart_;
  LibXR::Database::Key<Configuration> config_;

  LibXR::RamFS::File cmd_file_;

  LibXR::Thread thread_uart_;
  LibXR::Thread thread_can_;
  LibXR::Semaphore uart_write_sem_;

  LibXR::Quaternion<float> quat_ = {1.0f, 0.0f, 0.0f, 0.0f};
  LibXR::EulerAngle<float> eulr_ = {0.0f, 0.0f, 0.0f};
  Eigen::Matrix<float, 3, 1> gyro_ = {0.0f, 0.0f, 0.0f};
  Eigen::Matrix<float, 3, 1> accl_ = {0.0f, 0.0f, 0.0f};

  struct Vector3Sample {
    float data[3];
  };

  struct QuaternionSample {
    float data[4];
  };

  LibXR::MPMCQueue<Vector3Sample> accl_queue_{4};
  LibXR::MPMCQueue<Vector3Sample> gyro_queue_{4};
  LibXR::MPMCQueue<QuaternionSample> quat_queue_{4};
  LibXR::MPMCQueue<Vector3Sample> eulr_queue_{4};

  LibXR::Topic::Callback accl_callback_;
  LibXR::Topic::Callback gyro_callback_;
  LibXR::Topic::Callback quat_callback_;
  LibXR::Topic::Callback eulr_callback_;
  LibXR::Mutex data_mutex_;

  void RegisterTopicCallbacks() {
    auto accl_topic =
        LibXR::Topic::CreateTopic<decltype(accl_)>(accl_topic_name_);
    auto gyro_topic =
        LibXR::Topic::CreateTopic<decltype(gyro_)>(gyro_topic_name_);
    auto quat_topic =
        LibXR::Topic::CreateTopic<decltype(quat_)>(quat_topic_name_);
    auto eulr_topic =
        LibXR::Topic::CreateTopic<decltype(eulr_)>(eulr_topic_name_);

    accl_callback_ = LibXR::Topic::Callback::Create(OnAcclTopic, this);
    gyro_callback_ = LibXR::Topic::Callback::Create(OnGyroTopic, this);
    quat_callback_ = LibXR::Topic::Callback::Create(OnQuatTopic, this);
    eulr_callback_ = LibXR::Topic::Callback::Create(OnEulrTopic, this);

    accl_topic.RegisterCallback(accl_callback_);
    gyro_topic.RegisterCallback(gyro_callback_);
    quat_topic.RegisterCallback(quat_callback_);
    eulr_topic.RegisterCallback(eulr_callback_);
  }

  template <typename Queue, typename Sample>
  static void PushLatest(Queue& queue, const Sample& sample) {
    if (queue.Push(sample) == LibXR::ErrorCode::OK) {
      return;
    }

    Sample dropped{};
    (void)queue.Pop(dropped);
    (void)queue.Push(sample);
  }

  static void OnAcclTopic(bool, CanIMU* self,
                          const Eigen::Matrix<float, 3, 1>& data) {
    const Vector3Sample sample{{data.x(), data.y(), data.z()}};
    PushLatest(self->accl_queue_, sample);
  }

  static void OnGyroTopic(bool, CanIMU* self,
                          const Eigen::Matrix<float, 3, 1>& data) {
    const Vector3Sample sample{{data.x(), data.y(), data.z()}};
    PushLatest(self->gyro_queue_, sample);
  }

  static void OnQuatTopic(bool, CanIMU* self,
                          const LibXR::Quaternion<float>& data) {
    const QuaternionSample sample{{data.w(), data.x(), data.y(), data.z()}};
    PushLatest(self->quat_queue_, sample);
  }

  static void OnEulrTopic(bool, CanIMU* self,
                          const LibXR::EulerAngle<float>& data) {
    const Vector3Sample sample{{data.Roll(), data.Pitch(), data.Yaw()}};
    PushLatest(self->eulr_queue_, sample);
  }

  void DrainTopicQueues() {
    Vector3Sample vector_sample{};
    QuaternionSample quat_sample{};

    LibXR::Mutex::LockGuard lock(data_mutex_);

    while (accl_queue_.Pop(vector_sample) == LibXR::ErrorCode::OK) {
      accl_ = {vector_sample.data[0], vector_sample.data[1], vector_sample.data[2]};
    }

    while (gyro_queue_.Pop(vector_sample) == LibXR::ErrorCode::OK) {
      gyro_ = {vector_sample.data[0], vector_sample.data[1], vector_sample.data[2]};
    }

    while (quat_queue_.Pop(quat_sample) == LibXR::ErrorCode::OK) {
      quat_ = LibXR::Quaternion<float>(quat_sample.data);
    }

    while (eulr_queue_.Pop(vector_sample) == LibXR::ErrorCode::OK) {
      eulr_ = LibXR::EulerAngle<float>(vector_sample.data);
    }
  }

  static void ThreadUart(CanIMU* self) {
    self->uart_->SetConfig({1000000, LibXR::UART::Parity::NO_PARITY, 8, 1});

    Data send_buffer = {};
    LibXR::WriteOperation write_op(self->uart_write_sem_);

    auto last_waskup_time = LibXR::Timebase::GetMilliseconds();

    while (true) {
      self->DrainTopicQueues();

      if (self->config_.data_.uart_enabled) {
        send_buffer.prefix = 0xA5;
        send_buffer.id = self->config_.data_.id;
        send_buffer.time = LibXR::Timebase::GetMilliseconds();
        {
          LibXR::Mutex::LockGuard lock(self->data_mutex_);
          send_buffer.quat[0] = self->quat_.w();
          send_buffer.quat[1] = self->quat_.x();
          send_buffer.quat[2] = self->quat_.y();
          send_buffer.quat[3] = self->quat_.z();
          memcpy(send_buffer.gyro, self->gyro_.data(), sizeof(send_buffer.gyro));
          memcpy(send_buffer.accl, self->accl_.data(), sizeof(send_buffer.accl));
          memcpy(send_buffer.eulr, self->eulr_.data_, sizeof(send_buffer.eulr));
        }
        send_buffer.crc8 = LibXR::CRC8::Calculate(
            reinterpret_cast<const uint8_t*>(&send_buffer),
            sizeof(Data) - sizeof(uint8_t));

        self->uart_->Write(send_buffer, write_op);
      }

      LibXR::Thread::SleepUntil(last_waskup_time, self->config_.data_.fb_cycle);
    }
  }

  static void ThreadCan(CanIMU* self) {
    auto last_waskup_time = LibXR::Timebase::GetMilliseconds();

    LibXR::CAN::ClassicPack classic_pack = {};
    classic_pack.dlc = 8;

    CanData3* can_data3 = reinterpret_cast<CanData3*>(classic_pack.data);
    CanData4* can_data4 = reinterpret_cast<CanData4*>(classic_pack.data);

    while (true) {
      if (self->config_.data_.can_enabled) {
        classic_pack.type = LibXR::CAN::Type::STANDARD;

        using Encoder21 = LibXR::FloatEncoder<21>;

        if (self->config_.data_.gyro_enabled) {
          classic_pack.id =
              self->config_.data_.id + static_cast<uint32_t>(CanPackID::GYRO);
          Encoder21 encoder(-2000.0f * M_PI / 180.0f,
                            2000.0f * M_PI / 180.0f);  // rad/s
          Eigen::Matrix<float, 3, 1> gyro;
          {
            LibXR::Mutex::LockGuard lock(self->data_mutex_);
            gyro = self->gyro_;
          }
          can_data3->data1 = encoder.Encode(gyro.x());
          can_data3->data2 = encoder.Encode(gyro.y());
          can_data3->data3 = encoder.Encode(gyro.z());

          self->can_->AddMessage(classic_pack);
        }

        if (self->config_.data_.accl_enabled) {
          classic_pack.id =
              self->config_.data_.id + static_cast<uint32_t>(CanPackID::ACCL);
          Encoder21 encoder(-24.0f, 24.0f);  // ±24g
          Eigen::Matrix<float, 3, 1> accl;
          {
            LibXR::Mutex::LockGuard lock(self->data_mutex_);
            accl = self->accl_;
          }
          can_data3->data1 = encoder.Encode(accl.x());
          can_data3->data2 = encoder.Encode(accl.y());
          can_data3->data3 = encoder.Encode(accl.z());

          self->can_->AddMessage(classic_pack);
        }

        if (self->config_.data_.eulr_enabled) {
          classic_pack.id =
              self->config_.data_.id + static_cast<uint32_t>(CanPackID::EULR);
          Encoder21 encoder(-M_PI, M_PI);  // Euler angles in rad
          LibXR::EulerAngle<float> eulr;
          {
            LibXR::Mutex::LockGuard lock(self->data_mutex_);
            eulr = self->eulr_;
          }
          can_data3->data1 = encoder.Encode(eulr.Pitch());
          can_data3->data2 = encoder.Encode(eulr.Roll());
          can_data3->data3 = encoder.Encode(eulr.Yaw());
          self->can_->AddMessage(classic_pack);
        }

        if (self->config_.data_.quat_enabled) {
          classic_pack.id =
              self->config_.data_.id + static_cast<uint32_t>(CanPackID::QUAT);
          constexpr float SCALE =
              static_cast<float>(INT16_MAX);  // int16_t scaling
          LibXR::Quaternion<float> quat;
          {
            LibXR::Mutex::LockGuard lock(self->data_mutex_);
            quat = self->quat_;
          }
          can_data4->data[0] = static_cast<int16_t>(quat.w() * SCALE);
          can_data4->data[1] = static_cast<int16_t>(quat.x() * SCALE);
          can_data4->data[2] = static_cast<int16_t>(quat.y() * SCALE);
          can_data4->data[3] = static_cast<int16_t>(quat.z() * SCALE);
          self->can_->AddMessage(classic_pack);
        }
      }

      LibXR::Thread::SleepUntil(last_waskup_time, self->config_.data_.fb_cycle);
    }
  }
};
