#include "kmd_battery_uart_driver/io_worker.hpp"
#include <spdlog/fmt/bin_to_hex.h>

namespace kmd_battery_uart_driver {

IO_Worker::~IO_Worker() {
  Close();
  spdlog::info("kmd_battery_uart_driver: IO_Worker: Shutting down worker");
}

bool IO_Worker::Initialize_Serial() {
  // open serial port
  try {
    m_stream.Open(m_device);
    m_stream.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
    m_stream.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    m_stream.SetParity(LibSerial::Parity::PARITY_NONE);
    m_stream.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    m_stream.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

    spdlog::info("kmd_battery_uart_driver: IO_Worker: Opened serial port {}", m_device);
  } catch (const LibSerial::OpenFailed& e) {
    spdlog::error("kmd_battery_uart_driver: IO_Worker: Could not open serial port : {}", m_device);
    return false;
  }

  m_stopping_flag.store(false);
  m_connected.store(true);
  m_background_thread = std::thread([this](){Read();});

  return true;
}

void IO_Worker::Wait(const unsigned int timeout_milliseconds) {
  std::chrono::milliseconds duration(timeout_milliseconds);
  std::unique_lock<std::mutex> lck(m_read_mutex);
  m_read_condition.wait_for(lck, duration);
}

bool IO_Worker::Send_Message(const Battery_Message& battery_message) {
  std::lock_guard<std::mutex> lock(m_write_mutex);

  std::array<char, 13> battery_message_serialized;
  battery_message_serialized[0] = m_start_flag;
  battery_message_serialized[1] = battery_message.m_pc_address;
  battery_message_serialized[2] = battery_message.m_data_id;
  battery_message_serialized[3] = 8;
  std::memcpy(battery_message_serialized.data() + 4, battery_message.m_data.data(), 8);
  battery_message_serialized[12] = Calculate_Checksum(battery_message_serialized.data());

  try {
    m_stream.write(battery_message_serialized.data(), 13);
    m_stream.DrainWriteBuffer();
  } catch (std::runtime_error) {
    spdlog::error(
      "U-Blox driver: Could not send 13 bytes: {1}",
      spdlog::to_hex(battery_message_serialized.data(), battery_message_serialized.end())
    );
    
    return false;
  }

  return true;
}

void IO_Worker::Read() {
  spdlog::info("kmd_battery_uart_driver: IO_Worker: Starting read thread");

  while (!m_stopping_flag.load()) {
    if (m_stream.IsOpen() && m_stream.IsDataAvailable()) {
      std::lock_guard<std::mutex> lock(m_read_mutex);

      const int num_bytes_available = m_stream.GetNumberOfBytesAvailable();
      const int max_bytes_transferrable = m_read_buffer.size() - 2000;
      int bytes_to_transfer;
      if (num_bytes_available < max_bytes_transferrable) {
        bytes_to_transfer = num_bytes_available;
      } else {
        bytes_to_transfer = max_bytes_transferrable;

        spdlog::warn("kmd_battery_uart_driver: IO_Worker: Read buffer full, cannot transfer incoming bytes");
      }

      if (bytes_to_transfer > 0) {
        m_stream.read(reinterpret_cast<char *>(m_read_buffer.data()) + m_read_buffer_size, bytes_to_transfer);

        if (m_debug_messages_enabled.load()) {
          spdlog::info(
            "kmd_battery_uart_driver: IO_Worker: Received {0:d} bytes: {1}",
            bytes_to_transfer,
            spdlog::to_hex(m_read_buffer.data() + m_read_buffer_size - bytes_to_transfer, m_read_buffer.data() + m_read_buffer_size)
          );
        }

        m_read_buffer_size += bytes_to_transfer;
      }

      const char* buffer_data_ptr = m_read_buffer.data();

      while (m_read_buffer_size >= 13) {
        // look for the start flag
        if (*buffer_data_ptr == m_start_flag) {
          if (Verify_Checksum(buffer_data_ptr)) {

            if (m_debug_messages_enabled.load()) {
              spdlog::info(
                "kmd_battery_uart_driver: IO_Worker: Extracted a message: {1}",
                spdlog::to_hex(buffer_data_ptr, buffer_data_ptr + 13)
              );
            }

            m_callback_handler.Handle_Message(std::move(Battery_Message(buffer_data_ptr)));

            m_read_condition.notify_all();

            buffer_data_ptr += 13;
            m_read_buffer_size -= 13;

            continue;
          }
        }
        buffer_data_ptr += 1;
        m_read_buffer_size -= 1;
      };
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  } 
}

bool IO_Worker::Verify_Checksum(const char const* data_ptr) const {
  unsigned char tester = 0;
  for (int i = 4; i < 13; i++) {
    tester += *(data_ptr + i);
  }
  return (tester == 0xFF);
}

char IO_Worker::Calculate_Checksum(const char const* data_ptr) const {
  unsigned char minuser = 0;
  for (int i = 4; i < 12; i++) {
    minuser += *(data_ptr + i);
  }
  
  return (0xFF - minuser);
}

void IO_Worker::Close() {
  const std::lock_guard<std::mutex> lock(m_read_mutex);
  m_stopping_flag.store(true);
  if (m_background_thread.joinable()) {
    m_background_thread.join();
    spdlog::info("kmd_battery_uart_driver: IO_Worker: Stopped reading from serial port");
  }
  if (m_stream.IsOpen()) {
    m_stream.Close();
    spdlog::info("kmd_battery_uart_driver: IO_Worker: CLosed serial port {}", m_device);
  }
  m_connected.store(false);
}

} // namespace kmd_battery_uart_driver