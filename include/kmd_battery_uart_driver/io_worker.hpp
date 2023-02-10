#pragma once

#include <spdlog/spdlog.h>
#include <libserial/SerialStream.h>
#include <string>
#include <array>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include "kmd_battery_uart_driver/callback_handler.hpp"

namespace kmd_battery_uart_driver {

/**
 * @brief IO_Worker class to handle reading and writing messages to and from the battery
 */
class IO_Worker {

public:

  /**
   * @brief Construct a new io worker object
   * 
   * @param device The USB device to connect to
   */
  IO_Worker(std::string device) {m_device = device;};

  ~IO_Worker();

  /**
   * @brief Enables the printing of debug messages. Debug messages include: Received bytes, Transmitted bytes and extracted message bytes
   * 
   */
  void Enable_Debug_Messages() {m_debug_messages_enabled.store(true); }

  /**
   * @brief Initializes the serial connection to the battery and starts the background thread that
   * reads from it
   * 
   * @return true on success
   * @return false on failure
   */
  bool Initialize_Serial();

  /**
   * @brief Checks the serial connection status to the battery
   * 
   * @return true if connected to battery
   * @return false if not connected to battery
   */
  bool Is_Connected() const {return m_connected;};

  /**
   * @brief Wait until the next message is received
   * 
   * @param timeout_milliseconds timeout in milliseconds
   */
  void Wait(const unsigned int timeout_milliseconds);

private:

  /**
   * @brief Sends a message to the battery
   * 
   * @param battery_message A Battery_Message type containing the data to be sent
   * @return true if message was successfully sent
   * @return false otherwise
   */
  bool Send_Message(const Battery_Message& battery_message);

  /**
   * @brief This function is called on the m_background_thread to dump available bytes on the read buffer
   * , extract messages from it and call the callback function
   */
  void Read();

  /**
   * @brief Verifies the checksum of a given stream of bytes representing a serialized battery message
   * 
   * @param data_ptr The pointer to the data for which to verify the checksum
   * @return bool indicating if the checksum is correct
   */
  bool Verify_Checksum(const char const* data_ptr) const;

  /**
   * @brief Calculates the checksum value of a series of bytes representing a serialized battery message
   * 
   * @param data_ptr The pointer to the data for which to calculate the checksum
   * @return char Checksum value
   */
  char Calculate_Checksum(const char const* data_ptr) const;

  /**
   * @brief Shuts down the read thread 
   */
  void Close();

public:

  std::string m_device;

private:

  static constexpr char m_start_flag = 0xA5;

  std::atomic<bool> m_debug_messages_enabled{false};
  
  LibSerial::SerialStream m_stream; //!< The I/O stream  

  std::thread m_background_thread;

  std::atomic<bool> m_stopping_flag{true};

  std::array<char, 2000> m_read_buffer;
  std::mutex m_read_mutex;
  unsigned int m_read_buffer_size;

  std::condition_variable m_read_condition;

  std::mutex m_write_mutex;

  Callback_Handler m_callback_handler;

  std::atomic<bool> m_connected{false};

}; // class IO_Worker

} // namespace kmd_battery_uart_driver