#pragma once

#include <vector>
#include <cstring>

namespace kmd_battery_uart_driver {

/**
 * @brief Battery_Message struct representing the data to be transmitted or received from the battery
 */
struct Battery_Message {

  /**
   * @brief Construct a new Battery_Message object from a stream of bytes
   * 
   * @param data_ptr a pointer to the buffer data to initialize the Battery_Message from 
   */
  Battery_Message(const char* data_ptr) {
    m_pc_address = *(data_ptr + 1);
    m_data_id = *(data_ptr + 2);
    std::memcpy(m_data.data(), data_ptr + 4, 8);
  }

  char m_pc_address = 0;
  char m_data_id = 0;
  std::array<char, 8> m_data = {0, 0, 0, 0, 0, 0, 0, 0};
}; // class Battery_Message

} // namespace kmd_battery_uart_driver