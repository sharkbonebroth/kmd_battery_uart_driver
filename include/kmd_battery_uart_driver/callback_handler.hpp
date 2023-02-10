#pragma once

#include <unordered_map>
#include <functional>
#include <mutex>
#include "kmd_battery_uart_driver/message.hpp"

namespace kmd_battery_uart_driver {

/**
 * @brief Callback_Handler class. This class contains a Handle_Message function that would be called by the io_worker on the receipt
 * of a message
 * 
 */
class Callback_Handler {

public:

  typedef std::function<void(Battery_Message&& message)> Battery_Callback_Function;
  
  /**
   * @brief Adds a callback function to be called upon the recipt of the correct message type
   * 
   * @param data_id ID of the messages to call the callback function on
   * @param battery_callback_function The callback function to be called
   */
  void Insert_Callback(char data_id, Battery_Callback_Function battery_callback_function) {
    std::lock_guard<std::mutex> lock(m_callback_mutex);
    m_callback_functions.insert({data_id, battery_callback_function});
  };

  /**
   * @brief Calls the relevant callback function for a received message
   * 
   * @param message Battery_Message type received by the IO_Worker
   */
  void Handle_Message(Battery_Message&& message);

private:

  std::unordered_map<char, Battery_Callback_Function> m_callback_functions;
  
  std::mutex m_callback_mutex;
}; // class Callback_Handler

} // namespace kmd_battery_uart_driver