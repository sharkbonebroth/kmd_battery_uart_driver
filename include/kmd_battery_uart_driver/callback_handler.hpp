#pragma once

#include <unordered_map>
#include <map>
#include <functional>
#include <mutex>
#include "kmd_battery_uart_driver/message.hpp"

namespace kmd_battery_uart_driver {

/**
 * @brief Callback_Handler class. This class contains a Handle_Message function that would be called by the io_worker on the receipt
 * of a message
 */
class Callback_Handler {

public:

  typedef std::function<void(const Battery_Message& message)> Battery_Callback_Function;
  
  /**
   * @brief Adds a callback function to be called upon the recipt of the correct message type
   * 
   * @param data_id ID of the messages to call the callback function on
   * @param battery_callback_function The callback function to be called
   */
  std::multimap<char, Battery_Callback_Function>::iterator Insert_Callback(const char data_id, const Battery_Callback_Function battery_callback_function) {
    std::lock_guard<std::mutex> lock(m_callback_mutex);
    return m_callback_functions.insert({data_id, battery_callback_function});
  };

  /**
   * @brief Removes a callback function
   * 
   * @param callback_function_ptr An iterator pointing to the callback function to remove
   */
  void Remove_Callback(std::multimap<char, Battery_Callback_Function>::iterator callback_function_ptr) {
    std::lock_guard<std::mutex> lock(m_callback_mutex);
    m_callback_functions.erase(callback_function_ptr);
  }

  /**
   * @brief Calls the relevant callback function for a received message
   * 
   * @param message Battery_Message type received by the IO_Worker
   */
  void Handle_Message(const Battery_Message&& message);

private:

  // std::unordered_map<char, Battery_Callback_Function> m_callback_functions;
  std::multimap<char, Battery_Callback_Function> m_callback_functions;
  
  std::mutex m_callback_mutex;
}; // class Callback_Handler

} // namespace kmd_battery_uart_driver