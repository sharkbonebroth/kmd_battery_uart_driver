#include "kmd_battery_uart_driver/callback_handler.hpp"

namespace kmd_battery_uart_driver {

void Callback_Handler::Handle_Message(const Battery_Message&& message) {
  std::lock_guard<std::mutex> lock(m_callback_mutex);
  auto callback_function = m_callback_functions.find(message.m_data_id);
  for (
    auto callback_function_iterator = m_callback_functions.lower_bound(message.m_data_id); 
    callback_function_iterator != m_callback_functions.upper_bound(message.m_data_id);
    callback_function_iterator++
  ) {
    callback_function->second(message);
  }
}

} // namespace kmd_battery_uart_driver