#include "kmd_battery_uart_driver/callback_handler.hpp"

namespace kmd_battery_uart_driver {

void Callback_Handler::Handle_Message(Battery_Message&& message) {
  std::lock_guard<std::mutex> lock(m_callback_mutex);
  auto callback_function = m_callback_functions.find(message.m_data_id);
  if (callback_function != m_callback_functions.end()) {
    callback_function->second(std::move(message));
  }
}

} // namespace kmd_battery_uart_driver