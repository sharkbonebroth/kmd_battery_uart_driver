#include "kmd_battery_uart_driver/kmd_battery_uart_driver.hpp"

namespace kmd_battery_uart_driver {

Kmd_Battery_Uart_Driver::~Kmd_Battery_Uart_Driver() {
  Close();
  spdlog::info("kmd_battery_uart_driver: Kmd_Battery_Uart_Driver: Shuttng down driver");
}

bool Kmd_Battery_Uart_Driver::Initialize_Serial() {
  if (!m_io_worker.Initialize_Serial(m_device)) {
    spdlog::info("kmd_battery_uart_driver: Kmd_Battery_Uart_Driver: IO_Worker failed to initialize serial connection");
    return false;
  }

  m_running.store(true);
  m_connected.store(true);
  m_ping_battery_thread = std::thread([this](){Ping_Battery();});

  for (std::pair<char, unsigned int> subscribed: m_subscribed) {
    Periodically_Request_For_Information(subscribed.first, subscribed.second);
  }

  return true;
}

void Kmd_Battery_Uart_Driver::Close() {
  m_running.store(false);
  if (m_ping_battery_thread.joinable()) {
    m_ping_battery_thread.join();
    spdlog::info("kmd_battery_uart_driver: Kmd_Battery_Uart_Driver: Stopped testing for connection");
  }

  for (std::shared_ptr<std::thread> information_request_thread_ptr: m_request_for_information_threads) {
    if (information_request_thread_ptr->joinable()) {
      information_request_thread_ptr->join();
    }
  }

  m_request_for_information_threads.clear();

  m_io_worker.Close();
}

void Kmd_Battery_Uart_Driver::Subscribe(const char data_id, const Callback_Handler::Battery_Callback_Function battery_callback_function, const unsigned int rate_hz) {
  m_subscribed.push_back({data_id, rate_hz});
  m_io_worker.Insert_Callback(data_id, std::move(battery_callback_function));
  Periodically_Request_For_Information(data_id, rate_hz);
}

bool Kmd_Battery_Uart_Driver::Send_Message_Wait_Response(Battery_Message& battery_message, const unsigned int timeout_milliseconds) {
  std::shared_ptr<Battery_Message> battery_message_ptr = std::make_shared<Battery_Message>(battery_message);
  std::shared_ptr<std::atomic<bool>> received_ptr = std::make_shared<std::atomic<bool>>();
  received_ptr->store(false);

  auto callback_ptr = m_io_worker.Insert_Callback(battery_message.m_data_id, [this, battery_message_ptr, received_ptr](const Battery_Message& message){
    std::memcpy(battery_message_ptr->m_data.data(), message.m_data.data(), 8);
    received_ptr->store(true);
  });

  if (!m_io_worker.Send_Message(battery_message)) {return false;}

  std::chrono::time_point<std::chrono::steady_clock> wait_until_time = std::chrono::steady_clock::now();
  std::chrono::milliseconds duration(timeout_milliseconds);
  wait_until_time += duration;

  while (std::chrono::steady_clock::now() < wait_until_time) {
    m_io_worker.Wait(timeout_milliseconds);

    if (received_ptr->load()) {break;}
  }

  m_io_worker.Remove_Callback(callback_ptr);

  return received_ptr->load();
}

void Kmd_Battery_Uart_Driver::Periodically_Request_For_Information(const char data_id, const unsigned int rate_hz) {
  std::shared_ptr<std::thread> information_request_thread = std::make_shared<std::thread>(
    [this, rate_hz, data_id](){
      int sleep_time_milliseconds = 1000 / rate_hz;
      while (m_running.load() && m_connected.load()) {
        Battery_Message message;
        message.m_pc_address = m_pc_address;
        message.m_data_id = data_id;
        Send_Message(message);

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_milliseconds));
      }
    }
  );

  m_request_for_information_threads.push_back(information_request_thread);
}

void Kmd_Battery_Uart_Driver::Ping_Battery() {
  while (m_running.load()) {
    Battery_Message request_soc_message;
    request_soc_message.m_pc_address = m_pc_address;
    request_soc_message.m_data_id = 0x90;
    if (!Send_Message_Wait_Response(request_soc_message, 1000)) {
      // Attempt to close and reopen connection
      spdlog::error("kmd_battery_uart_driver: Kmd_Battery_Uart_Driver: Ping_Battery failed. Attempting a reconnection...");

      m_connected.store(false);

      for (std::shared_ptr<std::thread> information_request_thread_ptr: m_request_for_information_threads) {
        if (information_request_thread_ptr->joinable()) {
          information_request_thread_ptr->join();
        }
      }

      m_request_for_information_threads.clear();

      m_io_worker.Close();

      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      if (m_io_worker.Initialize_Serial(m_device)) {
        spdlog::info("kmd_battery_uart_driver: Kmd_Battery_Uart_Driver: Successfully reestablished serial connection");

        m_connected.store(true);

        for (std::pair<char, unsigned int> subscribed: m_subscribed) {
          Periodically_Request_For_Information(subscribed.first, subscribed.second);
        }
      } else {
        spdlog::error("kmd_battery_uart_driver: Kmd_Battery_Uart_Driver: Reconnection attempt failed");
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

} // namespace kmd_battery_uart_driver