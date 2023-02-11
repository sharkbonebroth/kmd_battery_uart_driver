#pragma once

#include "kmd_battery_uart_driver/io_worker.hpp"

namespace kmd_battery_uart_driver {

/**
 * @brief Kmd_Battery_Uart_Driver class provides an interface for interacting with the kamada battery
 */
class Kmd_Battery_Uart_Driver {

public:

  /**
   * @brief Construct a new Kmd_Battery_Uart_Driver object
   * 
   * @param device The USB device to connect to
   * @param pc_address The pc_address of the battery to connect to
   */
  Kmd_Battery_Uart_Driver(std::string device, const char pc_address) {
    m_device = device;
    m_pc_address = pc_address;
  };

  ~Kmd_Battery_Uart_Driver();

  /**
   * @brief Initializes the serial connection with the battery, starts the m_ping_battery_thread, and the threads that request information from the battery 
   * 
   * @return true 
   * @return false 
   */
  bool Initialize_Serial();

  /**
   * @brief Closes the serial connection with the battery and stops both the m_ping_battery_thread and the threads that request information from the battery 
   */
  void Close();

  bool Is_Connected() {return m_connected.load();}

  /**
   * @brief Subscribes to a message of a certain data_id and adds a callback function to be called upon the recipt of it
   * 
   * @param data_id ID of the messages to call the callback function on
   * @param battery_callback_function The callback function to be called
   * @param rate_hz The frequency of requesting the message
   */
  void Subscribe(const char data_id, const Callback_Handler::Battery_Callback_Function battery_callback_function, unsigned int rate_hz);

  /**
   * @brief Sends a message to the battery
   * 
   * @param battery_message A Battery_Message type containing the data to be sent
   * @return true if message was successfully sent
   * @return false otherwise
   */
  bool Send_Message(const Battery_Message& battery_message) {return m_io_worker.Send_Message(battery_message);};

  /**
   * @brief Sends a message to the battery, and waits for a response. The response would be written to the battery_message passed
   * to this function
   * 
   * @param battery_message 
   * @param timeout_milliseconds Timeout in milliseconds
   * @return true if the message was successfully sent and a response was received
   * @return false otherwise
   */
  bool Send_Message_Wait_Response(Battery_Message& battery_message, const unsigned int timeout_milliseconds);

  /**
   * @brief Enable the printing of IO worker debug messages
   */
  void Enable_Debug_Messages() {m_io_worker.Enable_Debug_Messages();}

private:

  /**
   * @brief Sends a message to the battery requesting for SOC of total voltage current, and waits checks for a reply every second.
   * A response would indicate that the battery is still connected. If no response is received, the driver would attempt to reconnect
   * to the battery every second
   */
  void Ping_Battery();

  /**
   * @brief Starts a thread that periodically requests information from the battery
   * 
   * @param data_id The data id of the information to be requested from the battery
   * @param rate_hz The rate of requests, in hz
   */
  void Periodically_Request_For_Information(const char data_id, const unsigned int rate_hz);
  
private:

  IO_Worker m_io_worker;

  std::string m_device;
  char m_pc_address;

  std::atomic<bool> m_connected{false};

  std::atomic<bool> m_running{false};

  std::thread m_ping_battery_thread; // Thread that tests connection to battery by pinging the battery and checking if a response is received
  
  std::vector<std::pair<char, unsigned int>> m_subscribed;
  std::vector<std::shared_ptr<std::thread>> m_request_for_information_threads; // A vector of threads that periodically request for information from the battery

}; // class Kmd_Battery_Uart_Driver

} // namespace kmd_battery_uart_driver