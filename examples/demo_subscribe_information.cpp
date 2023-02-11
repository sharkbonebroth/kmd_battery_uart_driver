#include "kmd_battery_uart_driver/kmd_battery_uart_driver.hpp"

void Cell_Voltage_Callback(const kmd_battery_uart_driver::Battery_Message& message) {
  for (int i = 0; i < 3; i++) {
    unsigned short cell_voltage = message.m_data[1 + (2 * i)];

    spdlog::info(
      "Cell {0:d} voltage (mV): {0:d}",
      -3 + (3 * message.m_data[0]) + (i + 1),
      cell_voltage
    );
  }
}

int main(int argc, char** argv) {
  if (argc < 2) {
    spdlog::error("Usage: kmd_battery_uart_driver {device}");
    return 1;
  }

  // Create the main Kmd_Battery_Uart_Driver object that interfaces with the battery. We Assume that the pc_address is 0x40
  kmd_battery_uart_driver::Kmd_Battery_Uart_Driver battery(argv[1], 0x40); 
  battery.Enable_Debug_Messages();
  battery.Initialize_Serial();

  battery.Subscribe(0x95, Cell_Voltage_Callback, 10);

  // Wait for 50 seconds to show that messages are being received
  std::chrono::milliseconds duration(50000);
  std::this_thread::sleep_for(duration);
}