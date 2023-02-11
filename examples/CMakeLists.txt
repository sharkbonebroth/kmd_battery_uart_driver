add_executable(demo_subscribe_information 
  ${CMAKE_CURRENT_SOURCE_DIR} demo_subscribe_information.cpp
)

target_link_libraries(demo_subscribe_information
PRIVATE
  dash::kmd_battery_uart_driver
)
