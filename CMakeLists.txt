cmake_minimum_required(VERSION 3.15)

project(kmd_battery_uart_driver VERSION 1.0)

# Remove this when adding to dash code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Logging things
find_package(spdlog REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/libserial 3rd_party/libserial)

add_library(kmd_battery_uart_driver 
  src/kmd_battery_uart_driver/kmd_battery_uart_driver.cpp
  src/kmd_battery_uart_driver/io_worker.cpp
  src/kmd_battery_uart_driver/callback_handler.cpp
)

add_library(dash::kmd_battery_uart_driver ALIAS kmd_battery_uart_driver)

target_include_directories(kmd_battery_uart_driver
PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(kmd_battery_uart_driver
PUBLIC
  dash3rdparty::libserial
  spdlog::spdlog
)

set_target_properties(kmd_battery_uart_driver
PROPERTIES
  CXX_STANDARD 17
)

if (${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
  add_subdirectory(${CMAKE_SOURCE_DIR}/examples)
endif()