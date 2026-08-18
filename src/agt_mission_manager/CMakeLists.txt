cmake_minimum_required(VERSION 3.16)
project(agt_mission_manager LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT TARGET agt_mission_core)
  add_subdirectory(
    ${CMAKE_CURRENT_LIST_DIR}/../agt_mission_core
    ${CMAKE_CURRENT_BINARY_DIR}/agt_mission_core
  )
endif()

add_library(agt_mission_manager
  src/mission_runtime.cpp
)
target_include_directories(agt_mission_manager PUBLIC include)
target_link_libraries(agt_mission_manager PUBLIC agt_mission_core)

enable_testing()

add_executable(test_adapter_contracts
  test/test_adapter_contracts.cpp
)
target_link_libraries(test_adapter_contracts PRIVATE agt_mission_manager)

add_test(NAME adapter_contracts COMMAND test_adapter_contracts)
