cmake_minimum_required(VERSION 3.16)
project(agt_mission_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(agt_mission_core
  src/mission_controller.cpp
)
target_include_directories(agt_mission_core PUBLIC include)

enable_testing()

add_executable(test_mission_controller
  test/test_mission_controller.cpp
)
target_link_libraries(test_mission_controller PRIVATE agt_mission_core)

add_test(NAME mission_controller COMMAND test_mission_controller)
