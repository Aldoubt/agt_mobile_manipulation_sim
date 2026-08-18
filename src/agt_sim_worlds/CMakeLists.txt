cmake_minimum_required(VERSION 3.8)
project(agt_sim_worlds)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds profiles
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
