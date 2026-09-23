cmake_minimum_required(VERSION 3.8)
project(aibomech_agrobot_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_manager.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
