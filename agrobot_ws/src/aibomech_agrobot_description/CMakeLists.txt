cmake_minimum_required(VERSION 3.8)
project(aibomech_agrobot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY cad config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py TIMEOUT 120)
endif()

ament_package()
