cmake_minimum_required(VERSION 3.8)
project(xycar_final_drive)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
