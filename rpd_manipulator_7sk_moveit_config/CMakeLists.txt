cmake_minimum_required(VERSION 3.22)
project(rpd_manipulator_7sk_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})