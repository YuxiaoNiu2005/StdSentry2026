cmake_minimum_required(VERSION 3.20)
project(auto_aim_interfaces)

# Enable compile_commands.json generation for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/GimbalCmd.msg"
  "msg/SerialReciveData.msg"
  "msg/ArmorChoose.msg"
  
  "srv/SetMode.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)


ament_package()
