cmake_minimum_required(VERSION 3.5)
project(rosbot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Uncomment if the package has a setup.py
# ament_python_install_package(${PROJECT_NAME})

# Generate messages in the 'msg' folder
# Uncomment and modify if you have custom messages
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Message1.msg"
#   "msg/Message2.msg"
# )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
# install(PROGRAMS
#   scripts/my_python_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install other files
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
