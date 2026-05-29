cmake_minimum_required(VERSION 3.5)
project(g1_arm_action_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

# ============================================================
# Unitree SDK2 Configuration
# ============================================================
# Set the path to unitree_sdk2 - adjust this path as needed
set(UNITREE_SDK2_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_sdk2" CACHE PATH "Path to unitree_sdk2")

# Check if the path exists
if(NOT EXISTS "${UNITREE_SDK2_PATH}")
  message(FATAL_ERROR "Unitree SDK2 path not found: ${UNITREE_SDK2_PATH}\n"
                      "Please set UNITREE_SDK2_PATH to the correct location.")
endif()

message(STATUS "Unitree SDK2 path: ${UNITREE_SDK2_PATH}")
message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Find Unitree SDK2 library
set(UNITREE_SDK_LIB_PATH ${UNITREE_SDK2_PATH}/lib/${CMAKE_SYSTEM_PROCESSOR})
find_library(UNITREE_SDK_LIB unitree_sdk2 PATHS ${UNITREE_SDK_LIB_PATH} NO_DEFAULT_PATH)

if(NOT UNITREE_SDK_LIB)
  message(FATAL_ERROR "Unitree SDK2 library not found in: ${UNITREE_SDK_LIB_PATH}")
else()
  message(STATUS "Found Unitree SDK2 library: ${UNITREE_SDK_LIB}")
endif()

# ============================================================
# CycloneDDS Configuration (from unitree_sdk2/thirdparty)
# ============================================================
set(DDS_THIRDPARTY_PATH ${UNITREE_SDK2_PATH}/thirdparty)
set(DDS_LIB_PATH ${DDS_THIRDPARTY_PATH}/lib/${CMAKE_SYSTEM_PROCESSOR})
set(DDS_INCLUDE_PATH ${DDS_THIRDPARTY_PATH}/include)

# Find DDS libraries
find_library(DDSC_LIB ddsc PATHS ${DDS_LIB_PATH} NO_DEFAULT_PATH)
find_library(DDSCXX_LIB ddscxx PATHS ${DDS_LIB_PATH} NO_DEFAULT_PATH)

if(NOT DDSC_LIB OR NOT DDSCXX_LIB)
  message(FATAL_ERROR "CycloneDDS libraries not found in: ${DDS_LIB_PATH}\n"
                      "DDSC_LIB: ${DDSC_LIB}\n"
                      "DDSCXX_LIB: ${DDSCXX_LIB}")
else()
  message(STATUS "Found CycloneDDS libraries:")
  message(STATUS "  ddsc: ${DDSC_LIB}")
  message(STATUS "  ddscxx: ${DDSCXX_LIB}")
endif()

# ============================================================
# Generate service interfaces
# ============================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteArmAction.srv"
  "srv/GetArmActionList.srv"
)

# ============================================================
# Build the arm action node
# ============================================================
add_executable(g1_arm_action_node
  src/g1_arm_action_node.cpp
)

target_include_directories(g1_arm_action_node PRIVATE
  ${UNITREE_SDK2_PATH}/include
  ${DDS_INCLUDE_PATH}
  ${DDS_INCLUDE_PATH}/ddscxx
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(g1_arm_action_node
  ${UNITREE_SDK_LIB}
  ${DDSC_LIB}
  ${DDSCXX_LIB}
  Threads::Threads
)

# Add rpath for runtime library loading
set_target_properties(g1_arm_action_node PROPERTIES
  BUILD_RPATH "${DDS_LIB_PATH};${UNITREE_SDK_LIB_PATH}"
  INSTALL_RPATH "${DDS_LIB_PATH};${UNITREE_SDK_LIB_PATH}"
)

ament_target_dependencies(g1_arm_action_node
  rclcpp
  std_msgs
)

# Link generated interfaces
rosidl_target_interfaces(g1_arm_action_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# ============================================================
# Install
# ============================================================
install(TARGETS
  g1_arm_action_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
