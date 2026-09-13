cmake_minimum_required(VERSION 3.8)
project(serialPort)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(Eigen3 REQUIRED)

message("")
message(STATUS "serial_INCLUDE_DIRS: ${serial_INCLUDE_DIRS}")
message(STATUS "serial_LIBRARIES: ${serial_LIBRARIES}")
message("")

ament_auto_find_build_dependencies(REQUIRED)
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/serialPort.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIRS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN serialPort::SerialDriver
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
find_package(serial REQUIRED)
list(APPEND AMENT_LINT_AUTO_EXCLUDE
  ament_cmake_copyright
  ament_cmake_uncrustify
)
target_link_libraries(${PROJECT_NAME} PRIVATE ${serial_LIBRARIES})
ament_lint_auto_find_test_dependencies()
endif()

set(dependencies
  rclcpp
  serial
)

message("")
message(STATUS "Dependencies: ${dependencies}")
message("")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

ament_package()