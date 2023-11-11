cmake_minimum_required(VERSION 3.8)
project(as2_mc_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Winvalid-pch)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  std_srvs
  geometry_msgs
  as2_mc_timer_tick
  as2_mc_input
  as2_mc_output
)

message(STATUS "Looking for dependencies")

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
  if(${DEPENDENCY}_FOUND)
    message(STATUS "${DEPENDENCY} was found")
  else()
    message(STATUS "${DEPENDENCY} was not found")
  endif()  
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES_TIMER_TICK 
  src/timer_tick.cpp
)

add_executable(timer_tick ${SOURCE_CPP_FILES_TIMER_TICK})
target_include_directories(timer_tick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(timer_tick PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(timer_tick ${PROJECT_DEPENDENCIES})

target_precompile_headers(timer_tick
  PRIVATE
    [["rclcpp/rclcpp.hpp"]]
)

install(TARGETS timer_tick
  DESTINATION lib/${PROJECT_NAME})

set(SOURCE_CPP_FILES_TIMER_TICK_ABST
  src/timer_tick_with_abstract.cpp
)

add_executable(timer_tick_abs ${SOURCE_CPP_FILES_TIMER_TICK_ABST})
target_include_directories(timer_tick_abs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(timer_tick_abs PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(timer_tick_abs ${PROJECT_DEPENDENCIES})
target_precompile_headers(timer_tick_abs REUSE_FROM timer_tick)

install(TARGETS timer_tick_abs
  DESTINATION lib/${PROJECT_NAME})

set(SOURCE_CPP_FILES_INPUT 
  src/process_input.cpp
)

add_executable(process_input ${SOURCE_CPP_FILES_INPUT})
target_include_directories(process_input PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(process_input PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(process_input ${PROJECT_DEPENDENCIES})
target_precompile_headers(process_input REUSE_FROM timer_tick)

install(TARGETS process_input
  DESTINATION lib/${PROJECT_NAME})

set(SOURCE_CPP_FILES_BOOL_SERVICE
  src/bool_service.cpp
)

add_executable(bool_service ${SOURCE_CPP_FILES_BOOL_SERVICE})
target_include_directories(bool_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bool_service PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(bool_service ${PROJECT_DEPENDENCIES})
target_precompile_headers(bool_service REUSE_FROM timer_tick)

install(TARGETS bool_service
  DESTINATION lib/${PROJECT_NAME})

set(SOURCE_CPP_FILES_BOOL_CLIENT
  src/bool_client.cpp
)

add_executable(bool_client ${SOURCE_CPP_FILES_BOOL_CLIENT})
target_include_directories(bool_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bool_client PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(bool_client ${PROJECT_DEPENDENCIES})
target_precompile_headers(bool_client REUSE_FROM timer_tick)

install(TARGETS bool_client
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
