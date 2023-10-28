cmake_minimum_required(VERSION 3.8)
project(as2_mc_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  geometry_msgs
  as2_mc_timer_tick
  as2_mc_input
)

message(STATUS "Looking for dependencies")

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
  if(${DEPENDENCY}_FOUND)
    message(STATUS "${DEPENDENCY} was found")
    #include_directories(${ANN_INCLUDE_DIR})
    add_definitions(-D${DEPENDENCY}_WAS_FOUND)
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

install(TARGETS timer_tick
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

install(TARGETS process_input
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