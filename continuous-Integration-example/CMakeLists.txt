cmake_minimum_required(VERSION 3.10)
project(ci-example)

# Use C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
elseif(MSVC)
  add_compile_options(-D_SILENCE_CXX17_OLD_ALLOCATOR_MEMBERS_DEPRECATION_WARNING )
  add_compile_options(-D_USE_MATH_DEFINES )
endif()


add_executable(hello_world hello_world.cpp)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../ext/fmt ${CMAKE_CURRENT_BINARY_DIR}/ext/fmt)
target_include_directories(hello_world PRIVATE fmt::fmt-header-only)
target_link_libraries(hello_world fmt::fmt-header-only)

get_filename_component(PARENT_DIR ${PROJECT_SOURCE_DIR} PATH)

ament_target_dependencies(hello_world rclcpp)

install(TARGETS hello_world DESTINATION lib/${PROJECT_NAME})

ament_package()
