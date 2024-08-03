cmake_minimum_required(VERSION 3.10)
project(mujoco)

set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_SOURCE_DIR}/include ~/.mujoco/mujoco-3.2.0/include)
link_directories(~/.mujoco/mujoco-3.2.0/bin)

find_package(glfw3 REQUIRED)

set(SOURCE_FILE "main.cpp" CACHE STRING "Source file to compile")

add_executable(mujoco ${SOURCE_FILE})
target_link_libraries(mujoco glfw ~/.mujoco/mujoco-3.2.0/lib/libmujoco.so GL)