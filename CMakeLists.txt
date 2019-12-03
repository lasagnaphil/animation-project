cmake_minimum_required(VERSION 3.15)
project(animation_project)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(gengine)

add_executable(animation_project main.cpp)
target_include_directories(animation_project PUBLIC gengine)
target_link_libraries(animation_project gengine)
add_dependencies(animation_project copy_resource_folder)

add_executable(data_driven_anim data-driven.cpp)
target_include_directories(data_driven_anim PUBLIC gengine)
target_link_libraries(data_driven_anim gengine)
add_dependencies(data_driven_anim copy_resource_folder)