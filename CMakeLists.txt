cmake_minimum_required(VERSION 3.15)
project(animation_project)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_STANDARD 17)

set(GENGINE_ENABLE_PHYSX ON BOOL)
add_subdirectory(gengine)

add_executable(animation_project main.cpp)
target_link_libraries(animation_project gengine)
add_dependencies(animation_project copy_resource_folder)

add_executable(data_driven_anim data-driven.cpp Box.cpp)
target_link_libraries(data_driven_anim gengine)
add_dependencies(data_driven_anim copy_resource_folder)

add_executable(stair_anim stair.cpp Box.cpp)
target_link_libraries(stair_anim gengine)
add_dependencies(stair_anim copy_resource_folder)

add_executable(jump_anim jump.cpp Box.cpp)
target_link_libraries(jump_anim gengine)
add_dependencies(jump_anim copy_resource_folder)

add_executable(physics_test physics.cpp)
target_link_libraries(physics_test gengine)
add_dependencies(physics_test copy_resource_folder)