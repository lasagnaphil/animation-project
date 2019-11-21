cmake_minimum_required(VERSION 3.15)
project(animation_project)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(gengine)

find_package(Threads REQUIRED)
find_package(PhysX REQUIRED)

add_executable(animation_project main.cpp PhysicsWorld.cpp)
target_include_directories(animation_project PUBLIC gengine ${PhysX_INCLUDE_DIR} ${PxShared_INCLUDE_DIR})
target_link_libraries(animation_project gengine)
target_link_libraries(animation_project debug ${PhysX_LIBRARIES_DEBUG})
target_link_libraries(animation_project optimized ${PhysX_LIBRARIES_RELEASE})
add_dependencies(animation_project copy_resource_folder)
