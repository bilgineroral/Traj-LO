cmake_minimum_required(VERSION 3.10)

add_subdirectory(ros EXCLUDE_FROM_ALL)
add_subdirectory(yaml-cpp)
add_subdirectory(eigen)

add_subdirectory(robin-map)

option(TBB_TEST "Enable testing" OFF)
add_subdirectory(oneTBB)

