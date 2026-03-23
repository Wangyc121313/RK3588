cmake_minimum_required(VERSION 3.16)

project(rk3588_perception LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(cmake/targets/base.cmake)
include(cmake/targets/mpp_rga_rknn.cmake)
include(cmake/targets/lidar.cmake)