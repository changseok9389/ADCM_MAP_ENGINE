cmake_minimum_required(VERSION 3.20)
project(ADCM_MAP_ENGINE)

set(CMAKE_CXX_STANDARD 17)

find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ADCM_MAP_ENGINE main.cpp map_parser.h map_parser.cpp road_network.pb.h road_network.pb.cc )
TARGET_LINK_LIBRARIES (ADCM_MAP_ENGINE ${PROTOBUF_LIBRARIES} ${OpenCV_LIBS})

