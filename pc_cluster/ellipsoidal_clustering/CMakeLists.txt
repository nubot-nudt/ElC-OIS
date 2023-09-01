cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)
project(ellipsoidalClustering)

add_subdirectory(third_party/pybind11)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(TARGET ellipsoidalClustering)

pybind11_add_module(${TARGET} main.cpp)

target_include_directories(${TARGET} PRIVATE
  ${PROJECT_SOURCE_DIR}
)

target_link_libraries(${TARGET} PRIVATE
  ${PCL_LIBRARIES} ${Boost_INCLUDE_DIRS}
)
