cmake_minimum_required(VERSION 3.1)

include_directories(${PROJECT_SOURCE_DIR})
add_definitions(-DCHRONO_DATA_DIR="${CHRONO_DATA_DIR}")

add_executable(loader loader.cpp)
target_link_libraries(loader sdf-loader ${CHRONO_LIBRARIES} ${SDFormat_LIBRARIES})
