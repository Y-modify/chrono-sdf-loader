cmake_minimum_required(VERSION 3.1)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(chrono-sdf-loader)

find_package(Chrono
             COMPONENTS Irrlicht
             CONFIG)
find_package(SDFormat REQUIRED)
find_package(Boost REQUIRED)


if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()


include_directories(${CHRONO_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${SDFormat_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${SDFormat_LIBRARY_DIRS})

include(cmake/clang-format.cmake)

add_subdirectory(lib/)
add_subdirectory(include/)

add_subdirectory(examples/)
