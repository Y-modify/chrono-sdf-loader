cmake_minimum_required(VERSION 3.1)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(chrono-sdf-loader)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-std=c++14 -pedantic-errors -Wall -Werror -Weverything -Wno-float-equal -Wno-padded -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-error=unused-parameter -Wno-error=global-constructors -Wno-error=exit-time-destructors)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-std=c++14 -pedantic-errors -Wall -Wextra -Werror -Wno-float-equal -Wno-padded -Wno-unused-parameter -Wcast-align -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 -Winit-self -Wlogical-op -Wmissing-declarations -Wmissing-include-dirs -Wnoexcept -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wsign-conversion -Wsign-promo -Wstrict-null-sentinel -Wstrict-overflow=5 -Wswitch-default -Wundef -Wno-error=unused)
endif()


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
