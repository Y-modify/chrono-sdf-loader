cmake_minimum_required(VERSION 3.1)

add_library(sdf-loader SHARED sdf-loader.cpp)

set_target_properties(sdf-loader PROPERTIES
	    COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	    LINK_FLAGS "${CHRONO_LINKER_FLAGS}")

target_link_libraries(sdf-loader ${CHRONO_LIBRARIES} ${SDFormat_LIBRARIES})

install(TARGETS sdf-loader DESTINATION lib)
