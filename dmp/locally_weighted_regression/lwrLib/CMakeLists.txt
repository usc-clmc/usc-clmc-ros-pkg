cmake_minimum_required(VERSION 2.4.6)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_definitions(-Wall -Wextra)
add_definitions(-O0 -g3 -ggdb3 -fno-inline)
set(CMAKE_BUILD_TYPE Debug)

include_directories( "include/" )

add_library(lwr
	src/lwr.cpp
	src/lwr_parameters.cpp
	src/logger.cpp
)

add_executable(test/lwr_test
  test/test_locally_weigthed_regression.cpp
)
target_link_libraries(test/lwr_test lwr)

# set( LIB_INSTALL_DIR "${LAB_ROOT}/lib/${MACHTYPE}" CACHE PATH	"Path the built library files are installed to." )
# set( INCLUDE_INSTALL_DIR "${LAB_ROOT}/include/lwr" CACHE PATH	"Path the header files are installed to." )

#set( LIB_INSTALL_DIR )
#set( INCLUDE_INSTALL_DIR )

#set( HEADER_PATH include )
#set( PUBLIC_HEADERS
#     ${HEADER_PATH}/lwr/locally_weighted_regression.h
#     ${HEADER_PATH}/lwr/lwr_parameters.h
#)

#install( TARGETS lwr DESTINATION ${LIB_INSTALL_DIR} )
#install( FILES ${PUBLIC_HEADERS} DESTINATION ${INCLUDE_INSTALL_DIR} )
