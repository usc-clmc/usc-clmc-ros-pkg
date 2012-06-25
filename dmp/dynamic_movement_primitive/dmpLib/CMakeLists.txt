cmake_minimum_required(VERSION 2.4.6)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_definitions(-Wall -Wextra)
add_definitions(-O0 -g3 -ggdb3 -fno-inline)
set(CMAKE_BUILD_TYPE Debug)

# TODO: fix this
include_directories( "../../locally_weighted_regression/lwrLib/include/" )
link_directories( "../../locally_weighted_regression/lwrLib/lib/" )

add_library(dmp++
  src/dynamic_movement_primitive.cpp
  src/dynamic_movement_primitive_parameters.cpp
  src/dynamic_movement_primitive_state.cpp

  src/transformation_system.cpp
  src/transformation_system_parameters.cpp
  src/transformation_system_state.cpp

  src/canonical_system.cpp
  src/canonical_system_parameters.cpp
  src/canonical_system_state.cpp

  src/trajectory.cpp
  src/logger.cpp

  src/icra2009_dynamic_movement_primitive.cpp
  src/icra2009_dynamic_movement_primitive_parameters.cpp
  src/icra2009_dynamic_movement_primitive_state.cpp

  src/icra2009_transformation_system.cpp
  src/icra2009_transformation_system_parameters.cpp
  src/icra2009_transformation_system_state.cpp

  src/icra2009_canonical_system.cpp
  src/icra2009_canonical_system_parameters.cpp
  src/icra2009_canonical_system_state.cpp

  src/nc2010_dynamic_movement_primitive.cpp
  src/nc2010_dynamic_movement_primitive_parameters.cpp
  src/nc2010_dynamic_movement_primitive_state.cpp

  src/nc2010_transformation_system.cpp
  src/nc2010_transformation_system_parameters.cpp
  src/nc2010_transformation_system_state.cpp

  src/nc2010_canonical_system.cpp
  src/nc2010_canonical_system_parameters.cpp
  src/nc2010_canonical_system_state.cpp
)

# link against liblwr.a
target_link_libraries(dmp++ lwr)

find_package(Boost COMPONENTS system filesystem REQUIRED)
target_link_libraries(dmp++ 
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)

add_executable(test/dmp_test
  test/test_dynamic_movement_primitive.cpp
  test/test_trajectory.cpp
  test/test_data.cpp
  test/icra2009_test.cpp
)
target_link_libraries(test/dmp_test dmp++)
