cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(Qt4 4.4.3 COMPONENTS QtCore QtGui REQUIRED )
include(${QT_USE_FILE})
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

MACRO (ROS_QT4_WRAP_UI outfiles)
  QT4_EXTRACT_OPTIONS(ui_files ui_options ${ARGN})
  FOREACH (it ${ui_files})
    GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)
    GET_FILENAME_COMPONENT(infile ${it} ABSOLUTE)
    SET(outfile ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/${outfile}.h) # Here we set output
    ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
      COMMAND ${QT_UIC_EXECUTABLE}
      ARGS ${ui_options} -o ${outfile} ${infile}
      MAIN_DEPENDENCY ${infile})
    SET(${outfiles} ${${outfiles}} ${outfile})
  ENDFOREACH (it)
ENDMACRO (ROS_QT4_WRAP_UI)

SET(qtproject_UIS
  ui/pr2_dynamic_movement_primitive_gui_main_window.ui
)
ROS_QT4_WRAP_UI(qtproject_UIS_H ${qtproject_UIS})

SET(qtproject_MOC_SRCS
  src/pr2_dynamic_movement_primitive_gui.cpp
)
SET(qtproject_MOC_HDRS
  include/${PROJECT_NAME}/pr2_dynamic_movement_primitive_gui.h
)
QT4_WRAP_CPP(qtproject_MOC_SRCS ${qtproject_MOC_HDRS})

rosbuild_add_library(${PROJECT_NAME}
  src/pr2_controller_manager_client.cpp
  ${qtproject_UIS_H}
  ${qtproject_MOC_SRCS}
  ${qtproject_MOC_HDRS}
)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

rosbuild_add_executable(pr2_dynamic_movement_primitive_gui_node
  src/pr2_dynamic_movement_primitive_gui_node.cpp
)
target_link_libraries(pr2_dynamic_movement_primitive_gui_node ${PROJECT_NAME} ${QT_LIBRARIES})









#common commands for building c++ executables and libraries

rosbuild_add_library(${PROJECT_NAME}
  src/pr2_dynamic_movement_primitive_gui.cpp
)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
