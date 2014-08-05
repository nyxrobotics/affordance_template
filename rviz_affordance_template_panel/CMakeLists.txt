cmake_minimum_required(VERSION 2.8.3)
project(rviz_affordance_template_panel)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# find ZMQ
find_package(ZeroMQ REQUIRED)
include_directories(${ZeroMQ_INCLUDE_DIR})

# add protobuf include
find_package(affordance_template_server_protobuf)
include_directories(${affordance_template_server_protobuf_DIR}/proto_gen/cpp)
# include_directories(${rviz_affordance_template_panel_DIR})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

set(QT_FORMS ${PROJECT_SOURCE_DIR}/ui/rviz_affordance_template_panel.ui)
set(QT_RESS ${PROJECT_SOURCE_DIR}/resources/images.qrc)

qt4_wrap_cpp(MOC_FILES
    src/rviz_affordance_template_panel.hpp
    src/Affordance.hpp
    src/RecognitionObject.hpp
    src/RobotConfig.hpp
)
qt4_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt4_add_resources(QT_RESOURCES ${QT_RESS})

set(SRC_FILES
    src/rviz_affordance_template_panel.cpp
    src/Affordance.cpp
    src/RecognitionObject.cpp
    ${MOC_FILES}
)

# everything else
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  roscpp
  rviz
  affordance_template_server
  affordance_template_server_protobuf
  visualization_msgs
)

find_package(orocos_kdl REQUIRED)
link_directories(${orocos_kdl_LIBRARY_DIRS})
find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES rviz_affordance_template_panel ${KDL_LIBRARY}
  CATKIN_DEPENDS geometry_msgs interactive_markers roscpp roslib rviz affordance_template_server affordance_template_server_protobuf visualization_msgs
  DEPENDS system_lib orocos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SRC_FILES} ${QT_FORMS_HPP} ${QT_RESOURCES})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${ZeroMQ_LIBRARIES} ${PROTOBUF_LIBRARY} ${orocos_kdl_LIBRARIES} AffordanceTemplateServerCmd)

