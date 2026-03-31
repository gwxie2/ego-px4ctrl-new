cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  quadrotor_msgs
  roscpp
  rviz
  tf
)

catkin_package()

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  qt4_wrap_cpp(MOC_FILES
    src/goal_tool.h
  )
else()
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  qt5_wrap_cpp(MOC_FILES
    src/goal_tool.h
  )
endif()

add_definitions(-DQT_NO_KEYWORDS)

set(SOURCE_FILES
  src/pose_tool.cpp
  src/goal_tool.cpp
  ${MOC_FILES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)