############################################################################
#
# fkie_potree_rviz_plugin
# Copyright © 2018 Fraunhofer FKIE
# Author: Timo Röhling
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
############################################################################
#
cmake_minimum_required(VERSION 3.1)
project(fkie_potree_rviz_plugin)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###############################################
# Find catkin macros, packages, and libraries #
###############################################

find_package(catkin REQUIRED COMPONENTS
  rviz
)

#######################
# System dependencies #
#######################

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(jsoncpp REQUIRED)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)


############################
# Configure catkin package #
############################

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    rviz
)

#########
# Build #
#########

add_definitions(-DQT_NO_KEYWORDS)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  src/cloud_loader.cpp
  src/cloud_meta_data.cpp
  src/loading_thread.cpp
  src/potree_display.cpp
  src/potree_node.cpp
  src/potree_visual.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  jsoncpp_lib
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

###########
# Install #
###########

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ogre_media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

