############################################################################
#
# fkie_potree_rviz_plugin
# Copyright © 2018-2023 Fraunhofer FKIE
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

cmake_minimum_required(VERSION 3.13...3.28)
project(fkie_potree_rviz_plugin VERSION 3.0.0)

include(GenerateExportHeader)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)


# Targets
add_library(${PROJECT_NAME} MODULE
    src/cloud_loader.cpp
    src/cloud_loader_1.cpp
    src/cloud_loader_2.cpp
    src/cloud_loader_streaming.cpp
    src/cloud_meta_data.cpp
    src/fs_path_editor.cpp
    src/fs_path_property.cpp
    src/loading_thread.cpp
    src/potree_display.cpp
    src/potree_node.cpp
    src/potree_visual.cpp
)
generate_export_header(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_compile_definitions(${PROJECT_NAME} PRIVATE QT_NO_KEYWORDS)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
    sensor_msgs
    geometry_msgs
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_VISIBILITY_PRESET "hidden"
    VISIBILITY_INLINES_HIDDEN ON
    AUTOMOC ON
)
target_link_libraries(${PROJECT_NAME}
    jsoncpp_lib
    pluginlib::pluginlib
    rviz_common::rviz_common
    rviz_rendering::rviz_rendering
    rclcpp::rclcpp
)


# Installation
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY icons
    DESTINATION share/${PROJECT_NAME}
)

# Ament package handling
register_rviz_ogre_media_exports(
    DIRECTORIES "materials"
)
pluginlib_export_plugin_description_file(rviz_common plugin.xml)
ament_package()
