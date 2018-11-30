#version 150

/****************************************************************************
 *
 * fkie_potree_rviz_plugin
 * Copyright © 2018 Fraunhofer FKIE
 * Author: Timo Röhling
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

// These variables are automatically bound by Ogre
in vec4 vertex;
in vec4 colour;

// We pass these variables to the geometry shader
out vec4 v_position;
out vec4 v_color;

void main() {
    v_position = vertex;
    v_color = colour;
}
