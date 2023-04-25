#version 130

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

uniform mat4 worldview_matrix;
uniform mat4 projection_matrix;
uniform float fov;
uniform float viewport_height;
uniform float splat_size;
uniform float spacing;
uniform int is_ortho_projection;
uniform float min_point_size;
uniform float max_point_size;

out vec3 v_position;
out vec3 v_color;
out float v_radius;

void main() {
	vec4 position = worldview_matrix * vertex;
	float scale = length(
		worldview_matrix * vec4(0, 0, 0, 1) - 
		worldview_matrix * vec4(1, 0, 0, 1)
	);
    float projection_factor =
        is_ortho_projection == 0
            ? -0.5 * viewport_height / (tan(fov / 2.0) * position.z)
            : viewport_height * projection_matrix[1][1];
    projection_factor *= scale;
    float point_size = splat_size * spacing;
    if (is_ortho_projection == 0)
        point_size *= projection_factor;
    point_size = max(min_point_size, point_size);
    point_size = min(max_point_size, point_size);
	gl_PointSize = point_size;
	gl_Position = projection_matrix * position;
	v_position = position.xyz;
	v_color = colour.rgb;
	v_radius = point_size / projection_factor;
}
