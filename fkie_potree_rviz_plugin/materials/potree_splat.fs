#version 130
#extension GL_ARB_conservative_depth : enable

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

uniform mat4 projection_matrix;

in vec3 v_position;
in vec3 v_color;
in float v_radius;

// Output color
out vec4 pixel_color;
#ifdef GL_ARB_conservative_depth
layout(depth_less) out float gl_FragDepth;
#endif

void main()
{
    float u = 2.0 * gl_PointCoord.x - 1.0;
    float v = 2.0 * gl_PointCoord.y - 1.0;
	float wi = 0.0 - ( u*u + v*v);
	if (wi < -1.0)
		discard;
	vec4 pos = vec4(v_position, 1.0);
	pos.z += wi * v_radius;
	pos = projection_matrix * pos;
	pos = pos / pos.w;
    gl_FragDepth = pos.z;

	pixel_color.a = 1.0;
	pixel_color.rgb = v_color;
}
