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

// Generates view-facing square (billboard) with a given size
// for each input vertex.

uniform mat4 worldviewproj_matrix;
uniform mat4 inverse_worldview_matrix;
uniform vec4 size;

layout(points) in;
layout(triangle_strip, max_vertices=4) out;

// These values come from the vertex shader
// Technically, it's an array, but since the input layout is points,
// the array contains just one element.
in vec4 v_position[];
in vec4 v_color[];

// Pass these values to the fragment shader
out vec4 frag_color;
out vec4 tex_coord;


void emitVertex( vec3 pos_rel, vec3 tex )
{
  pos_rel = mat3(inverse_worldview_matrix) * pos_rel;
  vec4 pos = v_position[0] + vec4(pos_rel,0.0);
  gl_Position = worldviewproj_matrix * pos;
  tex_coord = vec4( tex.xy, 0.0, 0.0 );
  frag_color = v_color[0];
  EmitVertex();
}

void main() 
{
  vec4 screen_pos = worldviewproj_matrix * v_position[0];
  float size_factor = 0.5 * size.x;
  if (size.y > 0.5) size_factor *= screen_pos.z; else size_factor /= size.z;
  emitVertex( vec3(-size_factor,-size_factor, 0.0), vec3(-1.0, -1.0, 0.0) );
  emitVertex( vec3( size_factor,-size_factor, 0.0), vec3(1.0, -1.0, 0.0) );
  emitVertex( vec3(-size_factor, size_factor, 0.0), vec3(-1.0, 1.0, 0.0) );
  emitVertex( vec3( size_factor, size_factor, 0.0), vec3(1.0, 1.0, 0.0) );
  EndPrimitive();
}
