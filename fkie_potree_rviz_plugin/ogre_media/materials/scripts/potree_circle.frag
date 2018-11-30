#version 150
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

// rasterizes a little camera-facing cone that looks like a shaded circle
// and merges nicely with surrounding points in dense point clouds

uniform vec4 shading;

// These attributes are passed from the geometry shader
in vec4 frag_color;
in vec4 tex_coord;

// The output variable will automatically become the fragment's color
out vec4 pixel_color;

// Help the renderer to avoid shader invocations for fragments which will be
// discarded anyway
layout(depth_greater) out float gl_FragDepth;

void main()
{
  float ax = tex_coord.x;
  float ay = tex_coord.y;

  float rsquared = ax*ax+ay*ay;
  if (rsquared >= 1)
  {
    discard;
  }
  pixel_color = vec4(frag_color.xyz * (1 - shading.x * rsquared), 1);
  gl_FragDepth = gl_FragCoord.z + sqrt(rsquared) * 0.001 * gl_FragCoord.w;
}
