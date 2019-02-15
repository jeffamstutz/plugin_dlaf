// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

// ospcommon
#include "ospcommon/containers/AlignedVector.h"
#include "ospcommon/vec.h"

namespace dlaf {
  using namespace ospcommon;

  struct DLAF_Params
  {
    int NumParticles = 100000;
    // ParticleSpacing defines the distance between particles that are
    // joined together
    float ParticleSpacing = 1;
    // AttractionDistance defines how close together particles must be in
    // order to join together
    float AttractionDistance = 3;
    // MinMoveDistance defines the minimum distance that a particle will move
    // during its random walk
    float MinMoveDistance = 1;
    // Stubbornness defines how many interactions must occur before a
    // particle will allow another particle to join to it.
    int Stubbornness = 0;
    // Stickiness defines the probability that a particle will allow another
    // particle to join to it.
    float Stickiness = 1;
    // BoundingRadius defines the radius of the bounding sphere that bounds
    // all of the particles
    float BoundingRadius = 0;
    // color gradients per-sphere
    vec4f LowerColor {0.f, 0.f, 0.f, 1.f};
    vec4f UpperColor {1.f, 1.f, 1.f, 1.f};
  };

  struct DLAF_Results
  {
    containers::AlignedVector<vec3f> points;
    containers::AlignedVector<vec4f> colors;
  };

  DLAF_Results compute_points(DLAF_Params p, float &done);

}  // namespace dlaf
