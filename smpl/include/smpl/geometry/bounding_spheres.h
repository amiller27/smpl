//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2014, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#ifndef SMPL_BOUNDING_SPHERES_H
#define SMPL_BOUNDING_SPHERES_H

// standard includes
#include <vector>

// system includes
#include <Eigen/Dense>

namespace sbpl {
namespace geometry {

void ComputeBoxBoundingSpheres(
    double length, double width, double height,
    double radius, std::vector<Eigen::Vector3d>& centers);

void ComputeSphereBoundingSpheres(
    double cradius,
    double radius, std::vector<Eigen::Vector3d>& centers);

void ComputeCylinderBoundingSpheres(
    double cradius, double cheight,
    double radius, std::vector<Eigen::Vector3d>& centers);

void ComputeConeBoundingSpheres(
    double cradius, double cheight,
    double radius, std::vector<Eigen::Vector3d>& centers);

void ComputeMeshBoundingSpheres(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    double radius, std::vector<Eigen::Vector3d>& centers);

} // namespace geometry
} // namespace sbpl

#endif
