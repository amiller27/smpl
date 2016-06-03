////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen
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
//     * Neither the name of the University of Pennsylvania nor the names of its
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
////////////////////////////////////////////////////////////////////////////////

/// \author Benjamin Cohen

#ifndef sbpl_manip_post_processing_h
#define sbpl_manip_post_processing_h

#include <vector>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sbpl_manipulation_components/collision_checker.h>

namespace sbpl {
namespace manip {

void shortcutPath(
    CollisionChecker* cc,
    std::vector<std::vector<double>>& pin,
    std::vector<std::vector<double>>& pout);

void shortcutTrajectory(
    CollisionChecker* cc,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_in,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_out);

bool interpolateTrajectory(
    CollisionChecker* cc,
    const std::vector<trajectory_msgs::JointTrajectoryPoint>& traj,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_out);

} // namespace manip
} // namespace sbpl

#endif
