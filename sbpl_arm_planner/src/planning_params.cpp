////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
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
/// \author Andrew Dornbush

#include <sbpl_arm_planner/planning_params.h>

// system includes
#include <leatherman/utils.h>
#include <leatherman/print.h>

namespace sbpl {
namespace manip {

const std::string PlanningParams::DefaultRobotModelLog = "robot";
const std::string PlanningParams::DefaultGraphLog = "graph";
const std::string PlanningParams::DefaultHeuristicLog = "heuristic";
const std::string PlanningParams::DefaultExpandsLog = "expands";
const std::string PlanningParams::DefaultPostProcessingLog = "post_process";
const std::string PlanningParams::DefaultSolutionLog = "solution";

std::string to_string(ShortcutType type)
{
    switch (type) {
    case ShortcutType::JOINT_SPACE:
        return "JOINT_SPACE";
    case ShortcutType::EUCLID_SPACE:
        return "EUCLID_SPACE";
    default:
        return "INVALID_SHORTCUT_TYPE";
    }
}

PlanningParams::PlanningParams() :
    planning_frame_(),
    group_name_(),
    num_joints_(0),
    planning_joints_(),
    coord_vals_(),
    coord_delta_(),

    use_multiple_ik_solutions_(DefaultUseMultipleIkSolutions),

    cost_multiplier_(DefaultCostMultiplier),
    cost_per_cell_(DefaultCostPerCell),
    cost_per_meter_(DefaultCostPerMeter),
    cost_per_second_(DefaultCostPerSecond),
    time_per_cell_(DefaultTimePerCell),
    max_mprim_offset_(DefaultMaxMprimOffset),

    use_bfs_heuristic_(DefaultUseBfsHeuristic),
    planning_link_sphere_radius_(DefaultPlanningLinkSphereRadius),

    planner_name_(),
    epsilon_(DefaultEpsilon),
    allowed_time_(DefaultAllowedTime),
    search_mode_(DefaultSearchMode),

    shortcut_path_(DefaultShortcutPath),
    interpolate_path_(DefaultInterpolatePath),
    waypoint_time_(DefaultWaypointTime),
    shortcut_type(DefaultShortcutType),

    print_path_(true),
    verbose_(false),
    verbose_heuristics_(false),
    verbose_collisions_(false),
    rmodel_log_(DefaultRobotModelLog),
    graph_log_(DefaultGraphLog),
    heuristic_log_(DefaultHeuristicLog),
    expands_log_(DefaultExpandsLog),
    post_processing_log_(DefaultPostProcessingLog),
    solution_log_(DefaultSolutionLog)
{
}

bool PlanningParams::init(const std::string& ns)
{
    ros::NodeHandle nh(ns);
    ROS_ERROR("Getting params from namespace: %s", nh.getNamespace().c_str());
    /* planning */
    nh.param("planning/epsilon", epsilon_, 10.0);
    nh.param<std::string>("planning/planner_name", planner_name_, "ARA*");
    nh.param("planning/use_bfs_heuristic", use_bfs_heuristic_, true);
    nh.param("planning/verbose", verbose_, false);
    nh.param("planning/verbose_collisions", verbose_collisions_, false);
    nh.param("planning/search_mode", search_mode_, false); //true: stop after first solution
    nh.param("planning/shortcut_path", shortcut_path_, false);
    // TODO: shortcut_type
    nh.param("planning/interpolate_path", interpolate_path_, false);
    nh.param("planning/use_multiple_ik_solutions", use_multiple_ik_solutions_, false);
    nh.param("planning/seconds_per_waypoint", waypoint_time_, 0.35);
    nh.param<std::string>("planning/planning_frame", planning_frame_, "");
    nh.param<std::string>("planning/group_name", group_name_, "");

    /* logging */
    nh.param ("debug/print_out_path", print_path_, true);

    /* planning joints */
    XmlRpc::XmlRpcValue xlist;
    nh.getParam("planning/planning_joints", xlist);
    std::string joint_list = std::string(xlist);
    std::stringstream joint_name_stream(joint_list);
    while (joint_name_stream.good() && !joint_name_stream.eof()) {
        std::string jname;
        joint_name_stream >> jname;
        if (jname.size() == 0) {
            continue;
        }
        planning_joints_.push_back(jname);
    }
    num_joints_ = planning_joints_.size();

    // discretization
    std::string p;
    if (nh.hasParam("planning/discretization")) {
        nh.getParam("planning/discretization", xlist);
        std::stringstream ss(xlist);
        while (ss >> p) {
            coord_vals_.push_back(atof(p.c_str()));
        }

        coord_delta_.resize(coord_vals_.size(), 0);
        for (int i = 0; i < num_joints_; ++i) {
            coord_delta_[i] = (2.0 * M_PI) / coord_vals_[i];
        }
    }
    else {
        ROS_ERROR("Discretization of statespace has not been defined.");
        return false;
    }

    return true;
}

void PlanningParams::printParams(const std::string& stream) const
{
    ROS_INFO_NAMED(stream, " ");
    ROS_INFO_NAMED(stream, "Manipulation Environment Parameters:");
    ROS_INFO_NAMED(stream, "%40s: %.2f", "epsilon",epsilon_);
    ROS_INFO_NAMED(stream, "%40s: %s", "use dijkstra heuristic", use_bfs_heuristic_ ? "yes" : "no");
    ROS_INFO_NAMED(stream, "%40s: %s", "sbpl search mode", search_mode_ ? "stop_after_first_sol" : "run_until_timeout");
    ROS_INFO_NAMED(stream, "%40s: %s", "postprocessing: shortcut", shortcut_path_ ? "yes" : "no");
    ROS_INFO_NAMED(stream, "%40s: %s", "postprocessing: interpolate", interpolate_path_ ? "yes" : "no");
    ROS_INFO_NAMED(stream, "%40s: %0.3fsec", "time_per_waypoint", waypoint_time_);
    ROS_INFO_NAMED(stream, "%40s: %d", "cost per cell", cost_per_cell_);
    ROS_INFO_NAMED(stream, "%40s: %s", "reference frame", planning_frame_.c_str());
    ROS_INFO_NAMED(stream, "%40s: %s", "group name", group_name_.c_str());
    ROS_INFO_NAMED(stream, "planning joints: ");
    for (size_t i = 0; i < planning_joints_.size(); ++i) {
        ROS_INFO_NAMED(stream, "   [%d] %30s", int(i), planning_joints_[i].c_str());
    }
    ROS_INFO_NAMED(stream, "discretization: ");
    for (size_t i = 0; i < coord_vals_.size(); ++i) {
        ROS_INFO_NAMED(stream, "   [%d] val: %d  delta: %0.3f", int(i), coord_vals_[i], coord_delta_[i]);
    }
    ROS_INFO_NAMED(stream, " ");
}

} // namespace manip
} // namespace sbpl
