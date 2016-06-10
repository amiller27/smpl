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

#ifndef sbpl_manip_kdl_robot_model_h
#define sbpl_manip_kdl_robot_model_h

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <angles/angles.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/console.h>
#include <sbpl_arm_planner/robot_model.h>
#include <sbpl_geometry_utils/interpolation.h>
#include <urdf/model.h>

namespace sbpl {
namespace manip {

class KDLRobotModel : public RobotModel
{
public:

    static const int DEFAULT_FREE_ANGLE_INDEX = 2;

    /// @brief Construct a KDL Robot Model with values from the ROS param server.
    ///
    /// Expects ROS Parameters:
    ///     robot_model/chain_root_link : string
    ///     robot_model/chain_tip_link : string
    ///     robot_model/free_angle : int (default: 2)
    KDLRobotModel();

    /// @brief Construct a KDL Robot Model.
    KDLRobotModel(
            const std::string& chain_root_link,
            const std::string& chain_tip_link,
            int free_angle = DEFAULT_FREE_ANGLE_INDEX);

    virtual ~KDLRobotModel();

    /* Initialization */
    virtual bool init(
        const std::string& robot_description,
        const std::vector<std::string>& planning_joints);

    const std::string& getKinematicsFrame() const;

    /// \brief Transform between Kinematics frame <-> Planning frame
    void setKinematicsToPlanningTransform(
        const KDL::Frame& f,
        const std::string& name);

    virtual double minPosLimit(int jidx) const { return min_limits_[jidx]; }
    virtual double maxPosLimit(int jidx) const { return max_limits_[jidx]; }
    virtual bool   hasPosLimit(int jidx) const { return continuous_[jidx]; }
    virtual double velLimit(int jidx) const { return vel_limits_[jidx]; }
    virtual double accLimit(int jidx) const { return 0.0; }

    /* Joint Limits */
    virtual bool checkJointLimits(
        const std::vector<double>& angles,
        bool verbose = false);

    /* Forward Kinematics */

    /// \brief Compute the forward kinematics pose of a link in the robot model.
    virtual bool computeFK(
        const std::vector<double>& angles,
        const std::string& name,
        KDL::Frame& f);

    virtual bool computeFK(
        const std::vector<double>& angles,
        const std::string& name,
        std::vector<double>& pose);

    virtual bool computePlanningLinkFK(
        const std::vector<double>& angles,
        std::vector<double>& pose);

    /* Inverse Kinematics */
    virtual bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution,
        ik_option::IkOption option = ik_option::UNRESTRICTED);

    virtual bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector< std::vector<double>>& solutions,
        ik_option::IkOption option = ik_option::UNRESTRICTED);

    virtual bool computeFastIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution);

    bool computeIKSearch(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution,
        double timeout);

    /* Debug Output */
    virtual void printRobotModelInformation();

protected:

    /** \brief frame that the kinematics is computed in (i.e. robot base) */
    std::string kinematics_frame_;

    KDL::Frame T_kinematics_to_planning_;
    KDL::Frame T_planning_to_kinematics_;

    bool initialized_;

    boost::shared_ptr<urdf::Model> urdf_;
    int free_angle_;
    std::string chain_root_name_;
    std::string chain_tip_name_;

    KDL::Tree ktree_;
    KDL::Chain kchain_;
    KDL::JntArray jnt_pos_in_;
    KDL::JntArray jnt_pos_out_;
    KDL::Frame p_out_;

    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL>        ik_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv>         ik_vel_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive>    fk_solver_;

    std::vector<bool> continuous_;
    std::vector<double> min_limits_;
    std::vector<double> max_limits_;
    std::vector<double> vel_limits_;
    std::vector<double> eff_limits_;
    std::map<std::string, int> joint_map_;
    std::map<std::string, int> link_map_;

    bool getJointLimits(
        std::vector<std::string>& joint_names,
        std::vector<double>& min_limits,
        std::vector<double>& max_limits,
        std::vector<bool>& continuous,
        std::vector<double>& vel_limits,
        std::vector<double>& acc_limits);
    bool getJointLimits(
        std::string joint_name,
        double& min_limit,
        double& max_limit,
        bool& continuous,
        double& vel_limit,
        double& acc_limit);
    bool getCount(int& count, const int& max_count, const int& min_count);
};

} // namespace manip
} // namespace sbpl

#endif
