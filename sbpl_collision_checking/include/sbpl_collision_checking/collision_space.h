////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Benjamin Cohen, Andrew Dornbush
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

#ifndef sbpl_collision_collision_space_h
#define sbpl_collision_collision_space_h

// standard includes
#include <string>
#include <memory>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <smpl/collision_checker.h>
#include <smpl/occupancy_grid.h>
#include <shape_msgs/MeshTriangle.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_collision_checking/allowed_collisions_interface.h>
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/robot_motion_collision_model.h>
#include <sbpl_collision_checking/robot_collision_state.h>
#include <sbpl_collision_checking/self_collision_model.h>
#include <sbpl_collision_checking/world_collision_model.h>
#include <sbpl_collision_checking/types.h>

namespace sbpl {
namespace collision {

class CollisionSpaceBuilder;

class CollisionSpace : public motion::CollisionChecker
{
public:

    ~CollisionSpace();

    bool setPlanningScene(const moveit_msgs::PlanningScene& scene);

    /// \name Robot State
    ///@{
    bool setJointPosition(const std::string& name, double position);
    void setWorldToModelTransform(const Eigen::Affine3d& transform);
    ///@}

    void setPadding(double padding);

    /// \name Self Collisions
    ///@{
    const AllowedCollisionMatrix& allowedCollisionMatrix() const;
    void updateAllowedCollisionMatrix(const AllowedCollisionMatrix& acm);
    void setAllowedCollisionMatrix(const AllowedCollisionMatrix& acm);
    ///@}

    /// \name World Collision Model
    ///@{
    bool insertObject(const ObjectConstPtr& object);
    bool removeObject(const ObjectConstPtr& object);
    bool removeObject(const std::string& object_name);
    bool moveShapes(const ObjectConstPtr& object);
    bool insertShapes(const ObjectConstPtr& object);
    bool removeShapes(const ObjectConstPtr& object);
    bool processCollisionObject(const moveit_msgs::CollisionObject& object);
    bool processOctomapMsg(const octomap_msgs::OctomapWithPose& octomap);
    ///@}

    /// \name Attached Objects
    ///@{
    bool attachObject(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3dVector& transforms,
        const std::string& link_name);

    bool attachObject(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3dVector& transforms,
        const std::string& link_name,
        double sphere_radius);

    bool detachObject(const std::string& id);

    bool processAttachedCollisionObject(
        const moveit_msgs::AttachedCollisionObject& obj);
    ///@}

    const std::string& getReferenceFrame() const;

    const std::string& getGroupName() const;

    const RobotCollisionModelConstPtr&  robotCollisionModel() const;
    const RobotMotionCollisionModelConstPtr& robotMotionCollisionModel() const;
    WorldCollisionModelConstPtr  worldCollisionModel() const;
    SelfCollisionModelConstPtr   selfCollisionModel() const;

    /// \name Visualization
    ///@{
    visualization_msgs::MarkerArray getWorldVisualization() const;
    visualization_msgs::MarkerArray getRobotVisualization() const;

    visualization_msgs::MarkerArray getCollisionWorldVisualization() const;
    visualization_msgs::MarkerArray getCollisionRobotVisualization() const;
    visualization_msgs::MarkerArray getCollisionRobotVisualization(
        const std::vector<double>& vals);

    visualization_msgs::MarkerArray getCollisionDetailsVisualization() const;
    visualization_msgs::MarkerArray getCollisionDetailsVisualization(
        const std::vector<double>& vals);

    visualization_msgs::MarkerArray getBoundingBoxVisualization() const;
    visualization_msgs::MarkerArray getDistanceFieldVisualization() const;
    visualization_msgs::MarkerArray getOccupiedVoxelsVisualization() const;
    ///@}

    bool checkCollision(
        const std::vector<double>& state,
        const AllowedCollisionsInterface& aci,
        bool verbose,
        bool visualize,
        double& dist);

    bool checkCollision(const std::vector<double>& state, double& dist);

    double collisionDistance(const std::vector<double>& state);

    bool collisionDetails(
        const std::vector<double>& state,
        CollisionDetails& details);

    /// \name Required Functions from Extension
    ///@{
    motion::Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Required Functions from CollisionChecker
    ///@{
    bool isStateValid(
        const motion::RobotState& state,
        bool verbose,
        bool visualize,
        double& dist) override;

    bool isStateToStateValid(
        const motion::RobotState& start,
        const motion::RobotState& finish,
        int& path_length,
        int& num_checks,
        double& dist) override;

    bool interpolatePath(
        const motion::RobotState& start,
        const motion::RobotState& finish,
        std::vector<motion::RobotState>& path) override;
    ///@}

    /// \name Reimplemented Functions from CollisionChecker
    ///@{
    visualization_msgs::MarkerArray
    getCollisionModelVisualization(const motion::RobotState& vals) override;

    visualization_msgs::MarkerArray
    getVisualization(const std::string& type) override;
    ///@}

private:

    OccupancyGrid*                  m_grid;

    RobotCollisionModelConstPtr         m_rcm;
    AttachedBodiesCollisionModelPtr     m_abcm;

    RobotMotionCollisionModelConstPtr   m_rmcm;

    RobotCollisionStatePtr          m_rcs;
    AttachedBodiesCollisionStatePtr m_abcs;
    std::vector<double>             m_joint_vars;

    WorldCollisionModelPtr          m_wcm;
    SelfCollisionModelPtr           m_scm;

    // Collision Group
    std::string                     m_group_name;
    int                             m_gidx;

    // Planning Joint Information
    std::vector<int>                m_planning_joint_to_collision_model_indices;
    std::vector<double>             m_increments;

    CollisionSpace();

    bool init(
        OccupancyGrid* grid,
        const std::string& urdf_string,
        const CollisionModelConfig& config,
        const std::string& group_name,
        const std::vector<std::string>& planning_joints);

    bool init(
        OccupancyGrid* grid,
        const urdf::ModelInterface& urdf,
        const CollisionModelConfig& config,
        const std::string& group_name,
        const std::vector<std::string>& planning_joints);

    bool init(
        OccupancyGrid* grid,
        const RobotCollisionModelConstPtr& rcm,
        const std::string& group_name,
        const std::vector<std::string>& planning_joints);

    bool setPlanningJoints(const std::vector<std::string>& joint_names);
    size_t planningVariableCount() const;

    bool isContinuous(int vidx) const;
    bool hasLimit(int vidx) const;
    double minLimit(int vidx) const;
    double maxLimit(int vidx) const;

    void updateState(const std::vector<double>& vals);
    void updateState(
        std::vector<double>& state,
        const std::vector<double>& vals);
    void copyState();

    bool withinJointPositionLimits(const std::vector<double>& positions) const;

    friend class CollisionSpaceBuilder;
};

/// \brief Return the reference frame of the occupancy grid
inline
const std::string& CollisionSpace::getReferenceFrame() const
{
    return m_grid->getReferenceFrame();
}

/// \brief Return the group being collision checked
inline
const std::string& CollisionSpace::getGroupName() const
{
    return m_group_name;
}

inline
const RobotCollisionModelConstPtr& CollisionSpace::robotCollisionModel() const
{
    return m_rcm;
}

inline
const RobotMotionCollisionModelConstPtr&
CollisionSpace::robotMotionCollisionModel() const
{
    return m_rmcm;
}

inline
WorldCollisionModelConstPtr CollisionSpace::worldCollisionModel() const
{
    return m_wcm;
}

inline
SelfCollisionModelConstPtr CollisionSpace::selfCollisionModel() const
{
    return m_scm;
}

inline
size_t CollisionSpace::planningVariableCount() const
{
    return m_planning_joint_to_collision_model_indices.size();
}

inline
bool CollisionSpace::isContinuous(int vidx) const
{
    const int jidx = m_planning_joint_to_collision_model_indices[vidx];
    return m_rcm->jointVarIsContinuous(jidx);
}

inline
bool CollisionSpace::hasLimit(int vidx) const
{
    const int jidx = m_planning_joint_to_collision_model_indices[vidx];
    return m_rcm->jointVarHasPositionBounds(jidx);
}

inline
double CollisionSpace::minLimit(int vidx) const
{
    const int jidx = m_planning_joint_to_collision_model_indices[vidx];
    return m_rcm->jointVarMinPosition(jidx);
}

inline
double CollisionSpace::maxLimit(int vidx) const
{
    const int jidx = m_planning_joint_to_collision_model_indices[vidx];
    return m_rcm->jointVarMaxPosition(jidx);
}

typedef std::shared_ptr<CollisionSpace> CollisionSpacePtr;
typedef std::shared_ptr<const CollisionSpace> CollisionSpaceConstPtr;

class CollisionSpaceBuilder
{
public:

    CollisionSpacePtr build(
        OccupancyGrid* grid,
        const std::string& urdf_string,
        const CollisionModelConfig& config,
        const std::string& group_name,
        const std::vector<std::string>& planning_joints);

    CollisionSpacePtr build(
        OccupancyGrid* grid,
        const urdf::ModelInterface& urdf,
        const CollisionModelConfig& config,
        const std::string& group_name,
        const std::vector<std::string>& planning_joints);

    CollisionSpacePtr build(
        OccupancyGrid* grid,
        const RobotCollisionModelConstPtr& rcm,
        const std::string& group_name,
        const std::vector<std::string>& planning_joints);
};

} // namespace collision
} // namespace sbpl

#endif
