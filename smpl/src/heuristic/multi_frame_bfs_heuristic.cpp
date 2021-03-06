////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

/// \author Andrew Dornbush

#include <smpl/heuristic/multi_frame_bfs_heuristic.h>

// system includes
#include <leatherman/viz.h>

// project includes
#include <smpl/bfs3d/bfs3d.h>

namespace sbpl {
namespace motion {

MultiFrameBfsHeuristic::MultiFrameBfsHeuristic(
    const RobotPlanningSpacePtr& ps,
    const OccupancyGrid* grid)
:
    RobotHeuristic(ps, grid),
    m_bfs(),
    m_ee_bfs()
{
    m_pp = ps->getExtension<PointProjectionExtension>();
    if (m_pp) {
        ROS_INFO_NAMED(params()->heuristic_log, "Got Point Projection Extension!");
    }
    m_ers = ps->getExtension<ExtractRobotStateExtension>();
    if (m_ers) {
        ROS_INFO_NAMED(params()->heuristic_log, "Got Extract Robot State Extension!");
    }
    m_fk_iface = ps->robot()->getExtension<ForwardKinematicsInterface>();
    if (m_fk_iface) {
        ROS_INFO_NAMED(params()->heuristic_log, "Got Forward Kinematics Interface!");
    }
    syncGridAndBfs();
}

MultiFrameBfsHeuristic::~MultiFrameBfsHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

Extension* MultiFrameBfsHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

void MultiFrameBfsHeuristic::updateGoal(const GoalConstraint& goal)
{
    ROS_DEBUG_NAMED(params()->heuristic_log, "Update goal");

    int ogx, ogy, ogz;
    grid()->worldToGrid(
            goal.tgt_off_pose[0], goal.tgt_off_pose[1], goal.tgt_off_pose[2],
            ogx, ogy, ogz);

    int plgx, plgy, plgz;
    grid()->worldToGrid(
            goal.pose[0], goal.pose[1], goal.pose[2],
            plgx, plgy, plgz);

    ROS_DEBUG_NAMED(params()->heuristic_log, "Setting the Two-Point BFS heuristic goals (%d, %d, %d), (%d, %d, %d)", ogx, ogy, ogz, plgx, plgy, plgz);

    if (!m_bfs->inBounds(ogx, ogy, ogz) ||
        !m_ee_bfs->inBounds(plgx, plgy, plgz))
    {
        ROS_ERROR_NAMED(params()->heuristic_log, "Heuristic goal is out of BFS bounds");
        return;
    }

    m_bfs->run(ogx, ogy, ogz);
    m_ee_bfs->run(plgx, plgy, plgz);
}

double MultiFrameBfsHeuristic::getMetricStartDistance(double x, double y, double z)
{
    // TODO: shamefully copied from BfsHeuristic
    int start_id = planningSpace()->getStartStateID();

    if (!m_pp) {
        return 0.0;
    }

    Eigen::Vector3d p;
    if (!m_pp->projectToPoint(planningSpace()->getStartStateID(), p)) {
        return 0.0;
    }

    int sx, sy, sz;
    grid()->worldToGrid(p.x(), p.y(), p.z(), sx, sy, sz);

    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);

    // compute the manhattan distance to the start cell
    const int dx = sx - gx;
    const int dy = sy - gy;
    const int dz = sz - gz;
    return grid()->resolution() * (abs(dx) + abs(dy) + abs(dz));
}

double MultiFrameBfsHeuristic::getMetricGoalDistance(
    double x, double y, double z)
{
    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_bfs->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * grid()->resolution();
    } else {
        return (double)m_bfs->getDistance(gx, gy, gz) * grid()->resolution();
    }
}

int MultiFrameBfsHeuristic::GetGoalHeuristic(int state_id)
{
    return getGoalHeuristic(state_id, true);
}

int MultiFrameBfsHeuristic::GetStartHeuristic(int state_id)
{
    ROS_WARN_ONCE("MultiFrameBfsHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int MultiFrameBfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    } else {
        ROS_WARN_ONCE("MultiFrameBfsHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

visualization_msgs::MarkerArray
MultiFrameBfsHeuristic::getWallsVisualization() const
{
    std::vector<geometry_msgs::Point> points;
    const int dimX = grid()->numCellsX();
    const int dimY = grid()->numCellsY();
    const int dimZ = grid()->numCellsZ();
    for (int z = 0; z < dimZ; z++) {
        for (int y = 0; y < dimY; y++) {
            for (int x = 0; x < dimX; x++) {
                if (m_bfs->isWall(x, y, z)) {
                    geometry_msgs::Point p;
                    grid()->gridToWorld(x, y, z, p.x, p.y, p.z);
                    points.push_back(p);
                }
            }
        }
    }

    ROS_DEBUG_NAMED(params()->heuristic_log, "BFS Visualization contains %zu points", points.size());

    std_msgs::ColorRGBA color;
    color.r = 100.0f / 255.0f;
    color.g = 149.0f / 255.0f;
    color.b = 238.0f / 255.0f;
    color.a = 1.0f;

    visualization_msgs::Marker cubes_marker = viz::getCubesMarker(
            points,
            grid()->resolution(),
            color,
            grid()->getReferenceFrame(),
            "bfs_walls",
            0);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(std::move(cubes_marker));
    return ma;
}

visualization_msgs::MarkerArray
MultiFrameBfsHeuristic::getValuesVisualization() const
{
    visualization_msgs::MarkerArray ma;

    // factor in the ee bfs values? This doesn't seem to make a whole lot of
    // sense since the color would be derived from colocated cell values
    const bool factor_ee = false;

    // hopefully this doesn't screw anything up too badly...this will flush the
    // bfs to a little past the start, but this would be done by the search
    // hereafter anyway
    int start_heur = getGoalHeuristic(planningSpace()->getStartStateID(), factor_ee);

    const int edge_cost = params()->cost_per_cell;

    int max_cost = (int)(1.1 * start_heur);

    // ...and this will also flush the bfs...

    std::vector<geometry_msgs::Point> points;
    std::vector<std_msgs::ColorRGBA> colors;
    for (int z = 0; z < grid()->numCellsZ(); ++z) {
    for (int y = 0; y < grid()->numCellsY(); ++y) {
    for (int x = 0; x < grid()->numCellsX(); ++x) {
        // skip cells without valid distances from the start
        if (m_bfs->isWall(x, y, z) || m_bfs->isUndiscovered(x, y, z)) {
            continue;
        }

        int d = edge_cost * m_bfs->getDistance(x, y, z);
        int eed = factor_ee ? edge_cost * m_ee_bfs->getDistance(x, y, z) : 0;
        double cost_pct = (double)combine_costs(d, eed) / (double)(max_cost);

        if (cost_pct > 1.0) {
            continue;
        }

        double hue = 300.0 - 300.0 * cost_pct;
        double sat = 1.0;
        double val = 1.0;
        double r, g, b;
        leatherman::HSVtoRGB(&r, &g, &b, hue, sat, val);

        std_msgs::ColorRGBA color;
        color.r = (float)r;
        color.g = (float)g;
        color.b = (float)b;
        color.a = 1.0f;

        auto clamp = [](double d, double lo, double hi) {
            if (d < lo) {
                return lo;
            } else if (d > hi) {
                return hi;
            } else {
                return d;
            }
        };

        color.r = clamp(color.r, 0.0f, 1.0f);
        color.g = clamp(color.g, 0.0f, 1.0f);
        color.b = clamp(color.b, 0.0f, 1.0f);

        geometry_msgs::Point p;
        grid()->gridToWorld(x, y, z, p.x, p.y, p.z);
        points.push_back(p);

        colors.push_back(color);
    }
    }
    }

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time(0);
    marker.header.frame_id = grid()->getReferenceFrame();
    marker.ns = "bfs_values";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5 * grid()->resolution();
    marker.scale.y = 0.5 * grid()->resolution();
    marker.scale.z = 0.5 * grid()->resolution();
//    marker.color;
    marker.lifetime = ros::Duration(0.0);
    marker.frame_locked = false;
    marker.points = std::move(points);
    marker.colors = std::move(colors);
    marker.text = "";
    marker.mesh_use_embedded_materials = false;

    ma.markers.push_back(std::move(marker));
    return ma;
}

int MultiFrameBfsHeuristic::getGoalHeuristic(int state_id, bool use_ee) const
{
    if (state_id == planningSpace()->getGoalStateID()) {
        return 0;
    }

    int h_planning_frame = 0;
    if (m_pp) {
        Eigen::Vector3d p;
        if (m_pp->projectToPoint(state_id, p)) {
            Eigen::Vector3i dp;
            grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());
            h_planning_frame = getBfsCostToGoal(*m_bfs, dp.x(), dp.y(), dp.z());
        }
    }

    int h_planning_link = 0;
    if (use_ee && m_ers && m_fk_iface) {
        const RobotState& state = m_ers->extractState(state_id);
        std::vector<double> pose;
        if (m_fk_iface->computePlanningLinkFK(state, pose)) {
            Eigen::Vector3i eex;
            grid()->worldToGrid(pose[0], pose[1], pose[2], eex[0], eex[1], eex[2]);
            h_planning_link = getBfsCostToGoal(*m_ee_bfs, eex[0], eex[1], eex[2]);
        } else {
            ROS_ERROR_NAMED(params()->heuristic_log, "Failed to compute FK for planning link (state = %d)", state_id);
        }
    }

    return combine_costs(h_planning_frame, h_planning_link);
}

void MultiFrameBfsHeuristic::syncGridAndBfs()
{
    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
    const int zc = grid()->numCellsZ();
    m_bfs.reset(new BFS_3D(xc, yc, zc));
    m_ee_bfs.reset(new BFS_3D(xc, yc, zc));
    const int cell_count = xc * yc * zc;
    int wall_count = 0;
    for (int z = 0; z < zc; ++z) {
        for (int y = 0; y < yc; ++y) {
            for (int x = 0; x < xc; ++x) {
                const double radius = params()->planning_link_sphere_radius;
                if (grid()->getDistance(x, y, z) <= radius) {
                    m_bfs->setWall(x, y, z);
                    m_ee_bfs->setWall(x, y, z);
                    ++wall_count;
                }
            }
        }
    }

    ROS_DEBUG_NAMED(params()->heuristic_log, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

int MultiFrameBfsHeuristic::getBfsCostToGoal(
    const BFS_3D& bfs, int x, int y, int z) const
{
    if (!bfs.inBounds(x, y, z)) {
        return Infinity;
    }
    else if (bfs.getDistance(x, y, z) == BFS_3D::WALL) {
        return Infinity;
    }
    else {
        return params()->cost_per_cell * bfs.getDistance(x, y, z);
    }
}

int MultiFrameBfsHeuristic::combine_costs(int c1, int c2) const
{
    return c1 + c2;
}

} // namespace motion
} // namespace sbpl
