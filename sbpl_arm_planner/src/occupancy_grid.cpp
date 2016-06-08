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

#include <sbpl_arm_planner/occupancy_grid.h>

 // system includes
#include <ros/console.h>
#include <leatherman/viz.h>

using namespace std;

namespace sbpl {

OccupancyGrid::OccupancyGrid(
    double dim_x, double dim_y, double dim_z,
    double resolution,
    double origin_x, double origin_y, double origin_z,
    double max_dist)
{
    grid_ = new distance_field::PropagationDistanceField(
        dim_x, dim_y, dim_z, resolution, origin_x, origin_y, origin_z, max_dist);
    grid_->reset();
    delete_grid_ = true;
}

OccupancyGrid::OccupancyGrid(distance_field::PropagationDistanceField* df)
{
    grid_ = df;
    delete_grid_ = false;
}

OccupancyGrid::~OccupancyGrid()
{
    if (grid_ && delete_grid_)
        delete grid_;
}

void OccupancyGrid::getGridSize(int &dim_x, int &dim_y, int &dim_z) const
{
    dim_x = grid_->getSizeX() / grid_->getResolution();
    dim_y = grid_->getSizeY() / grid_->getResolution();
    dim_z = grid_->getSizeZ() / grid_->getResolution();
}

void OccupancyGrid::getWorldSize(double &dim_x, double &dim_y, double &dim_z) const
{
    dim_x = grid_->getSizeX();
    dim_y = grid_->getSizeY();
    dim_z = grid_->getSizeZ();
}

void OccupancyGrid::reset()
{
    grid_->reset();
}

void OccupancyGrid::getOrigin(double &wx, double &wy, double &wz) const
{
    grid_->gridToWorld(0, 0, 0, wx, wy, wz);
}

void OccupancyGrid::getOccupiedVoxels(
    const geometry_msgs::Pose& pose,
    const std::vector<double>& dim,
    std::vector<Eigen::Vector3d>& voxels) const
{
    Eigen::Vector3d vin, vout, v(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Matrix3d m(Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));

    for (double x = 0 - dim[0] / 2.0; x <= dim[0] / 2.0; x += grid_->getResolution()) {
        for (double y = 0 - dim[1] / 2.0; y <= dim[1] / 2.0; y += grid_->getResolution()) {
            for (double z = 0 - dim[2] / 2.0; z <= dim[2] / 2.0; z += grid_->getResolution()) {
                vin(0) = (x);
                vin(1) = (y);
                vin(2) = (z);
                vout = m * vin;
                vout += v;

                voxels.push_back(vout);
            }
        }
    }
}

void OccupancyGrid::getOccupiedVoxels(
    double x_center,
    double y_center,
    double z_center,
    double radius,
    std::vector<geometry_msgs::Point>& voxels) const
{
    int x_c, y_c, z_c, radius_c;
    geometry_msgs::Point v;
    worldToGrid(x_center, y_center, z_center, x_c, y_c, z_c);
    radius_c = radius / getResolution() + 0.5;

    for (int z = z_c - radius_c; z < z_c + radius_c; ++z) {
        for (int y = y_c - radius_c; y < y_c + radius_c; ++y) {
            for (int x = x_c - radius_c; x < x_c + radius_c; ++x) {
                if (getCell(x, y, z) == 0) {
                    gridToWorld(x, y, z, v.x, v.y, v.z);
                    voxels.push_back(v);
                }
            }
        }
    }
}

void OccupancyGrid::getOccupiedVoxels(
    std::vector<geometry_msgs::Point>& voxels) const
{
    for (int gx = 0; gx < grid_->getXNumCells(); ++gx) {
        for (int gy = 0; gy < grid_->getYNumCells(); ++gy) {
            for (int gz = 0; gz < grid_->getZNumCells(); ++gz) {
                if (grid_->getDistance(gx, gy, gz) == 0.0) {
                    double wx, wy, wz;
                    grid_->gridToWorld(gx, gy, gz, wx, wy, wz);
                    geometry_msgs::Point v;
                    v.x = wx;
                    v.y = wy;
                    v.z = wz;
                    voxels.push_back(v);
                }
            }
        }
    }
}

visualization_msgs::MarkerArray OccupancyGrid::getVisualization(
    const std::string& type) const
{
    if (type == "bounds") {
        return getBoundingBoxVisualization();
    }
    else if (type == "distance_field") {
        return getDistanceFieldVisualization();
    }
    else if (type == "occupied_voxels") {
        return getOccupiedVoxelsVisualization();
    }
    else {
        ROS_ERROR("No Occupancy Grid visualization of type '%s' found", type.c_str());
        return visualization_msgs::MarkerArray();
    }
}

visualization_msgs::MarkerArray
OccupancyGrid::getBoundingBoxVisualization() const
{
    visualization_msgs::MarkerArray ma;
    double dimx, dimy, dimz, originx, originy, originz;
    std::vector<geometry_msgs::Point> pts(10);
    getOrigin(originx, originy, originz);
    getWorldSize(dimx,dimy,dimz);
    pts[0].x = originx;      pts[0].y = originy;      pts[0].z = originz;
    pts[1].x = originx+dimx; pts[1].y = originy;      pts[1].z = originz;
    pts[2].x = originx+dimx; pts[2].y = originy+dimy; pts[2].z = originz;
    pts[3].x = originx;      pts[3].y = originy+dimy; pts[3].z = originz;
    pts[4].x = originx;      pts[4].y = originy;      pts[4].z = originz;
    pts[5].x = originx;      pts[5].y = originy;      pts[5].z = originz+dimz;
    pts[6].x = originx+dimx; pts[6].y = originy;      pts[6].z = originz+dimz;
    pts[7].x = originx+dimx; pts[7].y = originy+dimy; pts[7].z = originz+dimz;
    pts[8].x = originx;      pts[8].y = originy+dimy; pts[8].z = originz+dimz;
    pts[9].x = originx;      pts[9].y = originy;      pts[9].z = originz+dimz;

    ma.markers.resize(1);

    double thickness = 0.05;
    int hue = 10;
    ma.markers[0] = viz::getLineMarker(
            pts, 0.05, 10, getReferenceFrame(), "collision_space_bounds");
    return ma;
}

visualization_msgs::MarkerArray
OccupancyGrid::getDistanceFieldVisualization() const
{
    visualization_msgs::MarkerArray ma;

    visualization_msgs::Marker m;
    grid_->getIsoSurfaceMarkers(
            grid_->getResolution(),
            getMaxDistance(),
            getReferenceFrame(),
            ros::Time::now(),
            m);
    m.color.a += 0.2;

    ma.markers.push_back(m);
    return ma;
}

visualization_msgs::MarkerArray
OccupancyGrid::getOccupiedVoxelsVisualization() const
{
    visualization_msgs::MarkerArray ma;

    visualization_msgs::Marker marker;

    marker.header.seq = 0;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = getReferenceFrame();

    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.0);

    marker.scale.x = grid_->getResolution();
    marker.scale.y = grid_->getResolution();
    marker.scale.z = grid_->getResolution();

    marker.color.r = 0.8f;
    marker.color.g = 0.3f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0f;

    std::vector<geometry_msgs::Point> voxels;
    getOccupiedVoxels(voxels);
    marker.points = voxels;

    ma.markers.push_back(marker);
    return ma;
}

void OccupancyGrid::addPointsToField(
    const std::vector<Eigen::Vector3d>& points)
{
    EigenSTL::vector_Vector3d pts(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        pts[i] = Eigen::Vector3d(points[i].x(), points[i].y(), points[i].z());
    }
    grid_->addPointsToField(pts);
}

void OccupancyGrid::removePointsFromField(
    const std::vector<Eigen::Vector3d>& points)
{
    grid_->removePointsFromField(toAlignedVector(points));
}

void OccupancyGrid::updatePointsInField(
    const std::vector<Eigen::Vector3d>& old_points,
    const std::vector<Eigen::Vector3d>& new_points)
{
    grid_->updatePointsInField(toAlignedVector(old_points), toAlignedVector(new_points));
}

EigenSTL::vector_Vector3d OccupancyGrid::toAlignedVector(
    const std::vector<Eigen::Vector3d>& v) const
{
    EigenSTL::vector_Vector3d pts(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        pts[i] = Eigen::Vector3d(v[i].x(), v[i].y(), v[i].z());
    }
    return pts;
}

} // namespace sbpl
