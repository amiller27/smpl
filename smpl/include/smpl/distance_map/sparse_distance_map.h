////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
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

#ifndef SMPL_SPARSE_DISTANCE_MAP_H
#define SMPL_SPARSE_DISTANCE_MAP_H

// standard includes
#include <array>
#include <utility>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <Eigen/StdVector>

// project includes
#include <smpl/grid/sparse_grid.h>
#include <smpl/distance_map/distance_map_interface.h>
#include "detail/distance_map_common.h"

namespace sbpl {

class SparseDistanceMap : public DistanceMapInterface
{
public:

    SparseDistanceMap(
        double origin_x, double origin_y, double origin_z,
        double size_x, double size_y, double size_z,
        double resolution,
        double max_dist);

    double maxDistance() const;

    double getDistance(double x, double y, double z) const;
    double getDistance(int x, int y, int z) const;

    bool isCellValid(const Eigen::Vector3i& gp) const;

    /// \name Required Functions from DistanceMapInterface
    ///@{
    DistanceMapInterface* clone() const override;

    void addPointsToMap(const std::vector<Eigen::Vector3d>& points) override;
    void removePointsFromMap(const std::vector<Eigen::Vector3d>& points) override;
    void updatePointsInMap(
        const std::vector<Eigen::Vector3d>& old_points,
        const std::vector<Eigen::Vector3d>& new_points) override;

    void reset() override;

    int numCellsX() const override;
    int numCellsY() const override;
    int numCellsZ() const override;

    double getUninitializedDistance() const override;

    double getMetricDistance(double x, double y, double z) const override;
    double getCellDistance(int x, int y, int z) const override;

    double getMetricSquaredDistance(double x, double y, double z) const override;
    double getCellSquaredDistance(int x, int y, int z) const override;

    void gridToWorld(
        int x, int y, int z,
        double& world_x, double& world_y, double& world_z) const override;

    void worldToGrid(
        double world_x, double world_y, double world_z,
        int& x, int& y, int& z) const override;

    bool isCellValid(int x, int y, int z) const override;
    ///@}

public:

    struct Cell
    {
        int ox;
        int oy;
        int oz;

        int dist;
        int dist_new;
#if SMPL_DMAP_RETURN_CHANGED_CELLS
        int dist_old;
#endif
        Cell* obs;
        int bucket;
        int dir;

        int pos;

        // NOTE: vacuous true here for interoperability with SparseGrid::prune.
        // This shouldn't be used to do unconditional pruning, but should be
        // used in conjunction with conditional pruning to remove cells with
        // unknown nearest obstacles, and which must not be referred to by any
        // other cell as its nearest obstacle.
        bool operator==(const Cell& rhs) const { return true; }
    };

    SparseGrid<Cell> m_cells;

    int m_cell_count_x;
    int m_cell_count_y;
    int m_cell_count_z;

    // max propagation distance in world units
    double m_max_dist;
    double m_inv_res;

    // max propagation distance in cells
    int m_dmax_int;
    int m_dmax_sqrd_int;

    int m_bucket;

    int m_no_update_dir;

    // Direction offsets to each of the 27 neighbors, including (0, 0, 0).
    // Indexed by a call to dirnum(x, y, z, 0);
    std::array<Eigen::Vector3i, 27> m_neighbors;

    // Storage for the indices of neighbor offsets that must have distance
    // information propagated to them, given the source's update direction. The
    // indices are arranged so that target neighbor indices for a given source
    // update direction are contiguous

    // [ s_1_t_1, ..., s_1_t_n, s_2_t_1, ..., s_2_t_n, ..., s_n_t_1, ..., s_n_t_n ]
    // where n = 2 * 27
    std::array<int, NEIGHBOR_LIST_SIZE> m_indices;

    // Map from a source update direction (obtained from dirnum(x, y, z, e)) to
    // a range of neighbor offsets (indices into m_neighbors) denoting neighbors
    // to which distance values must be propagated upon insertion
    std::array<std::pair<int, int>, NUM_DIRECTIONS> m_neighbor_ranges;

    // Map from a (source, target) update direction pair to the update direction
    // index
    std::array<int, NEIGHBOR_LIST_SIZE> m_neighbor_dirs;

    std::vector<double> m_sqrt_table;

    struct bucket_element
    {
        Cell* c;
        int x;
        int y;
        int z;

        bucket_element() { }
        bucket_element(Cell* c, int x, int y, int z) :
            c(c), x(x), y(y), z(z)
        { }
    };

    typedef std::vector<bucket_element> bucket_type;
    typedef std::vector<bucket_type> bucket_list;
    bucket_list m_open;

    std::vector<bucket_element> m_rem_stack;

    double m_error;

    void updateVertex(Cell* c, int cx, int cy, int cz);

    /// DistanceMap
    ///@{
    int distance(int nx, int ny, int nz, const Cell& s);

    void lower(Cell* s, int sx, int sy, int sz);
    void raise(Cell* s, int sx, int sy, int sz);
    void waveout(Cell* n, int nx, int ny, int nz);
    void propagate();

    void lowerBounded(Cell* s, int sx, int sy, int sz);
    void propagateRemovals();
    void propagateBorder();
    ///@}

    double getTrueMetricSquaredDistance(double x, double y, double z) const;
    double getInterpMetricSquaredDistance(double x, double y, double z) const;
};

} // namespace sbpl

#endif
