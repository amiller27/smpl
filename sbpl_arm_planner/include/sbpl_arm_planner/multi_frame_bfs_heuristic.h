#ifndef sbpl_manip_multi_frame_bfs_heuristic_h
#define sbpl_manip_multi_frame_bfs_heuristic_h

// standard includes
#include <memory>

// system includes
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_arm_planner/manip_heuristic.h>

namespace sbpl {
namespace manip {

class BFS_3D;

class MultiFrameBfsHeuristic : public ManipHeuristic
{
public:

    MultiFrameBfsHeuristic(
        ManipLattice* env,
        const OccupancyGridConstPtr& grid,
        const PlanningParams* params);

    virtual ~MultiFrameBfsHeuristic();

    bool setGoal(const GoalConstraint& goal);

    visualization_msgs::MarkerArray getWallsVisualization() const;
    visualization_msgs::MarkerArray getValuesVisualization() const;

    /// \sa ManipHeuristic::getMetricStartDistance
    double getMetricStartDistance(double x, double y, double z);

    /// \brief Return the metric distance of the planning link.
    double getMetricGoalDistance(double x, double y, double z);

    int GetGoalHeuristic(int state_id);
    int GetStartHeuristic(int state_id);
    int GetFromToHeuristic(int from_id, int to_id);

private:

    std::unique_ptr<BFS_3D> m_bfs;
    std::unique_ptr<BFS_3D> m_ee_bfs;

    int getGoalHeuristic(int state_id, bool use_ee) const;

    void syncGridAndBfs();
    int getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const;

    inline
    int combine_costs(int c1, int c2) const;
};

} // namespace manip
} // namespace sbpl

#endif
