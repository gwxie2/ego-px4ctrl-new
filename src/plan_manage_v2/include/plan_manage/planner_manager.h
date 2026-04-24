#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <string>
#include <stdlib.h>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils_v2/DataDisp.h>
#include <plan_env/grid_map.h>
#include <traj_utils/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const double &ts, poly_traj::MinJerkOpt &initMJO);
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const bool touch_goal);
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    void getLocalTarget(
        const double planning_horizen,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel,
        bool &touch_goal);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool checkCollision(int drone_id);
    bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal);
    inline double getSwarmClearance(void) { return ploy_traj_opt_->get_swarm_clearance_(); }
    inline int getCpsNumPrePiece(void) { return ploy_traj_opt_->get_cps_num_prePiece_(); }
    inline double getLastReplanSearchMs(void) const { return last_replan_search_ms_; }
    inline double getLastReplanOptimizeMs(void) const { return last_replan_optimize_ms_; }
    inline double getLastReplanAdjustMs(void) const { return last_replan_adjust_ms_; }
    inline double getLastReplanTotalMs(void) const { return last_replan_total_ms_; }
    inline int getLastReplanIterCount(void) const { return last_replan_iter_count_; }
    inline uint8_t getLastFailureReason(void) const { return last_failure_reason_; }
    inline const std::string &getLastFailureDetail(void) const { return last_failure_detail_; }
    inline int getLastAStarExpandedNodes(void) const { return last_astar_expanded_nodes_; }
    inline double getLastGradientNormFinal(void) const { return last_gradient_norm_final_; }
    inline double getLastCostInitial(void) const { return last_cost_initial_; }
    inline double getLastCostFinal(void) const { return last_cost_final_; }
    // inline PtsChk_t getPtsCheck(void) { return ploy_traj_opt_->get_pts_check_(); }

    PlanParameters pp_;
    GridMap::Ptr grid_map_;
    TrajContainer traj_;

  private:
    PlanningVisualization::Ptr visualization_;

    PolyTrajOptimizer::Ptr ploy_traj_opt_;

    int continous_failures_count_{0};

    double last_replan_search_ms_{0.0};
    double last_replan_optimize_ms_{0.0};
    double last_replan_adjust_ms_{0.0};
    double last_replan_total_ms_{0.0};
    int last_replan_iter_count_{0};
    uint8_t last_failure_reason_{0};
    std::string last_failure_detail_;
    std::string last_init_state_failure_detail_;
    int last_astar_expanded_nodes_{0};
    double last_gradient_norm_final_{0.0};
    double last_cost_initial_{0.0};
    double last_cost_final_{0.0};

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif