/**
 * @class TopoReplanFSM
 * @brief Fast-Planner 中用于拓扑重规划的有限状态机（FSM）。
 *
 * 本类负责管理基于拓扑引导和B样条优化的动力学路径规划的高层重规划逻辑。
 * 通过ROS接口接收航点、里程计信息，并发布规划轨迹。
 *
 * 主要功能：
 * - 支持多种目标类型（手动、预设、参考路径）。
 * - 管理FSM状态：初始化、等待目标、生成新轨迹、重规划、执行轨迹、基于新信息重规划。
 * - 集成FastPlannerManager进行路径规划，集成PlanningVisualization进行可视化。
 * - 支持碰撞检测、轨迹执行，以及基于距离和时间阈值的重规划触发。
 * - 提供ROS接口用于通信与可视化。
 *
 * 用法：
 * - 使用init()函数并传入ROS节点句柄进行初始化。
 * - FSM的执行通过ROS定时器和回调函数管理。
 *
 * @note 本类为香港科技大学Aerial Robotics Group开发的Fast-Planner项目的一部分。
 */
/**
* 本文件属于Fast-Planner项目。
*
* 版权所有 2019 周博宇，香港科技大学Aerial Robotics Group，<uav.ust.hk>
* 开发者：周博宇 <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* 更多信息请见 <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>。
* 如果您使用本代码，请引用上述网站列出的相关论文。
*
* Fast-Planner是自由软件：您可以根据自由软件基金会发布的GNU宽通用公共许可证（LGPL）第3版，
* 或（由您选择）任何更高版本，重新分发和/或修改本软件。
*
* Fast-Planner的发布希望对您有用，但不提供任何担保，
* 甚至不保证适销性或特定用途适用性。详情请参阅GNU通用公共许可证。
*
* 您应该已收到随Fast-Planner一起分发的GNU宽通用公共许可证副本。
* 如果没有，请参见 <http://www.gnu.org/licenses/>。
*/



#ifndef _TOPO_REPLAN_FSM_H_
#define _TOPO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace fast_planner {

class TopoReplanFSM {
private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double replan_distance_threshold_, replan_time_threshold_;
  double waypoints_[50][3];
  int waypoint_num_;
  bool act_map_;

  /* planning data */
  bool trigger_, have_target_, have_odom_, collide_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d target_point_, end_vel_;                        // target state
  int current_wp_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /* helper functions */
  bool callSearchAndOptimization();    // front-end and back-end method
  bool callTopologicalTraj(int step);  // topo path guided gradient-based
                                       // optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent& e);
  void checkCollisionCallback(const ros::TimerEvent& e);
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

public:
  TopoReplanFSM(/* args */) {}
  ~TopoReplanFSM() {}

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif