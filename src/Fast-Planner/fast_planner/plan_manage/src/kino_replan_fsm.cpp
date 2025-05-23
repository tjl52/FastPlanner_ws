/**
* 该文件是 Fast-Planner 项目的一部分。
* 版权所有 2019 Boyu Zhou，香港科技大学空中机器人组。
* 详细版权和许可证信息见原文件头部。
*/

#include <plan_manage/kino_replan_fsm.h> // 引入头文件

namespace fast_planner { // 命名空间

// 初始化函数
void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0; // 当前航点索引初始化为0
  exec_state_  = FSM_EXEC_STATE::INIT; // FSM 状态初始化为 INIT
  have_target_ = false; // 是否有目标点初始化为 false
  have_odom_   = false; // 是否有里程计初始化为 false

  /* 读取 FSM 参数 */
  nh.param("fsm/flight_type", target_type_, -1); // 飞行类型
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0); // 重新规划距离阈值
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0); // 不重新规划距离阈值

  nh.param("fsm/waypoint_num", waypoint_num_, -1); // 航点数量
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0); // 读取航点x
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0); // 读取航点y
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0); // 读取航点z
  }

  /* 初始化主要模块 */
  planner_manager_.reset(new FastPlannerManager); // 轨迹规划管理器
  planner_manager_->initPlanModules(nh); // 初始化规划模块
  visualization_.reset(new PlanningVisualization(nh)); // 可视化模块

  /* 定时器和回调函数 */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this); // FSM 主循环定时器
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this); // 碰撞检测定时器

  waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this); // 航点订阅
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this); // 里程计订阅

  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10); // 重新规划话题
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10); // 新轨迹话题
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10); // B样条轨迹话题
}

// 航点回调函数
void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return; // 如果z坐标小于-0.1，忽略

  cout << "Triggered!" << endl; // 打印触发信息
  trigger_ = true; // 触发标志

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) { // 手动目标
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0; // 目标点z固定为1.0

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) { // 预设目标
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_; // 循环切换航点
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0)); // 可视化目标点
  end_vel_.setZero(); // 终点速度清零
  have_target_ = true; // 标记有目标

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG"); // 等待目标时收到目标，切换到生成新轨迹
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG"); // 执行轨迹时收到目标，切换到重新规划
}

// 里程计回调函数
void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x; // 位置x
  odom_pos_(1) = msg->pose.pose.position.y; // 位置y
  odom_pos_(2) = msg->pose.pose.position.z; // 位置z

  odom_vel_(0) = msg->twist.twist.linear.x; // 速度x
  odom_vel_(1) = msg->twist.twist.linear.y; // 速度y
  odom_vel_(2) = msg->twist.twist.linear.z; // 速度z

  odom_orient_.w() = msg->pose.pose.orientation.w; // 姿态四元数w
  odom_orient_.x() = msg->pose.pose.orientation.x; // x
  odom_orient_.y() = msg->pose.pose.orientation.y; // y
  odom_orient_.z() = msg->pose.pose.orientation.z; // z

  have_odom_ = true; // 标记有里程计
}

// 状态切换函数
void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" }; // 状态字符串
  int    pre_s        = int(exec_state_); // 之前的状态
  exec_state_         = new_state; // 切换到新状态
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl; // 打印状态切换信息
}

// 打印当前状态
void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl; // 打印当前状态
}

// FSM 主循环回调
void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0; // 静态计数器
  fsm_num++;
  if (fsm_num == 100) { // 每100次打印一次状态
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) { // 根据当前状态执行不同逻辑
    case INIT: { // 初始化状态
      if (!have_odom_) { // 没有里程计则返回
        return;
      }
      if (!trigger_) { // 没有触发则返回
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM"); // 切换到等待目标状态
      break;
    }

    case WAIT_TARGET: { // 等待目标状态
      if (!have_target_)
        return; // 没有目标则返回
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // 有目标则切换到生成新轨迹
      }
      break;
    }

    case GEN_NEW_TRAJ: { // 生成新轨迹状态
      start_pt_  = odom_pos_; // 起点为当前里程计位置
      start_vel_ = odom_vel_; // 起始速度
      start_acc_.setZero(); // 起始加速度清零

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1); // 计算当前朝向
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0)); // 计算起始偏航角
      start_yaw_(1) = start_yaw_(2) = 0.0; // 偏航角速度、加速度清零

      bool success = callKinodynamicReplan(); // 调用动力学可行轨迹规划
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM"); // 成功则切换到执行轨迹
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // 失败则继续尝试
      }
      break;
    }

    case EXEC_TRAJ: { // 执行轨迹状态
      /* 判断是否需要重新规划 */
      LocalTrajData* info     = &planner_manager_->local_data_; // 局部轨迹数据
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
      t_cur                   = min(info->duration_, t_cur); // 当前轨迹时间

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur); // 当前轨迹点

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) { // 如果轨迹执行完毕
        have_target_ = false; // 清除目标
        changeFSMExecState(WAIT_TARGET, "FSM"); // 切换到等待目标
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) { // 距离终点很近
        // cout << "near end" << endl;
        return;

      } else if ((info->start_pos_ - pos).norm() < replan_thresh_) { // 距离起点很近
        // cout << "near start" << endl;
        return;

      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM"); // 其他情况进入重新规划
      }
      break;
    }

    case REPLAN_TRAJ: { // 重新规划状态
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur); // 当前点
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur); // 当前速度
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur); // 当前加速度

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0]; // 当前偏航角
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0]; // 当前偏航角速度
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0]; // 当前偏航角加速度

      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg); // 发布重新规划消息

      bool success = callKinodynamicReplan(); // 调用动力学可行轨迹重新规划
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM"); // 成功则切换到执行轨迹
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // 失败则切换到生成新轨迹
      }
      break;
    }
  }
}

// 碰撞检测回调
void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) { // 有目标时检查目标点安全
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.3) { // 目标点距离障碍物过近
      /* 在周围寻找最大距离的安全点 */
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* 重设目标点 */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) { // 找到更安全的点
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY"); // 切换到重新规划
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0)); // 可视化新目标
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM"); // 继续重新规划

        std_msgs::Empty emt;
        replan_pub_.publish(emt); // 发布重新规划消息
      }
    }
  }

  /* ---------- 检查轨迹安全 ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist); // 检查轨迹碰撞

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY"); // 进入重新规划
    }
  }
}

// 动力学可行轨迹规划函数
bool KinoReplanFSM::callKinodynamicReplan() {
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_); // 调用动力学可行轨迹规划

  if (plan_success) {

    planner_manager_->planYaw(start_yaw_); // 规划偏航角

    auto info = &planner_manager_->local_data_;

    /* 发布轨迹 */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt); // 添加控制点
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i)); // 添加节点
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw); // 添加偏航角控制点
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval(); // 偏航角间隔

    bspline_pub_.publish(bspline); // 发布 B样条轨迹

    /* 可视化 */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4)); // 可视化动力学路径
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1)); // 可视化B样条

    return true; // 规划成功

  } else {
    cout << "generate new traj fail." << endl;
    return false; // 规划失败
  }
}

// KinoReplanFSM::
}  // namespace fast_planner
