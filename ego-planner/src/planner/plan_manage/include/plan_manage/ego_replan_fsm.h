#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <ego_planner/Bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

#include <std_msgs/Int32.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      GEN_NEW_TRAJ2,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      EMERGENCY_STOP2,
      RESOLVING_ISSUES
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    ego_planner::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    double thresh_running_traj;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_;

    ///////////////////
    ros::Publisher waypt_pub,multidof_pub,flag_pub;
    ros::Subscriber poi_sub;
    int is_poi;
    Eigen::Vector3d goal_pos_;
    std_msgs::Int32 flag;
    //////////////////


    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);


    /////////////////
    void make_count_zero();
    void isPOI(const std_msgs::Int32 &msg);
    Eigen::Vector3d Peturb(const Eigen::Vector3d curr_pos);

    bool checkCollision();

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif