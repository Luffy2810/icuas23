
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/traj_thresh", thresh_running_traj, 1.0);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    //////
    waypt_pub = nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 100);
    multidof_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/red/position_hold/trajectory", 10);
    poi_sub = nh.subscribe("/is_poi", 1, &EGOReplanFSM::isPOI, this);
    flag_pub = nh.advertise<std_msgs::Int32>("/resolve_issue", 10); // notify whenever publishing directly to /red/position_hold/trajectory
    goal_pos_(0) = 0;
    goal_pos_(1) = 0;
    goal_pos_(2) = 0;
    is_poi = 1;
    //////

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

//////////////////////
  void EGOReplanFSM::isPOI(const std_msgs::Int32 &msg)
  {
    is_poi = msg.data;
  }
//////////////////////


  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();
    }
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)
      return;
    goal_pos_(0) = msg->poses[0].pose.position.x;
    goal_pos_(1) = msg->poses[0].pose.position.y;
    goal_pos_(2) = msg->poses[0].pose.position.z;
    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, msg->poses[0].pose.position.z;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[10] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "GEN_NEW_TRAJ2", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "EMERGENCY_STOP2", "RESOLVING_ISSUES"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[10] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "GEN_NEW_TRAJ2", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "EMERGENCY_STOP2", "RESOLVING_ISSUES"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::make_count_zero()
  {
    planner_manager_->bspline_optimizer_rebound_->count_flag[0] = 0;
    planner_manager_->bspline_optimizer_rebound_->count_flag[1] = 0;
    planner_manager_->bspline_optimizer_rebound_->count_flag[2] = 0;
  }

  Eigen::Vector3d EGOReplanFSM::Peturb(const Eigen::Vector3d curr_pos)
  {
    int obs_flag = 1;
    double x,y,z,rad;
    Eigen::Vector3d pos;
    double tmp_x = curr_pos(0), tmp_y = curr_pos(1), tmp_z = curr_pos(2);
    do
    {
      int k;
      if (flag.data == 1)
      {
        k = 4;
      }
      else
      {
        k = 2;
      }
      x = rand();
      y = rand();
      z = rand();
      rad = sqrt(x*x + y*y + z*z);
      x = k*(2*x/rad -1);
      y = k*(2*y/rad -1);
      z = k*(2*z/rad -1);
      tmp_x =curr_pos(0)+ x;
      tmp_y = curr_pos(1) + y;
      tmp_z = curr_pos(2) + z;
      pos = {tmp_x,tmp_y,tmp_z};
      obs_flag = planner_manager_->grid_map_->getInflateOccupancy(pos);
    }
    while(obs_flag ==1);
    return pos;
  }


  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    static int fsm_num = 0;
    static int count2 = -10;
    fsm_num++;
    // count++;
    // count2++;
    static int count = 0;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }
    
    case RESOLVING_ISSUES:
    {
      if(count >=300)
      {
        nav_msgs::Path path;
        std::cout<<"I AM IN TOO MUCH GEN_NEW_TRAJ   "<<std::endl;

        geometry_msgs::PoseStamped pose;
        double peturb_x = goal_pos_(0);
        double peturb_y = goal_pos_(1);
        double peturb_z = goal_pos_(2);
        Eigen::Vector3d curr_pos = {peturb_x, peturb_y, peturb_z};
        Eigen::Vector3d peturb_pos = Peturb(curr_pos);
        // pose.header.stamp = ...;
        // pose.header.frame_id = ...;
        pose.pose.position.x = peturb_pos(0);
        pose.pose.position.y = peturb_pos(1);
        pose.pose.position.z = goal_pos_(2);
        // path.header.stamp = ...;
        // path.header.frame_id = ...;
        path.poses.push_back(pose);
        waypt_pub.publish(path);
        ros::Duration(0.5).sleep();
        planner_manager_->bspline_optimizer_rebound_->count_flag[2] = 0;
        count = 0;
        changeFSMExecState(GEN_NEW_TRAJ, "TOO MUCH GEN_NEW_TRAJ");
        return;
      }
      count = 0;
      std::cout<<"I AM OUTSIDE  "<<std::endl;
      if(planner_manager_->bspline_optimizer_rebound_->count_flag[2] >= 150){
          nav_msgs::Path path;
          std::cout<<"I AM IN TERMINALPOINT OF RESOLVING_ISSUES  "<<std::endl;

          geometry_msgs::PoseStamped pose;
          double peturb_x = goal_pos_(0);
          double peturb_y = goal_pos_(1);
          double peturb_z = goal_pos_(2);
          Eigen::Vector3d curr_pos = {peturb_x, peturb_y, peturb_z};
          Eigen::Vector3d peturb_pos = Peturb(curr_pos);
          // pose.header.stamp = ...;
          // pose.header.frame_id = ...;
          pose.pose.position.x = peturb_pos(0);
          pose.pose.position.y = peturb_pos(1);
          pose.pose.position.z = goal_pos_(2);
          // path.header.stamp = ...;
          // path.header.frame_id = ...;
          path.poses.push_back(pose);
          waypt_pub.publish(path);
          ros::Duration(0.5).sleep();
          planner_manager_->bspline_optimizer_rebound_->count_flag[2] = 0;
          changeFSMExecState(GEN_NEW_TRAJ, "TERMINAL POINT IN OBSTACLE");
          return;
      }
      else if(planner_manager_->bspline_optimizer_rebound_->count_flag[1] >= 25){
        if(is_poi == 1)
        {         
         
          //   int size = 1000;
          //   planner_manager_->grid_map_->clearLocalOdomMap(odom_pos_, size);
          //     // planner_manager_.reset(new EGOPlaprev_inflate_obstaclennerManager);
          //     // planner_manager_->initPlanModules(nh, visualization_);       
          //   // count = 0;
          // 

          std::cout<<"I AM HERE IN Drone In Obstacle OF RESOLVING ISSUES"<<std::endl;
          int size = 1000;
          // prev_inflate_obstacle =  planner_manager_->grid_map_->mp_.obstacles_inflation_;
          planner_manager_->grid_map_->mp_.obstacles_inflation_ = 0.2;
          planner_manager_->grid_map_->clearAndInflateLocalMap();
          std::cout<<"I HAVE INFLATED OBSTACLES"<<std::endl;
          count2 = 1;
          planner_manager_->bspline_optimizer_rebound_->count_flag[1] = 0;
          std::cout<<"I AM Here  "<<std::endl;
          ros::Duration(0.02).sleep();
          changeFSMExecState(GEN_NEW_TRAJ, "DRONEINOBSTACLE");
          return;
        }else{
          
          flag.data = 1;
          flag_pub.publish(flag);
          std::cout<<"I AM HERE IN Drone In Obstacle OF RESOLVING ISSUES  "<<std::endl;
          double peturb_x = odom_pos_(0);
          double peturb_y = odom_pos_(1);
          double peturb_z = odom_pos_(2);
          nav_msgs::Path path;
          geometry_msgs::PoseStamped pose;
          Eigen::Vector3d curr_pos = {peturb_x, peturb_y, peturb_z};
          Eigen::Vector3d peturb_pos = Peturb(curr_pos);
          pose.pose.position.x = peturb_pos(0);
          pose.pose.position.y = peturb_pos(1);
          pose.pose.position.z = odom_pos_(2);
          // path.header.stamp = ...;
          // path.header.frame_id = ...;
          path.poses.push_back(pose);
          waypt_pub.publish(path);
          ros::Duration(0.5).sleep();
          planner_manager_->grid_map_->mp_.obstacles_inflation_ = 0.2;
          planner_manager_->grid_map_->clearAndInflateLocalMap();
          std::cout<<"I HAVE INFLATED OBSTACLES"<<std::endl;
          count2 = 1;
          // Tried to publish directly to topic through multidof but no success
          // Eigen::Vector3d curr_pos = {peturb_x, peturb_y, peturb_z};
          // Eigen::Vector3d peturb_pos = Peturb(curr_pos);
          // trajectory_msgs::MultiDOFJointTrajectoryPoint pt;
          // geometry_msgs::Transform transform;
          // geometry_msgs::Twist vel, accel;
          // double kp=1, kd = 1;
          // double err[3] ;
          // double prev_err[3];
          // // pose.header.stamp = ...;
          // // pose.header.frame_id = ...;
          // transform.translation.x = peturb_pos(0);
          // transform.translation.y = peturb_pos(1);
          // transform.translation.z = peturb_z;
          // pt.transforms.push_back(transform);
          // vel.linear.x = 0;
          // vel.linear.y = 0;
          // vel.linear.z = 0;
          // accel.linear.x = 0;
          // accel.linear.y = 0;
          // accel.linear.z = 0;
          // pt.velocities.push_back(vel);
          // pt.accelerations.push_back(accel);
          // pt.time_from_start = ros::Duration(1);
          std::cout<<peturb_pos<<std::endl;
          std::cout<<"Going To the above locations"<<std::endl;
          // multidof_pub.publish(pt);
          // err[0] = peturb_pos(0)-odom_pos_(0);
          // err[1] = peturb_pos(1)-odom_pos_(1);
          // err[2] = peturb_z-odom_pos_(2);
          // while (err[0]>0.1 || err[1]>0.1 || err[2]>0.1)
          // {

          //   prev_err[0] = err[0];
          //   prev_err[1] = err[1];
          //   prev_err[2] = err[2];
          //   err[0] = peturb_pos(0)-odom_pos_(0);
          //   err[1] = peturb_pos(1)-odom_pos_(1);
          //   err[2] = peturb_z-odom_pos_(2);
          //   pt.velocities[0].linear.x = kp*err[0] + kd*(err[0] - prev_err[0]);
          //   pt.velocities[0].linear.y = kp*err[1] + kd*(err[1] - prev_err[1]);
          //   pt.velocities[0].linear.z = kp*err[2] + kd*(err[2] - prev_err[2]);
          //   multidof_pub.publish(pt);
          //   ros::Duration(0.02).sleep();

          // }
          // path.header.stamp = ...;
          // path.header.frame_id = ...;
          // ros::Duration(0.5).sleep();
          planner_manager_->bspline_optimizer_rebound_->count_flag[1] = 0;
          flag.data = 0;
          flag_pub.publish(flag);
          changeFSMExecState(GEN_NEW_TRAJ, "DRONEINOBSTACLE");
          return;
        }
      }
      else
      { 
        flag.data = 1;
        flag_pub.publish(flag);
        std::cout<<"I AM HERE IN 1st three control points OF RESOLVING ISSUES  "<<std::endl;
        double peturb_x = odom_pos_(0);
        double peturb_y = odom_pos_(1);
        double peturb_z = odom_pos_(2);
        Eigen::Vector3d curr_pos = {peturb_x, peturb_y, peturb_z};
        Eigen::Vector3d peturb_pos = Peturb(curr_pos);
        nav_msgs::Path path;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = peturb_pos(0);
        pose.pose.position.y = peturb_pos(1);
        pose.pose.position.z = odom_pos_(2);
        // path.header.stamp = ...;
        // path.header.frame_id = ...;
        path.poses.push_back(pose);
        waypt_pub.publish(path);
        ros::Duration(0.5).sleep();
        // trajectory_msgs::MultiDOFJointTrajectoryPoint pt;
        // geometry_msgs::Transform transform;
        // geometry_msgs::Twist vel, accel;
        // double kp=1.5, kd = 1;
        // double err[3] ;
        // double prev_err[3];
        // // pose.header.stamp = ...;
        // // pose.header.frame_id = ...;
        // transform.translation.x = peturb_pos(0);
        // transform.translation.y = peturb_pos(1);
        // transform.translation.z = peturb_z;
        // pt.transforms.push_back(transform);
        // vel.linear.x = 0;
        // vel.linear.y = 0;
        // vel.linear.z = 0;
        // accel.linear.x = 0;
        // accel.linear.y = 0;
        // accel.linear.z = 0;
        // pt.velocities.push_back(vel);
        // pt.accelerations.push_back(accel);
        // pt.time_from_start = ros::Duration(1);

        std::cout<<peturb_pos<<std::endl;
        std::cout<<"Going to the above locations"<<std::endl;
        // multidof_pub.publish(pt);
        // err[0] = peturb_pos(0)-odom_pos_(0);
        // err[1] = peturb_pos(1)-odom_pos_(1);
        // err[2] = peturb_z-odom_pos_(2);
        // while (err[0]>0.1 || err[1]>0.1 || err[2]>0.1)
        // {

        //   prev_err[0] = err[0];
        //   prev_err[1] = err[1];
        //   prev_err[2] = err[2];
        //   err[0] = peturb_pos(0)-odom_pos_(0);
        //   err[1] = peturb_pos(1)-odom_pos_(1);
        //   err[2] = peturb_z-odom_pos_(2);
        //   pt.velocities[0].linear.x = kp*err[0] + kd*(err[0] - prev_err[0]);
        //   pt.velocities[0].linear.y = kp*err[1] + kd*(err[1] - prev_err[1]);
        //   pt.velocities[0].linear.z = kp*err[2] + kd*(err[2] - prev_err[2]);
        //   multidof_pub.publish(pt);
        //   ros::Duration(0.02).sleep();


        // }
        // // path.header.stamp = ...;
        // // path.header.frame_id = ...;
        // ros::Duration(0.5).sleep();
        planner_manager_->bspline_optimizer_rebound_->count_flag[0] = 0;
        flag.data = 0;
        flag_pub.publish(flag);
        changeFSMExecState(GEN_NEW_TRAJ, "FIRST3CONTROLPOINTS");
        return;
      }
      // else if ()
      // {

      // }
      // if(count > 200 ){
             
      //   // count = 0;
      // }
    }

    case WAIT_TARGET:
    {
      count = 0;
      make_count_zero();
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    

    case GEN_NEW_TRAJ:
    {
      if(count2 > 0)
        {count2++;}
      if(count2 > 35){
        planner_manager_->grid_map_->mp_.obstacles_inflation_ = 0.6;
        planner_manager_->grid_map_->clearAndInflateLocalMap();
        count2 = -10;
      }
      if (count > 300)
      {
        ROS_WARN("BOHOT SAARA GEN_NEW_TRAJ");
        changeFSMExecState(RESOLVING_ISSUES, "TOO MUCH GEN NEW TRAJ");
        return;
      }

      count ++;
      // int thresh = 25;
      if(planner_manager_->bspline_optimizer_rebound_->count_flag[0] >= 150)
      {
        ROS_WARN("THE VALUE OF FIRST THREE CONTROL POINTS COUNT: %f", float(planner_manager_->bspline_optimizer_rebound_->count_flag[0]));
        changeFSMExecState(RESOLVING_ISSUES, "CONTROL POINT ERROR");
        return;
      }
      if(planner_manager_->bspline_optimizer_rebound_->count_flag[1] >= 25)
      {
        ROS_WARN("THE VALUE OF DRONE IN OBSTACLE COUNT: %f", float(planner_manager_->bspline_optimizer_rebound_->count_flag[1]));
        changeFSMExecState(RESOLVING_ISSUES, "DRONE IN OBSTACLE ERROR");
        return;
      }
      if(planner_manager_->bspline_optimizer_rebound_->count_flag[2] >= 150)
      {
        ROS_WARN("THE VALUE OF TERMINALPOINT IN OBSTACLE COUNT: %f", float(planner_manager_->bspline_optimizer_rebound_->count_flag[2]));
        changeFSMExecState(RESOLVING_ISSUES, "TERMINAL POINT ERROR");
        return;
      }

        // 1. Drone in obstacle - obstacle inflation
        // 2. Terminal point - shift the point 
        // 3. First three control points  - obstacle inflation/ OR CHECK MORE
        // 4. Any misc problem - move the drone to obstacle free using closed loop control.
        
        // make publisher for requesting point
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ2:
    {
      // if(count > 200 ){
      //   int size = 1000;
      //   planner_manager_->grid_map_->clearLocalOdomMap(odom_pos_, size);
      //     // planner_manager_.reset(new EGOPlaprev_inflate_obstaclennerManager);
      //     // planner_manager_->initPlanModules(nh, visualization_);       
      //   // count = 0;
      // }
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      count = 0;
      make_count_zero();
      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      count = 0;
      make_count_zero();
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((odom_pos_ - info->start_pos_).norm() > thresh_running_traj)
      {
        changeFSMExecState(EMERGENCY_STOP2, "Trajectory_Ran_Away");
        info->start_time_ = ros::Time::now();
        return;
      }

      if (t_cur > info->duration_ - 1e-2)
      {
        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        //cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        //cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    case EMERGENCY_STOP2:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.2)
          changeFSMExecState(GEN_NEW_TRAJ2, "FSM");
        else 
        {
          changeFSMExecState(EMERGENCY_STOP, "FSM");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    // start_pt_ = odom_pos_
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // todo
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }

} // namespace ego_planner

