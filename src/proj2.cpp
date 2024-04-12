#include <tf2_ros/transform_listener.h>
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <global_planner/planner_core.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <rotate_recovery/rotate_recovery.h>
#include <base_local_planner/goal_functions.h>

geometry_msgs::PoseStamped global_goal_pose; 
geometry_msgs::PoseStamped global_start_pose;
std::vector<geometry_msgs::PoseStamped> global_plan;
geometry_msgs::Twist cmd_vel;
std_msgs::Float64 goal_distance;
bool new_point = false;
bool moving = false;

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose){
  global_goal_pose = *goal_pose;
  global_goal_pose.header.frame_id = "map";
  new_point = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "proj2");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  tf2_ros::Buffer tf_buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(tf_buffer);

  tf2_ros::Buffer tf_buffer_g;
  tf2_ros::TransformListener tfListener_g(tf_buffer_g);
  costmap_2d::Costmap2DROS global_costmap("global_costmap", tf_buffer_g);
  global_planner::GlobalPlanner global_planner;
  global_planner.initialize("GlobalPlanner", global_costmap.getCostmap(), "map");

  tf2_ros::Buffer tf_buffer_l;
  tf2_ros::TransformListener tfListener_l(tf_buffer_l);
  costmap_2d::Costmap2DROS local_costmap("local_costmap", tf_buffer_l);
  base_local_planner::TrajectoryPlannerROS local_planner;
  local_planner.initialize("LocalPlanner", &tf_buffer_l, &local_costmap);

  ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 10, goalCallback);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher status = n.advertise<std_msgs::Float64>("goal_distance", 1000);

  tf2_ros::Buffer tf_buffer_r;
  tf2_ros::TransformListener tfListener_r(tf_buffer_r);
  rotate_recovery::RotateRecovery recovery_behavior;
  recovery_behavior.initialize("RecoveryBehaviorRotate", &tf_buffer_r, &global_costmap, &local_costmap);

  tf2_ros::Buffer tf_buffer_rc;
  tf2_ros::TransformListener tfListener_rc(tf_buffer_rc);
  clear_costmap_recovery::ClearCostmapRecovery recovery_behavior_clear;
  recovery_behavior_clear.initialize("RecoveryBehaviorClear", &tf_buffer_rc, &global_costmap, &local_costmap);

  while(ros::ok()){
    global_costmap.getRobotPose(global_start_pose);
    if(new_point){
      ROS_INFO("Target coordinates: x=%f y=%f", global_goal_pose.pose.position.x, global_goal_pose.pose.position.y);
      global_planner.makePlan(global_start_pose, global_goal_pose, global_plan);
      if(global_plan.size()>0){
        global_planner.publishPlan(global_plan);
        local_planner.setPlan(global_plan);
        new_point = false;
        moving = true;
      }
      else{
        ROS_ERROR("Quo Vadis? Non Possum!");
        new_point = false;
      }
    }
    if(moving){
      if(local_planner.isGoalReached()){
        moving = false;
        goal_distance.data = 0.0;
        status.publish(goal_distance);
        ROS_INFO("I have arrived");
      }
      else{
        if(local_planner.computeVelocityCommands(cmd_vel)){
          vel_pub.publish(cmd_vel);
          goal_distance.data = base_local_planner::getGoalPositionDistance(global_start_pose, global_goal_pose.pose.position.x, global_goal_pose.pose.position.y);
        }
        else{
          ROS_INFO("I am preforming a recovery");
          recovery_behavior.runBehavior();
          recovery_behavior_clear.runBehavior();
          ROS_INFO("I should feel better");
        }
      }
    }
    status.publish(goal_distance);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return(0);
}