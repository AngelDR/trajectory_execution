#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

class NodeClass {

private:

  ros::Publisher base_pub;
  ros::Publisher s2_pub;
  ros::Publisher e1_pub; 
  ros::Publisher e2_pub;
  ros::Publisher w1_pub; 
  ros::Publisher w2_pub;

public:
  ros::NodeHandle n;
  ros::Subscriber sub;
     //--- defining a callback function---
  void trajectoryCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
  {
    ROS_INFO("Recibida trayectoria");
    moveit_msgs::RobotTrajectory _robot_trajectory = msg->trajectory[0];
    ROS_INFO("Robot trajectory");
    trajectory_msgs::JointTrajectory _joint_trajectory = _robot_trajectory.joint_trajectory;
    ROS_INFO("Joint trajectory");
    std::vector<trajectory_msgs::JointTrajectoryPoint> _points = _joint_trajectory.points;
    ROS_INFO("Points vector");


    for(int i=1; i < _points.size(); i++){
      //publish data
      base_pub.publish((double) _points.at(i).positions[0]);
      s2_pub.publish((double) _points.at(i).positions[1]);
      e1_pub.publish((double) _points.at(i).positions[2]);
      e2_pub.publish((double) _points.at(i).positions[3]);
      w1_pub.publish((double) _points.at(i).positions[4]);
      w2_pub.publish((double) _points.at(i).positions[5]);
      ROS_INFO("Punto de trayectoria %d", i);
      ros::spinOnce;
      //sleep(1.0);
    }
  }

  //Constructor
  NodeClass()
  {
    base_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_6dof/joint_base_to_s2_position_controller/command", 1000);
    s2_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_6dof/joint_s2_to_s3_position_controller/command", 1000);
    e1_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_6dof/joint_e1_to_e2_position_controller/command", 1000);
    e2_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_6dof/joint_e2_to_w1_position_controller/command", 1000);
    w1_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_6dof/joint_w1_to_w2_position_controller/command", 1000);
    w2_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_6dof/joint_w2_to_eef_position_controller/command", 1000);

  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectoryExecution");
  NodeClass* a = new NodeClass();
  a->sub = a->n.subscribe("/move_group/display_planned_path", 1000, &NodeClass::trajectoryCallback,a);

  ros::spin();
  return 0;
}
