/* Author: Angel Delgado */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <std_msgs/Float64.h>

// PI
#include <boost/math/constants/constants.hpp>
#include <math.h> 
#include <tf/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
    ros::init (argc, argv, "shadow_pa10_trajectory_foam");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    // Robot Model
    /**robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
   
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("pa10_shadow");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();*/

    // Move Group ::Planning scene
    moveit::planning_interface::MoveGroup group("pa10_shadow");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);  
    moveit_msgs::DisplayTrajectory display_trajectory_0,display_trajectory_1,display_trajectory_2,display_trajectory_3;
    moveit::planning_interface::MoveGroup::Plan plan_0,plan_1,plan_2,plan_3;
    std::vector<double> group_variable_values;
    group.setPoseReferenceFrame ("link_base");
    group.setGoalOrientationTolerance (0.17);  
    group.setGoalPositionTolerance (0.05); 
    group.setPlannerId("KPIECE");

    double angle_x_roll = 1.57;
    double angle_y_pitch = 0;
    double angle_z_yaw = 0;

    // Ejemplo Coordenadas articulares:
    /**
    double joint_position_list[9] = {1.27, -0.55, 0.0, 0.98, 0.78, 0.82, 0.10, 0.0, 0.0};
    for(int i = 0; i < 10; i++ ){
        group_variable_values.push_back(joint_position_list[i]);
    }
    //bool    setJointValueTarget (const std::vector< double > &group_variable_values)
    //group.setJointValueTarget(group_variable_values);
    */


    // PUNTO REFERENCIA 0 : ENCIMA PLANTILLA (Coordenadas cartesianas)

    geometry_msgs::Pose target_pose0;
    group.setStartStateToCurrentState();
    target_pose0.orientation =  tf::createQuaternionMsgFromRollPitchYaw(angle_x_roll,angle_y_pitch,angle_z_yaw);
    target_pose0.position.x = -0.10;
    target_pose0.position.y = -0.60;
    target_pose0.position.z = 0.70;
    group.setPoseTarget(target_pose0,"palm");
    //group.setPositionTarget (0.0, -0.5, 0.5, "palm");
    //plan: publish display planned path-> mover robot si conexion_pa10=1
    group.plan(plan_0);
    ROS_INFO("Visualizing plan 0");
    display_trajectory_0.trajectory_start = plan_0.start_state_;
    display_trajectory_0.trajectory.push_back(plan_0.trajectory_);
    display_publisher.publish(display_trajectory_0);
    group.execute(plan_0);

    sleep(5.0);



    // PUNTO REFERENCIA 1 : AGARRE PLANTILLA (Coordenadas cartesianas, cartesian path)

    std::vector<geometry_msgs::Pose> waypoints_1;
    moveit_msgs::RobotTrajectory trajectory_msg_1;
    group. setPoseReferenceFrame("link_base");
    group.setStartStateToCurrentState();
    geometry_msgs::Pose initial_pose_1 = group.getCurrentPose().pose;
    group.setPlanningTime(3.0);
    waypoints_1.push_back(initial_pose_1);  
    geometry_msgs::Pose waypoint_pose_1;
    
    waypoint_pose_1.orientation = initial_pose_1.orientation;
    waypoint_pose_1.position.x = initial_pose_1.position.x;
    waypoint_pose_1.position.y = initial_pose_1.position.y;
    waypoint_pose_1.position.z = initial_pose_1.position.z - 0.10;
    waypoints_1.push_back(waypoint_pose_1);
    
      
    double fraction = group.computeCartesianPath(waypoints_1, 0.2, 100.0, trajectory_msg_1, false);
    ROS_INFO("Computed cartesian path %f", fraction);
    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "pa10_shadow");

    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg_1);
  
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory_msg_1);
    // Check trajectory_msg for velocities not empty
    //std::cout << trajectory_msg << std::endl;

    plan_1.trajectory_ = trajectory_msg_1;
    //group.plan(plan_4);
   
    ROS_INFO("Visualizing plan 1");
    display_trajectory_1.trajectory_start = plan_1.start_state_;
    display_trajectory_1.trajectory.push_back(plan_1.trajectory_);
    display_publisher.publish(display_trajectory_1);
    /* Sleep to give Rviz time to visualize the plan. */
    group.execute(plan_1);

    sleep(5.0);


    // PUNTO REFERENCIA 2 : ENCIMA PLANTILLA (Coordenadas cartesianas, cartesian path)

    std::vector<geometry_msgs::Pose> waypoints_2;
    moveit_msgs::RobotTrajectory trajectory_msg_2;
    group. setPoseReferenceFrame("link_base");
    group.setStartStateToCurrentState();
    geometry_msgs::Pose initial_pose_2 = group.getCurrentPose().pose;
    group.setPlanningTime(3.0);
    waypoints_2.push_back(initial_pose_2);  
    geometry_msgs::Pose waypoint_pose_2;
    
    waypoint_pose_2.orientation = initial_pose_2.orientation;
    waypoint_pose_2.position.x = initial_pose_2.position.x;
    waypoint_pose_2.position.y = initial_pose_2.position.y;
    waypoint_pose_2.position.z = initial_pose_2.position.z + 0.20;
    waypoints_2.push_back(waypoint_pose_2);
    
    fraction = group.computeCartesianPath(waypoints_2, 0.2, 100.0, trajectory_msg_2, false);
    ROS_INFO("Computed cartesian path %f", fraction);
    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt_2(group.getCurrentState()->getRobotModel(), "pa10_shadow");

    // Second get a RobotTrajectory from trajectory
    rt_2.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg_2);
  
    // Thrid create a IterativeParabolicTimeParameterization object
    //trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    success = iptp.computeTimeStamps(rt_2);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
    // Get RobotTrajectory_msg from RobotTrajectory
    rt_2.getRobotTrajectoryMsg(trajectory_msg_2);
    // Check trajectory_msg for velocities not empty
    //std::cout << trajectory_msg << std::endl;

    plan_2.trajectory_ = trajectory_msg_2;
    //group.plan(plan_4);
   
    ROS_INFO("Visualizing plan 2");
    display_trajectory_2.trajectory_start = plan_2.start_state_;
    display_trajectory_2.trajectory.push_back(plan_2.trajectory_);
    display_publisher.publish(display_trajectory_2);
    /* Sleep to give Rviz time to visualize the plan. */
    group.execute(plan_2);
    sleep(5.0);


    // PUNTO REFERENCIA 3 : ENCIMA ZAPATO (Coordenadas cartesianas, cartesian path)

    std::vector<geometry_msgs::Pose> waypoints_3;
    moveit_msgs::RobotTrajectory trajectory_msg_3;
    group. setPoseReferenceFrame("link_base");
    group.setStartStateToCurrentState();
    geometry_msgs::Pose initial_pose_3 = group.getCurrentPose().pose;
    group.setPlanningTime(3.0);
    waypoints_3.push_back(initial_pose_3);  
    geometry_msgs::Pose waypoint_pose_3;
    angle_x_roll = 1.57;
    angle_y_pitch = 0.78;
    angle_z_yaw = -0.0;
    waypoint_pose_3.orientation = tf::createQuaternionMsgFromRollPitchYaw(angle_x_roll,angle_y_pitch,angle_z_yaw);
    waypoint_pose_3.position.x = initial_pose_3.position.x + 0.20;
    waypoint_pose_3.position.y = initial_pose_3.position.y;
    waypoint_pose_3.position.z = initial_pose_3.position.z - 0.10;
    waypoints_3.push_back(waypoint_pose_3);
    
    fraction = group.computeCartesianPath(waypoints_3, 0.2, 100.0, trajectory_msg_3, false);
    ROS_INFO("Computed cartesian path %f", fraction);
    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt_3(group.getCurrentState()->getRobotModel(), "pa10_shadow");

    // Second get a RobotTrajectory from trajectory
    rt_3.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg_3);
  
    // Thrid create a IterativeParabolicTimeParameterization object
    //trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    success = iptp.computeTimeStamps(rt_3);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
    // Get RobotTrajectory_msg from RobotTrajectory
    rt_3.getRobotTrajectoryMsg(trajectory_msg_3);
    // Check trajectory_msg for velocities not empty
    //std::cout << trajectory_msg << std::endl;

    plan_3.trajectory_ = trajectory_msg_3;
    //group.plan(plan_4);
   
    ROS_INFO("Visualizing plan 3");
    display_trajectory_3.trajectory_start = plan_3.start_state_;
    display_trajectory_3.trajectory.push_back(plan_3.trajectory_);
    display_publisher.publish(display_trajectory_3);
    /* Sleep to give Rviz time to visualize the plan. */
    group.execute(plan_3);
    sleep(5.0);


    // FIN TAREA
    ROS_INFO("End...");

    ros::shutdown();
    return 0;
}
