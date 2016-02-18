#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Pa10_Shadow_trajectory_Generator");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Cargar modelo
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  ROS_INFO("Robot model...");


  // Crear planning scene
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  ROS_INFO("Planning scene..");

  // Crear loader to planner
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;
  ROS_INFO("Planner loader...");


  // Load planner from param server
  if (!node_handle.getParam("/planning_plugin", planner_plugin_name))
  ROS_FATAL_STREAM("Could not find planner plugin name");
  planner_plugin_name = "ompl_interface/OMPLPlanner";
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0 ; i < classes.size() ; ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
  }

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(15.0);
  sleep_time.sleep();

  // Create pose goal
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  req.group_name = "pa10_shadow_palm";
  pose.header.frame_id = "palm";
  pose.pose.position.x = 0.75;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Set tolerance 0.01 m and 0.01 rad
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // Create request 
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("palm", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // Construct planning context
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;

  planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if(res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_WARN("Could not compute plan successfully");
    //return 0;
  }
  else{
    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);
    sleep_time.sleep();
  }

  double posx_, posy_, posz_, posw_;

  //bucle ejecucion continua
  do{ 
      // >> PARAMETROS DE LOS EXPERIMENTOS
      // >> Obtener posición goal
      
      if (node_handle.getParam("/grasp_reconfiguration/pa10_shadow_goal_x", posx_))
      {
        ROS_INFO("Posicion X: %f", posx_); 
      }
      if (node_handle.getParam("/grasp_reconfiguration/pa10_shadow_goal_z", posy_))
      {
        ROS_INFO("Posicion Y: %f", posy_); 
      }
      if (node_handle.getParam("/grasp_reconfiguration/pa10_shadow_goal_y", posz_))
      {
        ROS_INFO("Posicion Z: %f", posz_); 
      }
      if (node_handle.getParam("/grasp_reconfiguration/pa10_shadow_goal_w", posw_))
      {
        ROS_INFO("Posicion W: %f", posw_); 
      }


      pose.pose.position.x = posx_;
      pose.pose.position.y = posy_;
      pose.pose.position.z = posz_;
      pose.pose.orientation.w = posw_;


      // Create request 
      moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("palm", pose, tolerance_pose, tolerance_angle);
      req.goal_constraints.push_back(pose_goal);

      // Construct planning context
      planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
      context->solve(res);
      if(res.error_code_.val != res.error_code_.SUCCESS)
      {
        ROS_WARN("Could not compute plan successfully");
        //return 0;
      }
      else{
         /* Visualize the trajectory */
        ROS_INFO("Visualizing the trajectory");
        res.getMessage(response);

        display_trajectory.trajectory_start = response.trajectory_start;
        display_trajectory.trajectory.push_back(response.trajectory);
        display_publisher.publish(display_trajectory);
        sleep_time.sleep();
      }
  }while(true);


  ros::spin();
  return 0;
}
