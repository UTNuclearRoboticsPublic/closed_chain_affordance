#include <cc_affordance_planner/MoveItPlanAndViz.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

/*
   Author: Crasun Jans
*/
class MoveItPlanAndVizServer {
public:
  MoveItPlanAndVizServer() : spinner_(1), tfBuffer_(), tfListener_(tfBuffer_) {
    // Initialize service
    moveit_plan_and_viz_server_ = nh_.advertiseService(
        "/moveit_plan_and_viz_server",
        &MoveItPlanAndVizServer::moveit_plan_and_viz_server_callback_, this);
    // Start async spinner
    spinner_.start();
  }

private:
  /* Variables */
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner_;             // async spinner needed for MoveIt
  tf2_ros::Buffer tfBuffer_;              // buffer to hold tf data
  tf2_ros::TransformListener tfListener_; // listener object for tf data
  ros::ServiceServer
      moveit_plan_and_viz_server_; // server to plan joint trajectory with
                                   // moveit and visualize the tracing of a
                                   // specified frame as well

  /* Methods */
  // Given a target frame and reference frame, looks up and returns the tf
  geometry_msgs::TransformStamped get_htm_(std::string target_frame,
                                           std::string reference_frame) {
    geometry_msgs::TransformStamped htm; // ros message to hold transform info

    // Query the listener for the desired transformation at the latest available
    // time ros::Time(0)

    try {
      htm = tfBuffer_.lookupTransform(target_frame, reference_frame,
                                      ros::Time(0), ros::Duration(0.3));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
    }

    return htm;
  }

  // Service callback
  bool moveit_plan_and_viz_server_callback_(
      cc_affordance_planner::MoveItPlanAndViz::Request &serv_req,
      cc_affordance_planner::MoveItPlanAndViz::Response &serv_res) {

    /* MoveIt Planning using planning_pipeline. The following excerpt of code
     * has been directly copied from MoveIt tutorial */
    /* -------------------------------------------------------------- */

    // MoveIt Planning
    // Before we can
    // load the planning_pipeline planner, we need two objects, a RobotModel and
    // a PlanningScene.
    //
    // We will start by instantiating a `RobotModelLoader`_ object, which will
    // look up the robot description on the ROS parameter server and construct a
    // :moveit_core:`RobotModel` for us to use.
    //
    // .. _RobotModelLoader:
    //     http://docs.ros.org/noetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader("robot_description"));

    // Using the RobotModelLoader, we can construct a planing scene monitor that
    // will create a planning scene, monitors planning scene diffs, and apply
    // the diffs to it's internal planning scene. We then call
    // startSceneMonitor, startWorldGeometryMonitor and startStateMonitor to
    // fully initialize the planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr psm(
        new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    /* listen for planning scene messages on topic /XXX and apply them to
                         the internal planning scene accordingly */
    psm->startSceneMonitor();
    /* listens to changes of world geometry, collision objects, and (optionally)
     * octomaps */
    psm->startWorldGeometryMonitor();
    /* listen to joint state updates as well as changes in attached collision
       objects and update the internal planning scene accordingly*/
    psm->startStateMonitor();

    /* We can also use the RobotModelLoader to get a robot model which contains
     * the robot's kinematic information */
    moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

    /* We can get the most up to date robot state from the PlanningSceneMonitor
       by locking the internal planning scene for reading. This lock ensures
       that the underlying scene isn't updated while we are reading it's state.
       RobotState's are useful for computing the forward and inverse kinematics
       of the robot among many other uses */
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
        planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

    /* Create a JointModelGroup to keep track of the current robot pose and
       planning group. The Joint Model
       group is useful for dealing with one set of joints at a time such as a
       left arm or a end effector */
    const moveit::core::JointModelGroup *joint_model_group =
        robot_state->getJointModelGroup("arm");

    // We can now setup the PlanningPipeline object, which will use the ROS
    // parameter server to determine the set of request adapters and the
    // planning plugin to use
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(
            robot_model, nh_, "planning_plugin", "request_adapters"));

    // Visualization
    // ^^^^^^^^^^^^^
    // The package MoveItVisualTools provides many capabilities for visualizing
    // objects, robots, and trajectories in RViz as well as debugging tools such
    // as step-by-step introspection of a script.
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("arm0_base_link");
    visual_tools.deleteAllMarkers();

    /* Remote control is an introspection tool that allows users to step through
       a high level script via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    /* RViz provides many types of markers, in this demo we will use text,
     * cylinders, and spheres*/
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo",
                             rvt::WHITE, rvt::XLARGE);

    // We will now create a motion plan request for the arm
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = "arm";

    // Visualization variables
    ros::Publisher display_publisher =
        nh_.advertise<moveit_msgs::DisplayTrajectory>(
            "/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    moveit_msgs::MotionPlanResponse response;

    visual_tools.trigger();
    /* Wait for user input */
    visual_tools.prompt("Press next to plan the trajectory");

    // We ensure we plan from the current state of the robot
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(
        PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state =
        move_group_interface.getCurrentState();
    std::vector<double> joint_group_positions_cur;
    current_state->copyJointGroupPositions(joint_model_group,
                                           joint_group_positions_cur);

    // Lock the planning scene and set the current state to the current robot
    // state just to be sure
    *robot_state =
        planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState();

    //**** Planning loop
    const size_t lof_joint_traj = serv_req.joint_traj.points.size();

    for (size_t j = 0; j < lof_joint_traj; j++) {
      /* First, set the state in the planning scene to the final state of the
       * last plan */

      if (j == 0) {
        robot_state->setJointGroupPositions(joint_model_group,
                                            joint_group_positions_cur);
        moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
      } else {

        robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)
                          ->getCurrentStateUpdated(response.trajectory_start);
        robot_state->setJointGroupPositions(
            joint_model_group,
            response.trajectory.joint_trajectory.points.back().positions);
        moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
        robot_state->setJointGroupPositions(
            joint_model_group,
            response.trajectory.joint_trajectory.points.back().positions);
      }

      // Now, setup a joint space goal based on the info from service request
      std::vector<double> joint_group_positions;
      trajectory_msgs::JointTrajectoryPoint &point =
          serv_req.joint_traj.points[j];

      for (size_t i = 0; i < point.positions.size(); i++) {
        joint_group_positions.push_back(point.positions[i]);
      }

      moveit::core::RobotState goal_state(*robot_state);

      goal_state.setJointGroupPositions(joint_model_group,
                                        joint_group_positions);
      moveit_msgs::Constraints joint_goal =
          kinematic_constraints::constructGoalConstraints(goal_state,
                                                          joint_model_group);

      req.goal_constraints.clear();
      req.goal_constraints.push_back(joint_goal);

      // Before planning, we will need a Read Only lock on the planning scene so
      // that it does not modify the world representation while planning
      {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        /* Now, call the pipeline and check whether planning was successful. */
        planning_pipeline->generatePlan(lscene, req, res);
      }
      /* Check that the planning was successful */
      if (res.error_code_.val != res.error_code_.SUCCESS) {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
      }
      /* Visualize the trajectory */
      ROS_INFO("Visualizing the trajectory");
      res.getMessage(response);
      display_trajectory.trajectory_start = response.trajectory_start;
      display_trajectory.trajectory.push_back(response.trajectory);
      // Now you should see two planned trajectories in series
      display_publisher.publish(display_trajectory);
      visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(),
                                         joint_model_group);
    }
    /* -------------------------------------------------------------- */

    return true;
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "moveit_plan_and_viz_server_node");

  // Create server object
  MoveItPlanAndVizServer moveItPlanAndVizServer;

  // Keep the node alive
  ros::spin();

  return 0;
}
