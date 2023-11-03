#include <Eigen/Core>
#include <Eigen/Geometry>
#include <actionlib/client/simple_action_client.h>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
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

#include <control_msgs/FollowJointTrajectoryAction.h>
/*
   Author: Crasun Jans
*/
class CcAffordancePlannerNode {
public:
  const std::vector<std::string> joint_names = {
      "arm0_shoulder_yaw", "arm0_shoulder_pitch", "arm0_elbow_pitch",
      "arm0_elbow_roll",   "arm0_wrist_pitch",    "arm0_wrist_roll"};
  Eigen::VectorXd joint_states;

  ros::Subscriber joint_states_subscriber;
  CcAffordancePlannerNode(ros::NodeHandle nh)
      : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_) {
    joint_states = Eigen::VectorXd::Zero(6);

    joint_states_subscriber = nh_.subscribe(
        "joint_states", 1, &CcAffordancePlannerNode::jointStatesCallback, this);
  }

  Eigen::Isometry3d get_htm(std::string targetFrame_,
                            std::string referenceFrame_) {

    Eigen::Isometry3d htm; // Output
    // Query the listener for the desired transformation at the latest available
    // time ros::Time::now()

    try {
      transformStamped_ = tfBuffer_.lookupTransform(
          targetFrame_, referenceFrame_, ros::Time(0), ros::Duration(0.3));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
    }

    // Convert the message to Eigen::Isometry3d type
    // Extract translation and rotation from the TransformStamped message
    Eigen::Vector3d translation(transformStamped_.transform.translation.x,
                                transformStamped_.transform.translation.y,
                                transformStamped_.transform.translation.z);
    Eigen::Quaterniond rotation(transformStamped_.transform.rotation.w,
                                transformStamped_.transform.rotation.x,
                                transformStamped_.transform.rotation.y,
                                transformStamped_.transform.rotation.z);

    // Set the translation and rotation components of the HTM
    htm.translation() = translation;
    htm.linear() = rotation.toRotationMatrix();

    return htm;
  }

  // Callback function to process joint states
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    // Collect joint state data for the specified joints in the specified order
    int j = 0; // index for selected_joint_positions_eigen
    for (const std::string &joint_name : joint_names) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == joint_name) {
          /* selected_joint_positions.push_back(msg->position[i]); */
          joint_states[j] = msg->position[i];
          j++;
          break; // Move to the next joint name
        }
      }
    }
  }

  control_msgs::FollowJointTrajectoryGoal follow_joint_trajectory_msg_builder(
      const std::vector<Eigen::VectorXd> &bare_trajectory,
      const std::vector<double> &pos_offset, const double &time_step = 0.5) {
    std::cout << "Length of bare trajectory: " << bare_trajectory.size()
              << std::endl;
    control_msgs::FollowJointTrajectoryGoal fjtg_msg;

    // Set the joint names
    fjtg_msg.trajectory.joint_names = joint_names;

    fjtg_msg.trajectory.points.resize(
        bare_trajectory.size()); // resize points array before filling

    size_t j = 0;
    for (const Eigen::VectorXd &point : bare_trajectory) {
      trajectory_msgs::JointTrajectoryPoint &trajectory_point =
          fjtg_msg.trajectory.points[j];

      // Fill out point times
      trajectory_point.time_from_start = ros::Duration(j * time_step);

      // Fill out position values
      trajectory_point.positions.resize(
          joint_names.size()); // resize positions array before filling
      for (size_t i = 0; i < joint_names.size(); i++) {
        trajectory_point.positions[i] = point[i] + pos_offset[i];
      }

      j++;
    }

    return fjtg_msg;
  }

  Eigen::MatrixXd robot_builder() {

    //** Creation of screw axes
    //* Screw axis locations from base frame in home position
    Eigen::MatrixXd q(3, 10);

    // Get q vectors from tf data
    /* std::string targetFrame_ = "arm0_base_link"; */
    /* for (size_t i = 0; i < joint_names.size(); i++) { */
    /*   std::string referenceFrame_ = joint_names.at(i); */
    /*   Eigen::Isometry3d htm = get_htm(targetFrame_, referenceFrame_); */
    /*   q.col(i) = htm.translation(); */
    /*   std::cout << "Reference frame: \n" << referenceFrame_ << std::endl; */
    /*   std::cout << "q: \n" << q.col(i) << std::endl; */
    /* } */

    // Hard-coded q-vector values
    q.col(0) << 0, 0, 0;
    q.col(1) << 0, 0, 0;
    q.col(2) << 0.3385, -0.0001, -0.0003;
    q.col(3) << 0.7428, -0.0003, 0.0693;
    q.col(4) << 0.7428, -0.0003, 0.0693;
    q.col(5) << 0.7428, -0.0003, 0.0693;

    // Type and alignment of screw axes
    Eigen::MatrixXd w(3, 10);
    w.col(0) << 0, 0, 1;
    w.col(1) << 0, 1, 0;
    w.col(2) << 0, 1, 0;
    w.col(3) << 1, 0, 0;
    w.col(4) << 0, 1, 0;
    w.col(5) << 1, 0, 0;
    w.col(6) << 1, 0, 0;  // Imaginary joint
    w.col(7) << 0, 1, 0;  // Imaginary joint
    w.col(8) << 0, 0, 1;  // Imaginary joint
    w.col(9) << -1, 0, 0; // Affordance

    // Get ee q-vector from tf data
    /* Eigen::Isometry3d eeHtm = get_htm(targetFrame_, "arm0_fingers"); */
    /* Eigen::Vector3d q_ee = eeHtm.translation(); */
    /* std::cout << "Here is q_ee: \n" << q_ee << std::endl; */

    // Hard-coded ee q-vector
    const Eigen::Vector3d q_ee(0.8605, -0.0002, 0.0818);

    // Affordance and virtual joint location
    const double affOffset = 0.1;
    const Eigen::Vector3d q_aff =
        q_ee + Eigen::Vector3d(0, affOffset, 0); // q-vector for affordance
    q.col(6) = q_ee;                             // imaginary joint
    q.col(7) = q_ee;                             // imaginary joint
    q.col(8) = q_ee;                             // imaginary joint
    q.col(9) = q_aff;                            // affordance

    // Compute screw axes
    Eigen::MatrixXd slist(6, 10);

    // Construct screw axes and frames
    for (int i = 0; i < w.cols(); i++) {
      Eigen::Vector3d wcurr =
          w.col(i); // required to convert to VectorXd type for use with cross
      Eigen::Vector3d qcurr =
          q.col(i); // required to convert to VectorXd type for use with cross
      slist.col(i) << wcurr, -wcurr.cross(qcurr);
    }

    // Pure translation edit for affordance
    /* slist.col(9) = (Eigen::VectorXd(6) << 0, 0, 0, -1, 0, 0).finished(); */

    // Pure rotation edit for affordance
    slist.col(9) = (Eigen::VectorXd(6) << 0, 0, -1, 0, 0, 0).finished();
    return slist;
  }

private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tfBuffer_;              // buffer to hold tf data
  tf2_ros::TransformListener tfListener_; // listener object for tf data
  geometry_msgs::TransformStamped
      transformStamped_; // ros message to hold transform info
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "cc_affordance_planner_node");
  ros::NodeHandle node_handle;

  CcAffordancePlannerNode capN(node_handle);

  // Build robot
  Eigen::MatrixXd slist = capN.robot_builder();

  std::cout << "Here is the screwAxes matrix: \n" << slist << std::endl;

  // Put robot in the affordance start configuration to read joint states
  std::string startConfigConfirm;
  std::cout
      << "Put robot in the affordance start configuration to read joint states"
      << std::endl;
  std::cin >> startConfigConfirm;

  if (startConfigConfirm != "y")
    return 1;
  ros::spinOnce(); // read joint states

  // Compute the robot Jacobian based on the joint states. This will be the
  // screw list we send to the robot
  Eigen::MatrixXd robotSlist = slist.leftCols(capN.joint_states.size());
  CcAffordancePlanner jacTool(robotSlist, Eigen::VectorXd::Zero(slist.cols()),
                              0.3); // Meaningless constructor values just to
                                    // create the object, temporary fix
  Eigen::MatrixXd result = jacTool.JacobianSpace(robotSlist, capN.joint_states);
  slist.leftCols(capN.joint_states.size()) = result;

  // Define affordance goal and create planner object
  const double affGoal = 1.57;
  Eigen::VectorXd thetalist0 = Eigen::VectorXd::Zero(slist.cols());
  std::cout << "Before creating the object" << std::endl;
  CcAffordancePlanner ccAffordancePlanner(slist, thetalist0, affGoal);
  ccAffordancePlanner.affStep = 0.05;

  // Run the planner
  PlannerResult plannerResult = ccAffordancePlanner.affordance_stepper();

  // Print the first point in the trajectory if planner succeeds
  std::vector<Eigen::VectorXd> solution = plannerResult.jointTraj;
  if (plannerResult.success) {
    std::cout << "Planner succeeded with " << plannerResult.trajFullorPartial
              << " solution. and here is the first point in the trajectory \n"
              << solution.at(0) << std::endl;
  } else {
    std::cout << "No solution found" << std::endl;
  }

  // Send the trajectory to MoveIt for planning
  /* ros::AsyncSpinner spinner(1); */
  /* spinner.start(); */

  /* // Define planning group and move_group_interface */
  /* static const std::string PLANNING_GROUP = "arm"; */
  /* moveit::planning_interface::MoveGroupInterface move_group_interface( */
  /*     PLANNING_GROUP); */

  /* // Create visual tools and load remote control so we can introspect with */
  /* // RvizVisualToolsGui buttons */
  /* namespace rvt = rviz_visual_tools; */
  /* moveit_visual_tools::MoveItVisualTools visual_tools("arm0_base_link"); */
  /* visual_tools.deleteAllMarkers(); */
  /* visual_tools.loadRemoteControl(); */

  /* // Create the planner object */
  /* moveit::planning_interface::MoveGroupInterface::Plan my_plan; */

  /* // Prompt to start planning */
  /* visual_tools.trigger(); */
  /* visual_tools.prompt("Press 'next' to plan the trajectory"); */
  /* for (size_t j = 0; j <= solution.size(); j++) { */
  /*   moveit::core::RobotStatePtr current_state = */
  /*       move_group_interface.getCurrentState(); */
  /*   // */
  /*   // Next get the current set of joint values for the group. */
  /*   std::vector<double> joint_group_positions_cur; */
  /*   const moveit::core::JointModelGroup *joint_model_group = */
  /*       move_group_interface.getCurrentState()->getJointModelGroup( */
  /*           PLANNING_GROUP); */
  /*   current_state->copyJointGroupPositions(joint_model_group, */
  /*                                          joint_group_positions_cur); */
  /*   std::ostringstream joint_positions_stream; */
  /*   for (const auto &position : joint_group_positions_cur) { */
  /*     joint_positions_stream << position << " "; */
  /*   } */
  /*   ROS_INFO_STREAM( */
  /*       "Here are the current joint values: " <<
   * joint_positions_stream.str()); */

  /*   if (j == 0) { */
  /*     std::vector<double> start_config = { */
  /*         -0.13461518287658691, 0.03472280502319336, 1.1548473834991455, */
  /*         -0.27599477767944336, -1.3527731895446777, 0.08957767486572266}; */
  /*     move_group_interface.setJointValueTarget(start_config); */
  /*   } else { */
  /*     std::vector<double> joint_group_positions; */
  /*     const size_t armDoF = 6; */

  /*     for (size_t i = 0; i < armDoF; i++) { */
  /*       joint_group_positions.push_back(joint_group_positions_cur[i] + */
  /*                                       solution.at(j - 1)[i]); */
  /*       /1* joint_group_positions.push_back(solution.at(j)[i]); *1/ */
  /*     } */
  /*     move_group_interface.setJointValueTarget(joint_group_positions); */
  /*   } */

  /*   // Lower the max acceleration and velocity to 5%. Default is 10% */
  /*   move_group_interface.setMaxVelocityScalingFactor(0.05); */
  /*   move_group_interface.setMaxAccelerationScalingFactor(0.05); */

  /*   bool success = (move_group_interface.plan(my_plan) == */
  /*                   moveit::core::MoveItErrorCode::SUCCESS); */
  /*   ROS_INFO_NAMED("Visualizing plan 2 (joint space goal) %s", */
  /*                  success ? "" : "FAILED"); */

  /*   // Visualize the plan in RViz */
  /*   visual_tools.deleteAllMarkers(); */
  /*   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity(); */
  /*   text_pose.translation().z() = 1.0; */
  /*   visual_tools.publishText(text_pose, "MoveGroupInterface Demo",
   * rvt::WHITE, */
  /*                            rvt::XLARGE); */
  /*   visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, */
  /*                            rvt::XLARGE); */
  /*   visual_tools.publishTrajectoryLine(my_plan.trajectory_,
   * joint_model_group); */
  /*   /1* visual_tools.trigger(); *1/ */
  /*   /1* visual_tools.prompt("Press 'next' to execute the trajectory"); *1/ */
  /*   /1* if (success) { *1/ */
  /*   /1*   move_group_interface.execute(my_plan); *1/ */
  /*   /1* } *1/ */
  /*   /1* joint_group_positions.clear(); *1/ */
  /* } */

  // Attempt 2
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using a planning pipeline is pretty easy. Before we can
  // load the planner, we need two objects, a RobotModel and a PlanningScene.
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
  // will create a planning scene, monitors planning scene diffs, and apply the
  // diffs to it's internal planning scene. We then call startSceneMonitor,
  // startWorldGeometryMonitor and startStateMonitor to fully initialize the
  // planning scene monitor
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

  /* We can get the most up to date robot state from the PlanningSceneMonitor by
     locking the internal planning scene for reading. This lock ensures that the
     underlying scene isn't updated while we are reading it's state.
     RobotState's are useful for computing the forward and inverse kinematics of
     the robot among many other uses */
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  /* Create a JointModelGroup to keep track of the current robot pose and
     planning group. The Joint Model
     group is useful for dealing with one set of joints at a time such as a left
     arm or a end effector */
  const moveit::core::JointModelGroup *joint_model_group =
      robot_state->getJointModelGroup("arm");

  // We can now setup the PlanningPipeline object, which will use the ROS
  // parameter server to determine the set of request adapters and the planning
  // plugin to use
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(
          robot_model, node_handle, "planning_plugin", "request_adapters"));

  // Visualization
  // ^^^^^^^^^^^^^
  // The package MoveItVisualTools provides many capabilities for visualizing
  // objects, robots, and trajectories in RViz as well as debugging tools such
  // as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("arm0_base_link");
  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a
     high level script via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  /* RViz provides many types of markers, in this demo we will use text,
   * cylinders, and spheres*/
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo",
                           rvt::WHITE, rvt::XLARGE);

  /* Batch publishing is used to reduce the number of messages being sent to
   * RViz for large visualizations */
  /* visual_tools.trigger(); */

  /* We can also use visual_tools to wait for user input */
  /* visual_tools.prompt( */
  /* "Press 'next' in the RvizVisualToolsGui window to start the demo"); */

  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the Panda
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  /* geometry_msgs::PoseStamped pose; */
  /* pose.header.frame_id = "panda_link0"; */
  /* pose.pose.position.x = 0.3; */
  /* pose.pose.position.y = 0.0; */
  /* pose.pose.position.z = 0.75; */
  /* pose.pose.orientation.w = 1.0; */

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  /* std::vector<double> tolerance_pose(3, 0.01); */
  /* std::vector<double> tolerance_angle(3, 0.01); */

  // We will create the request as a constraint using a helper function
  // available from the `kinematic_constraints`_ package.
  //
  // .. _kinematic_constraints:
  //     http://docs.ros.org/noetic/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  req.group_name = "arm";
  /* moveit_msgs::Constraints pose_goal = */
  /*     kinematic_constraints::constructGoalConstraints( */
  /*         "panda_link8", pose, tolerance_pose, tolerance_angle); */
  /* req.goal_constraints.push_back(pose_goal); */

  // Before planning, we will need a Read Only lock on the planning scene so
  // that it does not modify the world representation while planning
  /* { */
  /*   planning_scene_monitor::LockedPlanningSceneRO lscene(psm); */
  /*   /1* Now, call the pipeline and check whether planning was successful. *1/
   */
  /*   planning_pipeline->generatePlan(lscene, req, res); */
  /* } */
  /* Now, call the pipeline and check whether planning was successful. */
  /* Check that the planning was successful */
  /* if (res.error_code_.val != res.error_code_.SUCCESS) { */
  /* ROS_ERROR("Could not compute plan successfully"); */
  /* return 0; */
  /* } */

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>(
          "/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* /1* Visualize the trajectory *1/ */
  /* ROS_INFO("Visualizing the trajectory"); */
  moveit_msgs::MotionPlanResponse response;
  /* res.getMessage(response); */

  /* display_trajectory.trajectory_start = response.trajectory_start; */
  /* display_trajectory.trajectory.push_back(response.trajectory); */
  /* display_publisher.publish(display_trajectory); */
  /* visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), */
  /* joint_model_group); */
  visual_tools.trigger();

  /* Wait for user input */
  visual_tools.prompt("Press next to plan the trajectory");

  // Joint Space Goals
  // ^^^^^^^^^^^^^^^^^
  /* First, set the state in the planning scene to the final state of the last
   * plan */
  /* robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm) */
  /*                   ->getCurrentStateUpdated(response.trajectory_start); */
  /* robot_state->setJointGroupPositions( */
  /*     joint_model_group, */
  /*     response.trajectory.joint_trajectory.points.back().positions); */
  /* moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state); */

  // Set the state in the planning scene to the current state of the robot

  //
  // Next get the current set of joint values for the group.
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(
      PLANNING_GROUP);
  moveit::core::RobotStatePtr current_state =
      move_group_interface.getCurrentState();
  std::vector<double> joint_group_positions_cur;
  /* const moveit::core::JointModelGroup *joint_model_group = */
  /*     move_group_interface.getCurrentState()->getJointModelGroup( */
  /*         PLANNING_GROUP); */
  current_state->copyJointGroupPositions(joint_model_group,
                                         joint_group_positions_cur);
  std::ostringstream joint_positions_stream;
  for (const auto &position : joint_group_positions_cur) {
    joint_positions_stream << position << " ";
  }
  ROS_INFO_STREAM(
      "Here are the current joint values: " << joint_positions_stream.str());
  std::cout << "Joint group positions cur: " << std::endl;

  // Lock the planning scene and set the current state to the current robot
  // state just to be sure
  /* *robot_state = */
  /*     planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState();
   */

  //**** Planning loop
  // Create the planner object

  for (int j = 0; j < solution.size(); j++) {
    /* First, set the state in the planning scene to the final state of the last
     * plan */

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

    /*   if (j == 0) { */
    /*     std::vector<double> start_config = { */
    /*         -0.13461518287658691, 0.03472280502319336, 1.1548473834991455, */
    /*         -0.27599477767944336, -1.3527731895446777, 0.08957767486572266};
     */
    /*     move_group_interface.setJointValueTarget(start_config); */
    /*   } else { */
    std::vector<double> joint_group_positions;
    const size_t armDoF = 6;

    for (size_t i = 0; i < armDoF; i++) {
      joint_group_positions.push_back(joint_group_positions_cur[i] +
                                      solution.at(j)[i]);
      /* joint_group_positions.push_back(solution.at(j)[i]); */
    }
    /*     move_group_interface.setJointValueTarget(joint_group_positions); */
    /*   } */

    // Now, setup a joint space goal
    moveit::core::RobotState goal_state(*robot_state);
    /* std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0,
     * 0.0};
     */
    std::cout << "Joint group positions: " << std::endl;
    for (const double &value : joint_group_positions) {
      std::cout << value << " ";
    }
    std::cout << std::endl;

    goal_state.setJointGroupPositions(joint_model_group, joint_group_positions);
    std::cout << "Debug flag" << std::endl;
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
    } else {
      /* moveit::planning_interface::MoveGroupInterface::Plan my_plan; */
      /* moveit_msgs::MotionPlanResponse response1; */
      /* res.getMessage(response1); */
      /* my_plan.planning_time_ = response1.planning_time; */
      /* my_plan.trajectory = response1.trajectory; */
      /* my_plan.start_state_ = response1.trajectory_start; */

      /* move_group_interface.execute(my_plan); */
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
  visual_tools.trigger();

  /* Wait for user input */
  visual_tools.prompt("Press next to execute the trajectory");

  /* // Using a Planning Request Adapter */
  /* // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
  /* // A planning request adapter allows us to specify a series of operations
   * that */
  /* // should happen either before planning takes place or after the planning
   */
  /* // has been done on the resultant path */

  /* /1* First, set the state in the planning scene to the final state of the
   * last */
  /*  * plan *1/ */
  /* robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm) */
  /*                   ->getCurrentStateUpdated(response.trajectory_start); */
  /* robot_state->setJointGroupPositions( */
  /*     joint_model_group, */
  /*     response.trajectory.joint_trajectory.points.back().positions); */
  /* moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state); */

  /* // Now, set one of the joints slightly outside its upper limit */
  /* const moveit::core::JointModel *joint_model = */
  /*     joint_model_group->getJointModel("panda_joint3"); */
  /* const moveit::core::JointModel::Bounds &joint_bounds = */
  /*     joint_model->getVariableBounds(); */
  /* std::vector<double> tmp_values(1, 0.0); */
  /* tmp_values[0] = joint_bounds[0].min_position_ - 0.01; */
  /* robot_state->setJointPositions(joint_model, tmp_values); */

  /* req.goal_constraints.clear(); */
  /* req.goal_constraints.push_back(pose_goal); */

  /* // Before planning, we will need a Read Only lock on the planning scene so
   */
  /* // that it does not modify the world representation while planning */
  /* { */
  /*   planning_scene_monitor::LockedPlanningSceneRO lscene(psm); */
  /*   /1* Now, call the pipeline and check whether planning was successful. *1/
   */
  /*   planning_pipeline->generatePlan(lscene, req, res); */
  /* } */
  /* if (res.error_code_.val != res.error_code_.SUCCESS) { */
  /*   ROS_ERROR("Could not compute plan successfully"); */
  /*   return 0; */
  /* } */
  /* /1* Visualize the trajectory *1/ */
  /* ROS_INFO("Visualizing the trajectory"); */
  /* res.getMessage(response); */
  /* display_trajectory.trajectory_start = response.trajectory_start; */
  /* display_trajectory.trajectory.push_back(response.trajectory); */
  /* Now you should see three planned trajectories in series*/
  /* display_publisher.publish(display_trajectory); */
  /* visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), */
  /*                                    joint_model_group); */
  /* visual_tools.trigger(); */

  /* /1* Wait for user input *1/ */
  /* visual_tools.prompt( */
  /*     "Press 'next' in the RvizVisualToolsGui window to finish the demo"); */

  const control_msgs::FollowJointTrajectoryGoal fjtg_msg =
      capN.follow_joint_trajectory_msg_builder(solution,
                                               joint_group_positions_cur);
  ROS_INFO_STREAM("Here is the coveted ROS msg: " << fjtg_msg);

  // Execute directly
  // Create the connection to the action server
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      client("/spot_arm/arm_controller/follow_joint_trajectory", true);
  client.waitForServer(); // Waits until the action server is up and running

  client.sendGoal(fjtg_msg);
  client.waitForResult();

  return 0;
}
