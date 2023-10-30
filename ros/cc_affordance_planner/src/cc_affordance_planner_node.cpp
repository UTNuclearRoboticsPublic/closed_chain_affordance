#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

/*
   Author: Crasun Jans
*/
class CcAffordancePlannerNode {
public:
  std::vector<std::string> joint_names_to_collect = {
      "arm0_shoulder_yaw", "arm0_shoulder_pitch", "arm0_elbow_pitch",
      "arm0_elbow_roll",   "arm0_wrist_pitch",    "arm0_wrist_roll"};
  std::vector<double> selected_joint_positions;

  ros::Subscriber joint_states_subscriber;
  CcAffordancePlannerNode(ros::NodeHandle nh)
      : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_) {

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

  Eigen::VectorXd htmToScrew(Eigen::Isometry3d htm) {

    /* std::cout << "Here is the rotation of the HTM: \n" */
    /*           << htm.linear() << std::endl; */
    /* std::cout << "Here is the translation of the HTM: \n" */
    /*           << htm.translation() << std::endl; */
    Eigen::VectorXd s(6);
    Eigen::Vector3d w = htm.linear().block<3, 1>(0, 2);
    s.head(3) = w;
    s.tail(3) = -w.cross(htm.translation());
    return s;
  }

  // Callback function to process joint states
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    // Collect joint state data for the specified joints in the specified order
    for (const std::string &joint_name : joint_names_to_collect) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == joint_name) {
          selected_joint_positions.push_back(msg->position[i]);
          break; // Move to the next joint name
        }
      }
    }

    // Check if all specified joints have been collected
    if (selected_joint_positions.size() == joint_names_to_collect.size()) {
      // Process or use the collected joint state data here
      // For example, you can print the selected joint positions
      for (size_t i = 0; i < selected_joint_positions.size(); ++i) {
        ROS_INFO("Joint Name: %s, Position: %f",
                 joint_names_to_collect[i].c_str(),
                 selected_joint_positions[i]);
      }
    }
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

  CcAffordancePlannerNode ccAffordancePlannerNode(node_handle);
  //** Creation of screw axes
  //* Screw axis locations from base frame in home position
  // First get HTMs from TF data
  std::vector<std::string> joint_names;
  joint_names.push_back("arm0_shoulder_yaw");
  joint_names.push_back("arm0_shoulder_pitch");
  joint_names.push_back("arm0_elbow_pitch");
  joint_names.push_back("arm0_elbow_roll");
  joint_names.push_back("arm0_wrist_pitch");
  joint_names.push_back("arm0_wrist_roll");

  std::string targetFrame_ = "arm0_base_link";

  Eigen::MatrixXd q(3, 10);

  for (size_t i = 0; i < joint_names.size(); i++) {
    std::string referenceFrame_ = joint_names.at(i);
    Eigen::Isometry3d htm =
        ccAffordancePlannerNode.get_htm(targetFrame_, referenceFrame_);
    q.col(i) = htm.translation();
    std::cout << "Reference frame: \n" << referenceFrame_ << std::endl;
    std::cout << "q: \n" << q.col(i) << std::endl;
  }

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

  Eigen::Isometry3d eeHtm =
      ccAffordancePlannerNode.get_htm(targetFrame_, "arm0_fingers");
  Eigen::Vector3d q_ee = eeHtm.translation();
  const double affOffset = 0.1;
  Eigen::Vector3d q_aff = q_ee + Eigen::Vector3d(0, affOffset, 0);
  q.col(6) = q_ee;  // imaginary joint
  q.col(7) = q_ee;  // imaginary joint
  q.col(8) = q_ee;  // imaginary joint
  q.col(9) = q_aff; // affordance
  std::cout << "Debug flag" << std::endl;
  Eigen::MatrixXd slist(6, 10);

  // Construct screw axes and frames
  for (int i = 0; i < w.cols(); i++) {
    Eigen::Vector3d wcurr =
        w.col(i); // required to convert to VectorXd type for use with cross
    Eigen::Vector3d qcurr =
        q.col(i); // required to convert to VectorXd type for use with cross
    slist.col(i) << wcurr, -wcurr.cross(qcurr);
  }

  // Pure translation edit:
  slist.col(9) = (Eigen::VectorXd(6) << 0, 0, 0, -1, 0, 0).finished();
  std::cout << "w: \n" << w << std::endl;
  std::cout << "q: \n" << q << std::endl;

  std::cout << "Here is the screwAxes matrix: \n" << slist << std::endl;

  std::string inp1;
  std::string inp2;
  std::cout << "Have the arm in the desired configuration. Done?" << std::endl;
  ;
  std::cin >> inp1;

  if (inp1 != "y")
    return 1;
  ros::spinOnce(); // get joint states
                   // Convert joint positions to Eigen::type
  Eigen::VectorXd selected_joint_positions_eigen(
      ccAffordancePlannerNode.selected_joint_positions.size());
  for (size_t i = 0;
       i < ccAffordancePlannerNode.selected_joint_positions.size(); ++i) {
    selected_joint_positions_eigen(i) =
        ccAffordancePlannerNode.selected_joint_positions[i];
  }

  Eigen::MatrixXd selected_slist =
      slist.leftCols(ccAffordancePlannerNode.selected_joint_positions.size());
  /* CcAffordancePlanner ccAffordancePlannerTemp( */
  /*     slist, Eigen::Matrix4d::Identity(),
   * Eigen::VectorXd::Zero(slist.cols()), */
  /*     Eigen::Matrix4d::Identity()); */
  CcAffordancePlanner ccAffordancePlannerTemp(
      slist, Eigen::VectorXd::Zero(slist.cols()), 0.3);
  Eigen::MatrixXd result = ccAffordancePlannerTemp.JacobianSpace(
      selected_slist, selected_joint_positions_eigen);
  slist.leftCols(ccAffordancePlannerNode.selected_joint_positions.size()) =
      result;

  std::cout << "Everything went well? Ready to proceed?" << std::endl;
  std::cin >> inp2;

  if (inp2 != "y")
    return 1;

  Eigen::Matrix4d Tsd = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d mErr = Eigen::Matrix4d::Identity();
  Eigen::VectorXd thetalist0 = Eigen::VectorXd::Zero(slist.cols());
  const double affGoal = 0.3;
  std::cout << "Before creating the object" << std::endl;
  /* CcAffordancePlanner ccAffordancePlanner(slist, mErr, thetalist0, Tsd); */
  CcAffordancePlanner ccAffordancePlanner(slist, thetalist0, affGoal);

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
  // Send it to MoveIt for planning
  /* ros::AsyncSpinner spinner(1); */
  /* spinner.start(); */
  /* moveit::planning_interface::MoveGroupInterface move_group("arm"); */

  /* // Set joint trajectory */
  /* std::vector<double> joint_values; */
  /* const size_t armDoF = 6; */

  /* for (size_t i = 0; i < armDoF; i++) { */
  /*   joint_values.push_back(solution.at(0)[i]); */
  /*   std::cout << "joint_value[" << i << "] = " << joint_values[i] <<
   * std::endl; */
  /* } */

  /* move_group.setJointValueTarget(joint_values); */

  /* // Plan */
  /* moveit::planning_interface::MoveGroupInterface::Plan my_plan; */
  /* moveit::core::MoveItErrorCode planning_result = move_group.plan(my_plan);
   */

  /* if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) { */
  /*   // The planning was successful, and you can execute it. */
  /*   /1* move_group.execute(my_plan); *1/ */
  /*   ROS_INFO("planning succeeded"); */
  /* } else { */
  /*   // Planning failed; you may want to handle the failure accordingly. */
  /*   ROS_ERROR("Planning failed "); */
  /* } */

  /* /1* if (success) { *1/ */
  /* /1*   move_group.execute(my_plan); *1/ */
  /* /1* } *1/ */

  /* // Visualize the plan in Rviz */
  /* moveit_visual_tools::MoveItVisualTools visual_tools( */
  /*     "move_group/monitored_planning_scene"); */
  /* visual_tools.loadRemoteControl(); */
  /* visual_tools.publishPath(my_plan.trajectory_, */
  /*                          rviz_visual_tools::colors::LIME_GREEN, */
  /*                          rviz_visual_tools::scales::SMALL, "arm"); */
  /* visual_tools.publishPath(my_plan.trajectory_,
   * rviz_visual_tools::colors::BLUE, */
  /*                          rviz_visual_tools::scales::SMALL); */
  /* visual_tools.trigger(); */

  /* ros::shutdown(); */

  // new attempt
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them
  // in an object called the `JointModelGroup`. Throughout MoveIt the terms
  // "planning group" and "joint model group" are used interchangeably.
  static const std::string PLANNING_GROUP = "arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control
  // and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(
      PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for
  // improved performance.
  const moveit::core::JointModelGroup *joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing
  // objects, robots, and trajectories in RViz as well as debugging tools such
  // as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("arm0_base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a
  // high level script via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text,
  // cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE,
                           rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to
  // RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s",
                 move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s",
                 move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to start the demo");

  // .. _move_group_interface-planning-to-pose-goal:
  //
  /* // Planning to a Pose goal */
  /* // ^^^^^^^^^^^^^^^^^^^^^^^ */
  /* // We can plan a motion for this group to a desired pose for the */
  /* // end-effector. */
  /* geometry_msgs::Pose target_pose1; */
  /* target_pose1.orientation.w = 1.0; */
  /* target_pose1.position.x = 0.28; */
  /* target_pose1.position.y = -0.2; */
  /* target_pose1.position.z = 0.5; */
  /* move_group_interface.setPoseTarget(target_pose1); */

  /* // Now, we call the planner to compute the plan and visualize it. */
  /* // Note that we are just planning, not asking move_group_interface */
  /* // to actually move the robot. */
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  /* bool success = (move_group_interface.plan(my_plan) == */
  /*                 moveit::core::MoveItErrorCode::SUCCESS); */

  /* ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", */
  /*                success ? "" : "FAILED"); */

  /* // Visualizing plans */
  /* // ^^^^^^^^^^^^^^^^^ */
  /* // We can also visualize the plan as a line with markers in RViz. */
  /* ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line"); */
  /* visual_tools.publishAxisLabeled(target_pose1, "pose1"); */
  /* visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
   */
  /* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
   */
  /* visual_tools.trigger(); */
  /* visual_tools.prompt( */
  /*     "Press 'next' in the RvizVisualToolsGui window to continue the demo");
   */

  // Finally, to execute the trajectory stored in my_plan, you could use the
  // following method call: Note that this can lead to problems if the robot
  // moved in the meanwhile. move_group_interface.execute(my_plan);

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // If you do not want to inspect the planned trajectory,
  // the following is a more robust combination of the two-step plan+execute
  // pattern shown above and should be preferred. Note that the pose goal we had
  // set earlier is still active, so the robot will try to move to that goal.

  // move_group_interface.move();

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's
  // state. RobotState is the object that contains all the current
  // position/velocity/acceleration data.
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to plan and execute the trajectory");
  for (size_t j = 0; j < solution.size(); j++) {
    moveit::core::RobotStatePtr current_state =
        move_group_interface.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions_cur;
    current_state->copyJointGroupPositions(joint_model_group,
                                           joint_group_positions_cur);
    std::vector<double> joint_group_positions;
    const size_t armDoF = 6;

    /* visual_tools.trigger(); */
    /* visual_tools.prompt("Press 'next' to plan the trajectory"); */
    for (size_t i = 0; i < armDoF; i++) {
      joint_group_positions.push_back(joint_group_positions_cur[i] +
                                      solution.at(j)[i]);
      /* joint_group_positions.push_back(solution.at(j)[i]); */
      /* std::cout << "joint_value[" << i << "] = " << joint_group_positions[i]
       */
      /*           << std::endl; */
    }

    // Now, let's modify one of the joints, plan to the new joint space goal and
    // visualize the plan.
    /* joint_group_positions[0] = -tau / 6; // -1/6 turn in radians */
    move_group_interface.setJointValueTarget(joint_group_positions);

    // We lower the allowed maximum velocity and acceleration to 5% of their
    // maximum. The default values are 10% (0.1). Set your preferred defaults in
    // the joint_limits.yaml file of your robot's moveit_config or set explicit
    // factors in your code if you need your robot to move faster.
    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);

    bool success = (move_group_interface.plan(my_plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s",
                   success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE,
                             rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    /* visual_tools.trigger(); */
    /* visual_tools.prompt("Press 'next' to execute the trajectory"); */
    if (success) {
      move_group_interface.execute(my_plan);
    }
    /* joint_group_positions.clear(); */
  }

  return 0;
}
