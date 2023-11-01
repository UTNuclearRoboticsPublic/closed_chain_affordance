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
  ccAffordancePlanner.affStep = 1.57;

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
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Define planning group and move_group_interface
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(
      PLANNING_GROUP);

  // Create visual tools and load remote control so we can introspect with
  // RvizVisualToolsGui buttons
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("arm0_base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // Create the planner object
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Prompt to start planning
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to plan the trajectory");
  for (size_t j = 0; j <= solution.size(); j++) {
    moveit::core::RobotStatePtr current_state =
        move_group_interface.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions_cur;
    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(
            PLANNING_GROUP);
    current_state->copyJointGroupPositions(joint_model_group,
                                           joint_group_positions_cur);
    std::ostringstream joint_positions_stream;
    for (const auto &position : joint_group_positions_cur) {
      joint_positions_stream << position << " ";
    }
    ROS_INFO_STREAM(
        "Here are the current joint values: " << joint_positions_stream.str());

    if (j == 0) {
      std::vector<double> start_config = {
          -0.13461518287658691, 0.03472280502319336, 1.1548473834991455,
          -0.27599477767944336, -1.3527731895446777, 0.08957767486572266};
      move_group_interface.setJointValueTarget(start_config);
    } else {
      std::vector<double> joint_group_positions;
      const size_t armDoF = 6;

      for (size_t i = 0; i < armDoF; i++) {
        joint_group_positions.push_back(joint_group_positions_cur[i] +
                                        solution.at(j - 1)[i]);
        /* joint_group_positions.push_back(solution.at(j)[i]); */
      }
      move_group_interface.setJointValueTarget(joint_group_positions);
    }

    // Lower the max acceleration and velocity to 5%. Default is 10%
    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);

    bool success = (move_group_interface.plan(my_plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Visualizing plan 2 (joint space goal) %s",
                   success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE,
                             rvt::XLARGE);
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE,
                             rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to execute the trajectory");
    if (success) {
      move_group_interface.execute(my_plan);
    }
    /* joint_group_positions.clear(); */
  }

  return 0;
}
