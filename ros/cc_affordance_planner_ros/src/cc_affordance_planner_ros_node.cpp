#include <Eigen/Core>
#include <Eigen/Geometry>
#include <actionlib/client/simple_action_client.h>
#include <affordance_util/affordance_util.hpp>
#include <affordance_util_ros/affordance_util_ros.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <filesystem>
#include <moveit_plan_and_viz/MoveItPlanAndViz.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
/*
   Author: Crasun Jans
*/

class CustomException : public std::exception {
public:
  CustomException(const char *message) : msg(message) {}

  // Override the what() function to provide a description of the exception
  virtual const char *what() const noexcept override { return msg.c_str(); }

private:
  std::string msg;
};

std::string get_filepath_inside_pkg(const std::string &package_name,
                                    const std::string &rel_dir,
                                    const std::string &filename) {

  std::string full_filepath; // output of the function

  // Get the path to the package
  const std::string package_path = ros::package::getPath(package_name);

  // Build full filepath and check for errors
  if (!package_path.empty()) {
    full_filepath = package_path + rel_dir + filename;
  } else {
    throw CustomException(
        ("Failed to find path for package '" + package_name).c_str());
  }

  // Make sure the file exists
  if (!std::filesystem::exists(full_filepath)) {
    throw CustomException(
        ("File does not exist at this path: '" + full_filepath).c_str());
  }

  return full_filepath;
}
Eigen::Matrix<double, 6, 1> get_screw(const Eigen::Vector3d &w,
                                      const Eigen::Vector3d &q) {

  Eigen::VectorXd screw(6); // Output of the function

  screw.head(3) = w;
  screw.tail(3) = -w.cross(q); // q cross w

  return screw;
}
Eigen::MatrixXd compose_cc_model_slist(
    const Eigen::MatrixXd &robot_slist, const Eigen::VectorXd &thetalist,
    const Eigen::Matrix4d &M, const Eigen::Matrix<double, 6, 1> &aff_screw) {

  // Compute robot Jacobian
  Eigen::MatrixXd robot_jacobian =
      AffordanceUtil::JacobianSpace(robot_slist, thetalist);

  // Append virtual screw axes as well as the affordance screw. TODO: Use home
  // config screws and angles so as to track true gripper orientation
  Eigen::MatrixXd app_slist(6, 4);

  // Virtual screws
  const Eigen::Matrix4d ee_htm =
      AffordanceUtil::FKinSpace(M, robot_slist, thetalist);
  const Eigen::Vector3d q_vir = ee_htm.block<3, 1>(0, 3);

  Eigen::Matrix<double, 3, 3> w_vir;
  w_vir.col(0) << 1, 0, 0; // x
  w_vir.col(1) << 0, 1, 0; // y
  w_vir.col(2) << 0, 0, 1; // z

  for (size_t i = 0; i < w_vir.cols(); ++i) {
    app_slist.col(i) = get_screw(w_vir.col(i), q_vir);
  }

  // Affordance screw
  app_slist.col(4) = aff_screw;

  // Putting altogether
  Eigen::MatrixXd slist(robot_slist.rows(),
                        (robot_slist.cols() + app_slist.cols()));
  slist << robot_slist, app_slist;

  return slist;
}

// Get affordance screw
Eigen::Matrix<double, 6, 1> get_aff_screw_from_tag(
    const Eigen::Vector3d &w_aff, const std::string &reference_frame,
    const std::string &affordance_frame, tf2_ros::Buffer &tf_buffer) {

  // Extract end-effector location from TF data
  Eigen::Vector3d q_aff;
  const Eigen::Isometry3d tag_htm =
      AffordanceUtilROS::get_htm(reference_frame, affordance_frame, tf_buffer);
  q_aff = tag_htm.translation();

  return get_screw(w_aff, q_aff); // return screw axis
}

class CcAffordancePlannerRosNode {
public:
  CcAffordancePlannerRosNode(const std::string &robot_config_file_path,
                             const Eigen::VectorXd &aff_screw,
                             const double &aff_goal)
      : nh_("~"), aff_screw_(aff_screw), aff_goal_(aff_goal) {

    // Subscribers
    joint_states_sub_ =
        nh_.subscribe("/joint_states", 1000,
                      &CcAffordancePlannerRosNode::joint_states_cb_, this);

    // Extract robot config info
    const AffordanceUtil::RobotConfig &robotConfig =
        AffordanceUtil::robot_builder(robot_config_file_path);
    h_slist_ = robotConfig.Slist;
    M_ = robotConfig.M;
    tool_name_ = robotConfig.tool_name;
    const size_t real_robot_joints =
        robotConfig.joint_names.size() - 3; // Disregard virtual joint names
    joint_names_.assign(robotConfig.joint_names.begin(),
                        robotConfig.joint_names.begin() + real_robot_joints);
  }

  ~CcAffordancePlannerRosNode() {}

  void run_cc_affordance_planner() {

    // Extract robot screw list
    Eigen::MatrixXd robot_slist =
        h_slist_.leftCols(joint_states_.positions.size());

    // Read joint states
    // Capture initial configuration joint states
    std::string capture_joint_states_conf;
    std::cout << "Put the robot in the start configuration for affordance "
                 "execution. Done? y for yes."
              << std::endl;
    std::cin >> capture_joint_states_conf;

    if (capture_joint_states_conf != "y" && capture_joint_states_conf != "Y") {
      std::cout << "You indicated you are not ready to capture joint states"
                << std::endl;
      return;
    }
    // Take a second to read callbacks to ensure proper capturing of joint
    // states data
    ros::Rate loop_rate(4);
    for (int i = 0; i < 4; ++i) {
      ros::spinOnce();
      loop_rate.sleep();
    }
    // Compose cc model and affordance goal
    Eigen::MatrixXd cc_slist = compose_cc_model_slist(
        robot_slist, joint_states_.positions, M_, aff_screw_);

    // Call planner
    // Define affordance goal and create planner object
    /* const double affGoal = 1.0 * M_PI; */
    /* const double affGoal = 0.1; */
    Eigen::VectorXd thetalist0 = Eigen::VectorXd::Zero(cc_slist.cols());
    CcAffordancePlanner ccAffordancePlanner(cc_slist, thetalist0, aff_goal_);
    ccAffordancePlanner.affStep = 0.01;
    ccAffordancePlanner.taskOffset = 1;
    /* ccAffordancePlanner.affStep = 0.01; */

    // Run the planner
    PlannerResult plannerResult = ccAffordancePlanner.affordance_stepper();

    // Print the first point in the trajectory if planner succeeds
    std::vector<Eigen::VectorXd> solution = plannerResult.jointTraj;
    if (plannerResult.success) {
      std::cout
          << "Planner succeeded with "
          << plannerResult.trajFullorPartial
          /* << " solution. and here is the first point in the trajectory \n" */
          /* << solution.at(0) << std::endl; */
          << " solution\n"
          << std::endl;
      /* std::cout << "Here is the trajectory \n"; */
      /* for (size_t i = 0; i < solution.size(); i++) { */
      /*   std::cout << solution.at(i) << std::endl; */
      /* } */
    } else {
      std::cout << "No solution found" << std::endl;
    }

    // Convert the solution trajectory to ROS message type
    const double traj_time_step = 0.3;
    const control_msgs::FollowJointTrajectoryGoal goal =
        AffordanceUtilROS::follow_joint_trajectory_msg_builder(
            solution, joint_states_.positions, joint_names_, traj_time_step);
    ROS_INFO_STREAM("Here is the coveted ROS msg: " << goal);

    // Visualize plan
    // Send to move_plan_and_viz_server to visualize planning
    std::string plan_and_viz_service_name = "/moveit_plan_and_viz_server";
    ros::ServiceClient moveit_plan_and_viz_client =
        nh_.serviceClient<moveit_plan_and_viz::MoveItPlanAndViz>(
            plan_and_viz_service_name);

    moveit_plan_and_viz::MoveItPlanAndViz moveit_plan_and_viz_goal;
    moveit_plan_and_viz_goal.request.joint_traj = goal.trajectory;

    ROS_INFO_STREAM("Waiting for " << plan_and_viz_service_name
                                   << " service to be available");
    ros::Duration timeout(60.0);
    if (!ros::service::waitForService(plan_and_viz_service_name)) {
      ROS_INFO_STREAM(plan_and_viz_service_name
                      << " service isn't available. Timed out waiting");
    } else {
      ROS_INFO_STREAM(plan_and_viz_service_name
                      << " service found. Now sending goal.");
    }

    if (moveit_plan_and_viz_client.call(moveit_plan_and_viz_goal)) {
      ROS_INFO_STREAM("Calling " << plan_and_viz_service_name
                                 << " service was successful.");
      std::string executionConfirm;
      std::cout << "Ready to execute the trajectory? y to confirm" << std::endl;
      std::cin >> executionConfirm;

      if (executionConfirm != "y") {
        std::cout << "You indicated you are not ready to execute trajectory"
                  << std::endl;
        return;
      }

      // Execute directly
      // Create the connection to the action server
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
          client("/spot_arm/arm_controller/follow_joint_trajectory", true);
      client.waitForServer(); // Waits until the action server is up and running

      client.sendGoal(goal);
      client.waitForResult();
    }

    else {
      ROS_INFO_STREAM("Failed to call " << plan_and_viz_service_name
                                        << " service.");
    }
    // Execute plan
  }

private:
  // ROS variables
  ros::NodeHandle nh_;
  ros::Subscriber joint_states_sub_;
  AffordanceUtilROS::JointTrajPoint joint_states_;
  // Robot data
  Eigen::MatrixXd h_slist_;
  std::vector<std::string> joint_names_;
  Eigen::MatrixXd M_;
  std::string tool_name_;
  // Affordance data
  Eigen::Matrix<double, 6, 1> aff_screw_;
  double aff_goal_;

  // Callback function for the joint_states subscriber
  void joint_states_cb_(const sensor_msgs::JointState::ConstPtr &msg) {
    joint_states_ = AffordanceUtilROS::get_ordered_joint_states(
        msg,
        joint_names_); // Takes care of filtering and ordering the joint_states
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "cc_affordance_planner_node");

  // Buffer to lookup tf data for the tag
  tf2_ros::Buffer tf_buffer;

  // Find CC Affordance Description for the robot
  const std::string package_name = "cc_affordance_spot_description";
  const std::string rel_dir = "/config/";
  const std::string filename = package_name + ".yaml";
  std::string robot_cc_description_path;
  try {
    robot_cc_description_path =
        get_filepath_inside_pkg(package_name, rel_dir, filename);
  } catch (const CustomException &e) {
    // Catch and handle the custom exception
    std::cerr << "Caught exception: " << e.what() << std::endl;
    return 1;
  }

  // Compose affordance screw
  const bool aff_from_tag = false;
  const Eigen::Vector3d w_aff(0, 0, 1); // To be hard-coded as needed
  const Eigen::Vector3d q_aff(0, 0, 0); // To be hard-coded as needed
  Eigen::Matrix<double, 6, 1> aff_screw;
  if (aff_from_tag) {
    const std::string affordance_frame =
        "affordance_frame"; // Name of the tag frame
    const std::string reference_frame =
        "arm0_base_link"; // Maybe we can move the aff_from_tag portion inside
                          // the class later since reference_frame info is
                          // available there.
    aff_screw = get_aff_screw_from_tag(w_aff, reference_frame, affordance_frame,
                                       tf_buffer);
  } else {
    aff_screw = get_screw(w_aff, q_aff);
  }

  // Set affordance goal
  const double aff_goal = 1.57;

  CcAffordancePlannerRosNode ccAffordancePlannerRosNode(
      robot_cc_description_path, aff_screw, aff_goal);
  // Run cc affordance planner
  ccAffordancePlannerRosNode.run_cc_affordance_planner();

  return 0;
}
