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
  double joint_states_timestamp;

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
    joint_states_timestamp = msg->header.stamp.toSec();
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
      const std::vector<double> &config_offset,
      const double &time_step = 0.05) {
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
        trajectory_point.positions[i] = point[i] + config_offset[i];
      }

      j++;
    }

    return fjtg_msg;
  }

  Eigen::MatrixXd robot_builder() {

    //** Creation of screw axes
    //* Screw axis locations from base frame in home position
    Eigen::MatrixXd q(3, 6);

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
    Eigen::MatrixXd w(3, 6);
    w.col(0) << 0, 0, 1;
    w.col(1) << 0, 1, 0;
    w.col(2) << 0, 1, 0;
    w.col(3) << 1, 0, 0;
    w.col(4) << 0, 1, 0;
    w.col(5) << 1, 0, 0;
    /* w.col(6) << 1, 0, 0;  // Imaginary joint */
    /* w.col(7) << 0, 1, 0;  // Imaginary joint */
    /* w.col(8) << 0, 0, 1;  // Imaginary joint */
    /* w.col(9) << -1, 0, 0; // Affordance */

    // Get ee q-vector from tf data
    /* Eigen::Isometry3d eeHtm = get_htm(targetFrame_, "arm0_fingers"); */
    /* Eigen::Vector3d q_ee = eeHtm.translation(); */
    /* std::cout << "Here is q_ee: \n" << q_ee << std::endl; */

    /* // Hard-coded ee q-vector */
    /* const Eigen::Vector3d q_ee(0.8605, -0.0002, 0.0818); */

    /* // Affordance and virtual joint location */
    /* const double affOffset = 0.1; */
    /* const Eigen::Vector3d q_aff = */
    /*     q_ee + Eigen::Vector3d(0, affOffset, 0); // q-vector for affordance
     */
    /* q.col(6) = q_ee;                             // imaginary joint */
    /* q.col(7) = q_ee;                             // imaginary joint */
    /* q.col(8) = q_ee;                             // imaginary joint */
    /* q.col(9) = q_aff;                            // affordance */

    // Compute screw axes
    /* Eigen::MatrixXd slist(6, 10); */
    Eigen::MatrixXd slist(6, 6);

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
    /* slist.col(9) = (Eigen::VectorXd(6) << 0, 0, -1, 0, 0, 0).finished(); */
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
  ros::init(argc, argv, "cc_affordance_planner_data_recorder");
  ros::NodeHandle node_handle;

  CcAffordancePlannerNode cAPN(node_handle);
  ros::Rate loop_rate(4);

  std::string target_frame = "arm0_base_link";
  /* std::string source_frame = "arm0_fingers"; */
  std::string source_frame = "arm0_tool0";

  // Open a CSV file for writing
  std::ofstream csvFile("act_tf_and_joint_states_data.csv");

  // Check if the file was opened successfully
  if (!csvFile.is_open()) {
    std::cerr << "Failed to open the CSV file for writing. " << std::endl;
    return 1;
  }

  for (const std::string &joint_name : cAPN.joint_names) {
    csvFile << joint_name << ",";
  }
  csvFile << "EE x,EE y, EE z,"; // CSV header
  csvFile << "timestamp" << std::endl;

  // Ask if we should start recording data

  std::cout << "Start recording actual data? ";
  std::string conf;
  std::cin >> conf;

  if (conf != "y") {
    std::cout << "You indicated you are not ready to record data. Exiting."
              << std::endl;
    return 1;
  }

  Eigen::MatrixXd slist = cAPN.robot_builder();
  CcAffordancePlanner fkinSpaceTool(slist, Eigen::VectorXd::Zero(slist.cols()),
                                    0.3); // Meaningless constructor values just
  Eigen::MatrixXd fkinSlist(6, 6);
  fkinSlist = slist.leftCols(6).eval();
  Eigen::MatrixXd M = Eigen::Matrix4d::Identity();
  // Set the translation part
  /* M(0, 3) = 0.9383; */
  M(0, 3) = 0.94;
  /* M(1, 3) = 0.0005; */
  M(1, 3) = -0.0004;
  /* M(2, 3) = 0.0664; */
  M(2, 3) = 0.0656;

  while (ros::ok()) {

    // Write joint_states data to file
    for (int i = 0; i < cAPN.joint_states.size(); ++i) {
      csvFile << cAPN.joint_states[i] << ",";
    }

    /* /1* // Write TF data to file *1/ */
    Eigen::Isometry3d ee_htm = cAPN.get_htm(target_frame, source_frame);
    csvFile << ee_htm.translation().x() << "," << ee_htm.translation().y()
            << "," << ee_htm.translation().z() << ",";
    // Write computed predicted ee htm to file
    /* Eigen::MatrixXd act_ee_htm = */
    /*     fkinSpaceTool.FKinSpace(M, fkinSlist, cAPN.joint_states); */
    /* csvFile << act_ee_htm(0, 3) << "," << act_ee_htm(1, 3) << "," */
    /*         << act_ee_htm(2, 3) << ","; */

    // Write timestamp to file
    csvFile << cAPN.joint_states_timestamp << std::endl;

    //
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
