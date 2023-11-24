#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <boost/thread.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <tuple>
#include <unordered_map>
/*
Description: This node listens to a specified follow_joint_trajectory action
server goal and writes onto separate csv files predicted and actual joint states
Input: Action server topic where the joint trajectory is sent, and the path to
the config file that contains robot config info to compute forward kinematics
and EE location. Author: Crasun Jans
*/

struct JointTrajPoint {
  Eigen::VectorXd positions;
  double timestamp; // In seconds
};

class JointTrajAndTfRecorder {
public:
  JointTrajAndTfRecorder(const std::string &robot_config_file_path,
                         const std::string &joint_traj_topic)
      : tfBuffer_(), tfListener_(tfBuffer_) {

    // Subscribers
    joint_traj_sub_ =
        nh_.subscribe(joint_traj_topic + "/goal", 1,
                      &JointTrajAndTfRecorder::joint_traj_sub_cb_, this);
    joint_traj_result_sub_ =
        nh_.subscribe(joint_traj_topic + "/result", 1,
                      &JointTrajAndTfRecorder::joint_traj_result_sub_cb_, this);
    joint_states_sub_ = nh_.subscribe(
        "/joint_states", 1, &JointTrajAndTfRecorder::joint_states_cb_, this);

    // Get path to the config directory
    filepath_prefix_ = get_config_directory_path();

    // Extract robot config info
    const AffordanceUtil::RobotConfig &robotConfig =
        AffordanceUtil::robot_builder(robot_config_file_path);
    const size_t &noFJoints =
        robotConfig.joint_names.size() - 3; // Disregard appended virtual joints
    slist_ = robotConfig.Slist.leftCols(noFJoints);
    joint_names_.assign(robotConfig.joint_names.begin(),
                        robotConfig.joint_names.begin() + noFJoints);
    M_ = robotConfig.M;
    tool_name_ = robotConfig.tool_name;
    ref_frame_name_ = robotConfig.ref_frame_name;
    std::cout << "Tool name: " << tool_name_ << std::endl;
    std::cout << "Ref frame name: " << ref_frame_name_ << std::endl;
    std::cout << " Joint names: ";
    for (auto &joint_name : joint_names_) {
      std::cout << joint_name << " ";
    }
    std::cout << std::endl;
    std::cout << "M: \n" << M_ << std::endl;

    // Initialize joint_states to the same size as joint_names_
    /* joint_states_ = Eigen::VectorXd::Zero(joint_names_.size()); */

    // Concurrently, while writing predicted data, we'll write actual data as
    // well, because while predicted data is being written, action server is
    // probably executing joint movement already
    act_data_writer_thread_ =
        boost::thread(&JointTrajAndTfRecorder::write_act_data_, this);
  }

  ~JointTrajAndTfRecorder() {
    // Join the threads before exiting
    act_data_writer_thread_.join();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_traj_sub_; // subscriber to the joint trajectory topic
  ros::Subscriber
      joint_traj_result_sub_; // subscriber to the joint trajectory topic
  ros::Subscriber joint_states_sub_;
  tf2_ros::Buffer tfBuffer_;              // buffer to hold tf data
  tf2_ros::TransformListener tfListener_; // listener object for tf data
  geometry_msgs::TransformStamped
      transformStamped_; // ros message to hold transform info
  Eigen::MatrixXd slist_;
  Eigen::MatrixXd M_;
  bool act_write_flag_ = false;
  std::vector<std::string> joint_names_;
  std::string tool_name_;
  boost::thread pred_data_writer_thread_;
  boost::thread act_data_writer_thread_;
  boost::mutex mutex_;
  bool cb_called_ = false;
  /* Eigen::VectorXd joint_states_; */
  JointTrajPoint joint_states_;
  double joint_states_timestamp_;
  std::string filepath_prefix_;
  std::string ref_frame_name_;
  bool as_active_ = true;

  // returns path to config directory
  std::string get_config_directory_path() {
    std::string full_path = __FILE__;
    std::filesystem::path source_file_path(full_path);
    std::filesystem::path source_file_parent_path =
        source_file_path.parent_path(); // directory containing the source file
    const std::string filepath_prefix =
        source_file_parent_path.string() + "/../data/";
    return filepath_prefix;
  }

  void joint_traj_sub_cb_(
      const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr &msg) {
    // Lock the mutex before updating data shared between threads - doing this
    // as good practice. There isn't anything shared except for the following
    // flag.
    {
      boost::lock_guard<boost::mutex> lock(mutex_);
      cb_called_ = true;
    } // scope for mutex

    // Extract the predicted trajectory and write to file
    ROS_INFO_STREAM(
        "Here is the predicted trajectory: " << msg->goal.trajectory);
    const auto &unordered_pred_traj_ = msg->goal.trajectory;
    std::vector<JointTrajPoint> pred_traj_ =
        order_joint_states(unordered_pred_traj_, joint_names_);
    std::cout << "Writing predicted data now" << std::endl;
    write_pred_data(pred_traj_);
    std::cout << "Finished writing predicted data" << std::endl;
  }

  void joint_traj_result_sub_cb_(
      const control_msgs::FollowJointTrajectoryActionResult::ConstPtr &msg) {

    boost::lock_guard<boost::mutex> lock(mutex_);
    ros::Rate cb_sleep(0.2);
    cb_sleep.sleep();
    /* boost::this_thread::sleep_for(boost::chrono::seconds(5)); */
    as_active_ = false; // If the result callback is called then, it means the
                        // action server is done
  }
  // Callback function to process joint states
  void joint_states_cb_(const sensor_msgs::JointState::ConstPtr &msg) {
    boost::lock_guard<boost::mutex> lock(mutex_);
    /* // Collect joint state data for the specified joints in the specified
     */
    /* // order */
    /* int j = 0; // index for selected_joint_positions_eigen */
    /* joint_states_timestamp_ = msg->header.stamp.toSec(); */
    /* /1* std::cout << "MoveIt joint names: \n"; *1/ */
    /* /1* for (auto &moveit_joint_name : msg->name) { *1/ */
    /* /1*   std::cout << moveit_joint_name << " "; *1/ */
    /* /1* } *1/ */
    /* /1* std::cout << std::endl; *1/ */
    /* for (const std::string &joint_name : joint_names_) { */
    /*   for (size_t i = 0; i < msg->name.size(); ++i) { */
    /*     if (msg->name[i] == joint_name) { */
    /*       /1* selected_joint_positions.push_back(msg->position[i]); *1/ */
    /*       joint_states_[j] = msg->position[i]; */
    /*       j++; */
    /*       break; // Move to the next joint name */
    /*     } */
    /*   } */
    /* } */
    joint_states_ = order_joint_states(msg, joint_names_);
    /* joint_states_timestamp_ = msg->header.stamp.toSec(); */
  }

  std::vector<JointTrajPoint>
  order_joint_states(const trajectory_msgs::JointTrajectory &traj,
                     const std::vector<std::string> &joint_name_order) {

    std::vector<JointTrajPoint> ordered_traj;

    size_t nof_joints =
        joint_name_order
            .size(); // this is the relevant number of joints. Even if the
                     // trajectory contains more joints, we'll only extract
                     // relevant joint positions and in order

    // First, we determine the order that the indices need to be in to match the
    // joint_name_order.
    //  Store unordered joint names in a map and assign indices
    std::unordered_map<std::string, size_t> joint_index_map;
    for (size_t i = 0; i < traj.joint_names.size(); ++i) {
      joint_index_map[traj.joint_names[i]] = i;
    }

    // Determine the correct index order of the unordered joint names to match
    // ordered joint names
    std::vector<size_t> index_order(nof_joints);
    for (size_t i = 0; i < nof_joints; ++i) {
      const std::string &joint_name = joint_name_order[i];

      auto it = joint_index_map.find(joint_name);
      if (it != joint_index_map.end()) {
        index_order[i] = it->second;
      } else {
        // Handle the case where the joint name is not found
        std::cerr << "Joint name not found: " << joint_name << std::endl;
      }
    }

    // Iterate through each trajectory point and extract joint states in the
    // correct order as well as timestamps
    for (const auto &point : traj.points) {
      JointTrajPoint ordered_traj_point;
      ordered_traj_point.positions.conservativeResize(nof_joints);

      // Extract the joint positions in correct order and fill it in the result
      for (size_t i = 0; i < joint_name_order.size(); ++i) {
        ordered_traj_point.positions[i] = point.positions[index_order[i]];
      }

      // Retrieve timestamp as well and push it into the result vector
      ordered_traj_point.timestamp = point.time_from_start.toSec();
      ordered_traj.push_back(ordered_traj_point);
    }

    return ordered_traj;
  }

  JointTrajPoint
  order_joint_states(const sensor_msgs::JointState::ConstPtr &joint_states,
                     const std::vector<std::string> &joint_name_order) {

    size_t nof_joints =
        joint_name_order
            .size(); // this is the relevant number of joints. Even if the
                     // trajectory contains more joints, we'll only extract
                     // relevant joint positions and in order
    JointTrajPoint ordered_joint_states;
    ordered_joint_states.positions.conservativeResize(nof_joints);

    // First, we determine the order that the indices need to be in to match the
    // joint_name_order.
    //  Store unordered joint names in a map and assign indices
    // Create a mapping between joint names and indices
    std::unordered_map<std::string, size_t> joint_index_map;
    for (size_t i = 0; i < joint_states->name.size(); ++i) {
      joint_index_map[joint_states->name[i]] = i;
    }

    // Determine the index order
    std::vector<size_t> index_order(nof_joints);
    for (size_t i = 0; i < nof_joints; ++i) {
      const std::string &joint_name = joint_name_order[i];
      auto it = joint_index_map.find(joint_name);
      if (it != joint_index_map.end()) {
        index_order[i] = it->second;
      } else {
        // Handle the case where the joint name is not found
        std::cerr << "Joint name not found: " << joint_name << std::endl;
      }
    }

    // Extract the joint positions in correct order
    for (size_t i = 0; i < nof_joints; ++i) {
      ordered_joint_states.positions[i] =
          joint_states->position[index_order[i]];
    }

    // Extract and set the timestamp as well
    ordered_joint_states.timestamp = joint_states->header.stamp.toSec();

    return ordered_joint_states;
  }

  Eigen::Isometry3d get_htm_(std::string targetFrame_,
                             std::string referenceFrame_) {

    Eigen::Isometry3d htm; // Output
    // Query the listener for the desired transformation at the latest
    // available time ros::Time::now()

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

  void write_pred_data(const std::vector<JointTrajPoint> &pred_traj_) {

    // Write predicted data to file
    // Open a CSV file for writing
    std::ofstream csvFile(filepath_prefix_ +
                          "pred_tf_and_joint_states_data.csv");
    std::cout << "Here is the predicted data writing path: "
              << filepath_prefix_ + "pred_tf_and_joint_states_data.csv"
              << std::endl;
    // Check if the file was opened successfully
    if (!csvFile.is_open()) {
      std::cerr << "Failed to open the predicted data CSV file for writing. "
                << std::endl;
      return;
    }

    /* Headers */
    // Joint_names
    for (const std::string &joint_name : joint_names_) {
      csvFile << joint_name << ",";
    }
    // EE position and timestamp
    csvFile << "Pred EE x,Pred EE y,Pred EE z,"; // CSV header
    csvFile << "Timestamp" << std::endl;

    /* std::cout << "Here is the trajectory: " << pred_traj_ << std::endl; */
    /* Data */
    /* for (const auto &goalPoint : pred_traj_) { */
    for (const auto &pred_traj_point : pred_traj_) {
      /* const auto &thetalist = goalPoint.positions; */

      // Joint positions
      for (size_t i = 0; i < pred_traj_point.positions.size(); ++i) {
        csvFile << pred_traj_point.positions[i] << ",";
      }

      // Store thetalist as an Eigen::VectorXd type to use it with the
      // FkinSpace function
      /* Eigen::VectorXd thetalist_vec = */
      /*     Eigen::Map<const Eigen::VectorXd>(thetalist.data(),
       * thetalist.size()); */

      // EE position
      Eigen::MatrixXd ee_htm =
          /* AffordanceUtil::FKinSpace(M_, slist_, thetalist_vec); */
          AffordanceUtil::FKinSpace(M_, slist_, pred_traj_point.positions);
      csvFile << ee_htm(0, 3) << "," << ee_htm(1, 3) << "," << ee_htm(2, 3)
              << ",";

      // Timestamp
      double timestamp = pred_traj_point.timestamp;
      csvFile << timestamp << std::endl;
      csvFile << std::endl;
    }
  }
  void write_act_data_() {
    // Open a CSV file for writing
    std::ofstream csvFile(filepath_prefix_ +
                          "act_tf_and_joint_states_data.csv");
    // Check if the file was opened successfully
    if (!csvFile.is_open()) {
      std::cerr << "Failed to open the actual data CSV file for writing."

                << std::endl;
      return;
    }

    while (ros::ok()) {

      if (cb_called_) {
        std::cout << "Writing actual data now" << std::endl;

        // Write actual joint_states and tool TF data to file
        const std::string &target_frame = ref_frame_name_;
        const std::string &source_frame = tool_name_;

        for (const std::string &joint_name : joint_names_) {
          csvFile << joint_name << ",";
        }
        csvFile << "Act EE x,Act EE y,Act EE z,"; // CSV header
        csvFile << "timestamp" << std::endl;

        while (as_active_) {
          // Write joint_states data to file
          for (int i = 0; i < joint_states_.positions.size(); ++i) {
            csvFile << joint_states_.positions[i] << ",";
          }

          // Write TF data to file
          Eigen::Isometry3d ee_htm = get_htm_(target_frame, source_frame);

          csvFile << ee_htm.translation().x() << "," << ee_htm.translation().y()
                  << "," << ee_htm.translation().z() << ",";
          // EE position
          /* Eigen::MatrixXd ee_htm = */
          /*     AffordanceUtil::FKinSpace(M_, slist_, joint_states_); */
          /* csvFile << ee_htm(0, 3) << "," << ee_htm(1, 3) << "," */
          /*         << ee_htm(2, 3) << ","; */

          // Write timestamp to file
          csvFile << joint_states_.timestamp << std::endl;
          boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
        }

        {
          // Create a lock_guard and lock the mutex
          boost::lock_guard<boost::mutex> lock(mutex_);
          // Reset the callback flag
          cb_called_ = false;
        } // scope for mutex

        csvFile.close();
        std::cout << "Finished writing actual data" << std::endl;
      }

      // Sleep for a little to avoid busy-waiting
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "joint_traj_and_tf_recorder");

  // Furnish the filepath where the robot config yaml file is located and
  // supply the topic where the goal topic for the
  // follow_joint_trajectory_action_server
  const std::string robot_config_file_path =
      "/home/crasun/spot_ws/src/cc_affordance_planner/config/"
      "robot_setup.yaml";
  const std::string joint_traj_topic =
      "/spot_arm/arm_controller/follow_joint_trajectory";
  JointTrajAndTfRecorder jointTrajAndTfRecorder(robot_config_file_path,
                                                joint_traj_topic);
  ros::spin();

  return 0;
}
