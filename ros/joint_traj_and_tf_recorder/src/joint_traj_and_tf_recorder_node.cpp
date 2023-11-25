#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <affordance_util_ros/affordance_util_ros.hpp>
#include <boost/thread.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
/*
Author: Crasun Jans
*/

class JointTrajAndTfRecorder {
public:
  JointTrajAndTfRecorder(const std::string &robot_config_file_path,
                         const std::string &joint_traj_topic)
      : tfBuffer_(), tfListener_(tfBuffer_) {

    // Subscribers
    joint_traj_sub_ =
        nh_.subscribe(joint_traj_topic + "/goal", 10,
                      &JointTrajAndTfRecorder::joint_traj_sub_cb_, this);
    joint_traj_result_sub_ =
        nh_.subscribe(joint_traj_topic + "/result", 10,
                      &JointTrajAndTfRecorder::joint_traj_result_sub_cb_, this);
    joint_states_sub_ = nh_.subscribe(
        "/joint_states", 1000, &JointTrajAndTfRecorder::joint_states_cb_, this);

    // Get path to the directory where we wills av
    filepath_prefix_ =
        AffordanceUtilROS::get_abs_path_to_rel_dir(__FILE__, "/../data/");

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
  AffordanceUtilROS::JointTrajPoint joint_states_;
  double joint_states_timestamp_;
  std::string filepath_prefix_;
  std::string ref_frame_name_;
  bool as_active_ = false;

  void joint_traj_sub_cb_(
      const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr &msg) {
    // Lock the mutex before updating data shared between threads - doing this
    // as good practice. There isn't anything shared except for the following
    // flag.
    {
      boost::lock_guard<boost::mutex> lock(mutex_);
      cb_called_ = true;
      as_active_ = true;
    } // scope for mutex

    // Extract the predicted trajectory and write to file
    ROS_INFO_STREAM(
        "Here is the predicted trajectory: " << msg->goal.trajectory);
    const auto &unordered_pred_traj_ = msg->goal.trajectory;
    std::vector<AffordanceUtilROS::JointTrajPoint> pred_traj_ =
        AffordanceUtilROS::get_ordered_joint_traj(unordered_pred_traj_,
                                                  joint_names_);
    std::cout << "Writing predicted data now" << std::endl;
    write_pred_data(pred_traj_);
    std::cout << "Finished writing predicted data" << std::endl;
  }

  void joint_traj_result_sub_cb_(
      const control_msgs::FollowJointTrajectoryActionResult::ConstPtr &msg) {

    boost::lock_guard<boost::mutex> lock(mutex_);
    as_active_ = false; // If the result callback is called then, it means the
                        // action server is done
  }
  // Callback function to process joint states
  void joint_states_cb_(const sensor_msgs::JointState::ConstPtr &msg) {
    boost::lock_guard<boost::mutex> lock(mutex_);
    /* // Collect joint state data for the specified joints in the specified
     */
    joint_states_ =
        AffordanceUtilROS::get_ordered_joint_states(msg, joint_names_);
  }

  void write_pred_data(
      const std::vector<AffordanceUtilROS::JointTrajPoint> &pred_traj_) {

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
    ros::Rate loop_rate(10);
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

      bool cb_called_copy;
      {
        boost::lock_guard<boost::mutex> lock(mutex_);
        cb_called_copy = cb_called_;
      }
      if (cb_called_copy) {
        std::cout << "Writing actual data now" << std::endl;

        // Write actual joint_states and tool TF data to file
        const std::string &target_frame = ref_frame_name_;
        const std::string &source_frame = tool_name_;

        for (const std::string &joint_name : joint_names_) {
          csvFile << joint_name << ",";
        }
        csvFile << "Act EE x,Act EE y,Act EE z,"; // CSV header
        csvFile << "timestamp"
                << "\n";

        bool as_active_copy;
        {
          boost::lock_guard<boost::mutex> lock(mutex_);
          as_active_copy = as_active_;
        }
        /* while (as_active_copy) { */
        while (ros::ok()) {
          AffordanceUtilROS::JointTrajPoint joint_states_copy;
          {
            boost::lock_guard<boost::mutex> lock(mutex_);
            joint_states_copy = joint_states_;
          }
          // Write joint_states data to file
          for (int i = 0; i < joint_states_copy.positions.size(); ++i) {
            csvFile << joint_states_copy.positions[i] << ",";
          }

          // Write TF data to file
          /* Eigen::Isometry3d ee_htm = get_htm_(target_frame, source_frame);
           */

          /* csvFile << ee_htm.translation().x() << "," <<
           * ee_htm.translation().y() */
          /*         << "," << ee_htm.translation().z() << ","; */
          // EE position
          Eigen::MatrixXd ee_htm = AffordanceUtil::FKinSpace(
              M_, slist_, joint_states_copy.positions);

          csvFile << ee_htm(0, 3) << "," << ee_htm(1, 3) << "," << ee_htm(2, 3)
                  << ",";

          // Write timestamp to file
          csvFile << joint_states_copy.timestamp << "\n";
          std::cout << "Joint states: " << joint_states_copy.positions
                    << std::endl;
          // Check the as_active_ flag again
          {
            boost::lock_guard<boost::mutex> lock(mutex_);
            as_active_copy = as_active_;
          }
          /* boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); */
          loop_rate.sleep();
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
      /* boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); */
      loop_rate.sleep();
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
