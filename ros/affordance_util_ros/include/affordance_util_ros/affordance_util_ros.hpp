/* Author: Crasun Jans */
#ifndef AFFORDANCE_UTIL_ROS
#define AFFORDANCE_UTIL_ROS

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <filesystem>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>

namespace AffordanceUtilROS {

struct JointTrajPoint {
  Eigen::VectorXd positions;
  double timestamp; // In seconds
};

std::string get_abs_path_to_rel_dir(const std::string &full_path,
                                    const std::string &relative_path);

std::vector<JointTrajPoint>
get_ordered_joint_traj(const trajectory_msgs::JointTrajectory &traj,
                       const std::vector<std::string> &joint_name_order);

JointTrajPoint
get_ordered_joint_states(const sensor_msgs::JointState::ConstPtr &joint_states,
                         const std::vector<std::string> &joint_name_order);

Eigen::Isometry3d get_htm(const std::string &target_frame,
                          const std::string &reference_frame,
                          tf2_ros::Buffer &tf_buffer);

control_msgs::FollowJointTrajectoryGoal follow_joint_trajectory_msg_builder(
    const std::vector<Eigen::VectorXd> &bare_trajectory,
    const Eigen::VectorXd &config_offset,
    const std::vector<std::string> &joint_names,
    const double &time_step = 0.05);
} // namespace AffordanceUtilROS

#endif // AFFORDANCE_UTIL_ROS
