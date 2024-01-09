///////////////////////////////////////////////////////////////////////////////
//      Title     : affordance_util_ros.hpp
//      Project   : affordance_util_ros
//      Created   : Fall 2023
//      Author    : Janak Panthi (Crasun Jans)
//      Copyright : Copyright© The University of Texas at Austin, 2014-2026. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative license is explicitly included within the text of
//          each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation, including but not limited to those resulting from
//          defects in software and/or documentation, or loss or inaccuracy of
//          data of any kind.
//
///////////////////////////////////////////////////////////////////////////////
#ifndef AFFORDANCE_UTIL_ROS
#define AFFORDANCE_UTIL_ROS

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <filesystem>
#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>

namespace AffordanceUtilROS
{

class CustomException : public std::exception
{
  public:
    CustomException(const char *message);

    /**
     * @brief Overrides the what function in std::exception so as to throw a custom exception
     *
     * @return char containing exception message
     */
    virtual const char *what() const noexcept override;

  private:
    std::string msg; // custom exception message
};

/**
 * @brief Given a ROS package name, relative directory path, and filename inside that directory, returns full path to
 * the file.
 *
 * @param package_name std::string containing ROS package name. Example: "spot_description"
 * @param rel_dir std::string containing relative directory path. Example: "/config/"
 * @param filename std::string containing name of a desired file inside the relative directory. Example: package_name +
 * ".yaml"
 *
 * @return std::string containing full path to the desired file
 */
std::string get_filepath_inside_pkg(const std::string &package_name, const std::string &rel_dir,
                                    const std::string &filename);

/**
 * @brief Struct to hold a joint trajectory point in the form of positions and timestamp
 */
struct JointTrajPoint
{
    Eigen::VectorXd positions;
    double timestamp; // In seconds
};

/**
 * @brief Given the source file's full path as __FILE__ (no quotation marks) and relative path from the source file,
 * returns the full path to the relative directory
 *
 * @param full_path std::string containing full path to source file. Send as __FILE__
 * @param relative_path std::string containing relative path from the source file
 *
 * @return std::string containing full path to relative directory
 */
std::string get_abs_path_to_rel_dir(const std::string &full_path, const std::string &relative_path);

/**
 * @brief Given a ROS joint trajectory and a vector of joint names specifying a desired
 * order, returns the joint trajectory with joint positions in the correct order.
 *
 * @param traj trajectory_msgs::JointTrajectory containing ROS joint trajectory
 * @param joint_name_order std::vector<std::string> containing joint names specifying a desired order
 *
 * @return std::vector<JointTrajPoint> containing joint trajectory with joint positions in correct order
 */
std::vector<JointTrajPoint> get_ordered_joint_traj(const trajectory_msgs::JointTrajectory &traj,
                                                   const std::vector<std::string> &joint_name_order);

/**
 * @brief Given a ROS joint_states message and a vector of joint names specifying a desired
 * order, returns the timestamp and joint positions in the correct order.
 *
 * @param joint_states sensor_msgs::JointState containing joint state
 * @param joint_name_order std::vector<std::string> containing joint names specifying a desired order
 *
 * @return JointTrajPoint containing the joint trajectory point (i.e. timestamp and joint positions) in the correct
 * order
 */
JointTrajPoint get_ordered_joint_states(const sensor_msgs::JointState::ConstPtr &joint_states,
                                        const std::vector<std::string> &joint_name_order);

/**
 * @brief Given a space frame name, body frame name, and tf buffer, returns the transformation of the body frame with
 * respect to the space frame.
 *
 * @param space_frame std::string containing space_frame name
 * @param body_frame std::string containing body_frame name
 * @param tf_buffer tf2_ros::Buffer containing an empty buffer
 *
 * @return Eigen::Isometry3d containing the transformation of the body frame wrt to space frame
 */
Eigen::Isometry3d get_htm(const std::string &space_frame, const std::string &body_frame, tf2_ros::Buffer &tf_buffer);

/**
 * @brief Given a bare differential joint trajectory (i.e. just a vector of differential joint trajectory points from a
 * reference point), reference trajectory point, joint names, and desired time step between trajectory points, returns
 * a ROS follow_joint_trajectory message
 *
 * @param bare_trajectory std::vector<Eigen::VectorXd> containing a bare differential joint trajectory (i.e. just a
 * vector of differential joint trajectory points from a reference point) vector
 * @param config_offset Eigen::VectorXd containing reference trajectory point
 * @param joint_names std::vector<std::string> containing joint names. Ensure the order of the joint names matches the
 * order of joint positions
 * @param time_step Optional double containing time step. Default is 0.05
 *
 * @return control_msgs::FollowJointTrajectoryGoal containing ready-to-use ROS follow_joint_trajectory message
 */
control_msgs::FollowJointTrajectoryGoal follow_joint_trajectory_msg_builder(
    const std::vector<Eigen::VectorXd> &bare_trajectory, const Eigen::VectorXd &config_offset,
    const std::vector<std::string> &joint_names, const double &time_step = 0.05);
} // namespace AffordanceUtilROS

#endif // AFFORDANCE_UTIL_ROS
