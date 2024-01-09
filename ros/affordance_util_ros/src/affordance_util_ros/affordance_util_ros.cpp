#include <affordance_util_ros/affordance_util_ros.hpp>
namespace AffordanceUtilROS
{

CustomException::CustomException(const char *message) : msg(message) {}

// Override the what() function to provide a description of the exception
const char *CustomException::what() const noexcept { return msg.c_str(); }

std::string get_filepath_inside_pkg(const std::string &package_name, const std::string &rel_dir,
                                    const std::string &filename)
{
    try
    {
        std::string full_filepath; // output of the function

        // Get the path to the package
        const std::string package_path = ros::package::getPath(package_name);

        // Build full filepath and check for errors
        if (!package_path.empty())
        {
            full_filepath = package_path + rel_dir + filename;
        }
        else
        {
            throw CustomException(("Failed to find path for package '" + package_name).c_str());
        }

        // Make sure the file exists
        if (!std::filesystem::exists(full_filepath))
        {
            throw CustomException(("File does not exist at this path: '" + full_filepath).c_str());
        }

        return full_filepath;
    }
    // throw runtime error if exception is caught
    catch (const CustomException &e)
    {
        std::cerr << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error("Cannot proceed without building a valid filepath to the ROS package file");
    }
}

std::string get_abs_path_to_rel_dir(const std::string &full_path, const std::string &relative_path)
{
    std::filesystem::path source_file_path(full_path);
    std::filesystem::path source_file_parent_path =
        source_file_path.parent_path();                                          // Directory containing the source file
    const std::string output = source_file_parent_path.string() + relative_path; // Full path to relative directory

    return output;
}

std::vector<JointTrajPoint> get_ordered_joint_traj(const trajectory_msgs::JointTrajectory &traj,
                                                   const std::vector<std::string> &joint_name_order)
{

    std::vector<JointTrajPoint> ordered_traj;

    const size_t nof_joints = joint_name_order.size(); // This is the relevant number of joints. Even if the
                                                       // trajectory contains more joints, we'll only extract
                                                       // relevant joint positions and in order

    // First, we determine the order that the indices need to be in to match
    // the joint_name_order.
    //  Store unordered joint names in a map and assign indices
    std::unordered_map<std::string, size_t> joint_index_map;
    for (size_t i = 0; i < traj.joint_names.size(); ++i)
    {
        joint_index_map[traj.joint_names[i]] = i;
    }

    // Determine the correct index order of the unordered joint names to match
    // ordered joint names
    std::vector<size_t> index_order(nof_joints);
    for (size_t i = 0; i < nof_joints; ++i)
    {
        const std::string &joint_name = joint_name_order[i];

        auto it = joint_index_map.find(joint_name);
        if (it != joint_index_map.end())
        {
            index_order[i] = it->second;
        }
        else
        {
            // Handle the case where the joint name is not found
            std::cerr << "Joint name not found: " << joint_name << std::endl;
        }
    }

    // Iterate through each trajectory point and extract joint states in the
    // correct order as well as timestamps
    for (const auto &point : traj.points)
    {
        JointTrajPoint ordered_traj_point;
        ordered_traj_point.positions.conservativeResize(nof_joints);

        // Extract the joint positions in correct order and fill it in the
        // result
        for (size_t i = 0; i < joint_name_order.size(); ++i)
        {
            ordered_traj_point.positions[i] = point.positions[index_order[i]];
        }

        // Retrieve timestamp as well and push it into the result vector
        ordered_traj_point.timestamp = point.time_from_start.toSec();
        ordered_traj.push_back(ordered_traj_point);
    }

    return ordered_traj;
}

JointTrajPoint get_ordered_joint_states(const sensor_msgs::JointState::ConstPtr &joint_states,
                                        const std::vector<std::string> &joint_name_order)
{

    const size_t nof_joints = joint_name_order.size(); // This is the relevant number of joints. Even if the
                                                       // trajectory contains more joints, we'll only extract
                                                       // relevant joint positions and in order
    JointTrajPoint ordered_joint_states;
    ordered_joint_states.positions.conservativeResize(nof_joints);

    // First, we determine the order that the indices need to be in to match
    // the joint_name_order.
    //  Store unordered joint names in a map and assign indices
    // Create a mapping between joint names and indices
    std::unordered_map<std::string, size_t> joint_index_map;
    for (size_t i = 0; i < joint_states->name.size(); ++i)
    {
        joint_index_map[joint_states->name[i]] = i;
    }

    // Determine the index order
    std::vector<size_t> index_order(nof_joints);
    for (size_t i = 0; i < nof_joints; ++i)
    {
        const std::string &joint_name = joint_name_order[i];
        auto it = joint_index_map.find(joint_name);
        if (it != joint_index_map.end())
        {
            index_order[i] = it->second;
        }
        else
        {
            // Handle the case where the joint name is not found
            std::cerr << "Joint name not found: " << joint_name << std::endl;
        }
    }

    // Extract the joint positions in correct order
    for (size_t i = 0; i < nof_joints; ++i)
    {
        ordered_joint_states.positions[i] = joint_states->position[index_order[i]];
    }

    // Extract and set the timestamp as well
    ordered_joint_states.timestamp = joint_states->header.stamp.toSec();

    return ordered_joint_states;
}

Eigen::Isometry3d get_htm(const std::string &space_frame, const std::string &body_frame, tf2_ros::Buffer &tf_buffer)
{

    tf2_ros::TransformListener tf_listener(tf_buffer); // Initialize listener with the buffer

    Eigen::Isometry3d htm; // Output

    geometry_msgs::TransformStamped transform_stamped; // ROS message to hold transform info

    // Query the listener for the desired transformation at the latest
    // available time

    try
    {
        transform_stamped = tf_buffer.lookupTransform(space_frame, body_frame, ros::Time(0), ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    // Convert the message to Eigen::Isometry3d type
    // Extract translation and rotation from the TransformStamped message
    Eigen::Vector3d translation(transform_stamped.transform.translation.x, transform_stamped.transform.translation.y,
                                transform_stamped.transform.translation.z);
    Eigen::Quaterniond rotation(transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x,
                                transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z);

    // Set the translation and rotation components of the HTM
    htm.translation() = translation;
    htm.linear() = rotation.toRotationMatrix();

    return htm;
}

control_msgs::FollowJointTrajectoryGoal follow_joint_trajectory_msg_builder(
    const std::vector<Eigen::VectorXd> &bare_trajectory, const Eigen::VectorXd &config_offset,
    const std::vector<std::string> &joint_names, const double &time_step)
{
    control_msgs::FollowJointTrajectoryGoal fjtg_msg;

    // Set the joint names
    fjtg_msg.trajectory.joint_names = joint_names;

    fjtg_msg.trajectory.points.resize(bare_trajectory.size() +
                                      1); // Resize points array before filling to bare_trajectory.size()+1
                                          // since the first point will be the config_offset itself

    // Add the first point with config_offset
    trajectory_msgs::JointTrajectoryPoint &first_point = fjtg_msg.trajectory.points[0];
    first_point.time_from_start = ros::Duration(0.0); // Start time is 0
    first_point.positions.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); i++)
    {
        first_point.positions[i] = config_offset[i];
    }

    // Fill out the rest of the trajectory with bare_trajectory
    size_t j = 1; // Start with 1 to skip the first point
    for (const Eigen::VectorXd &point : bare_trajectory)
    {
        trajectory_msgs::JointTrajectoryPoint &trajectory_point = fjtg_msg.trajectory.points[j];

        // Fill out point times
        trajectory_point.time_from_start = ros::Duration(j * time_step);

        // Fill out position values
        trajectory_point.positions.resize(joint_names.size()); // resize positions array before filling
        for (size_t i = 0; i < joint_names.size(); i++)
        {
            trajectory_point.positions[i] = point[i] + config_offset[i];
        }

        j++;
    }

    return fjtg_msg;
}
} // namespace AffordanceUtilROS
