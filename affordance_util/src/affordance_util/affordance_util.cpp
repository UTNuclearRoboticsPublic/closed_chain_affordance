#include <affordance_util/affordance_util.hpp>

namespace YAML
{
template <int N> bool convert<Eigen::Matrix<double, N, 1>>::decode(const Node &node, Eigen::Matrix<double, N, 1> &vec)
{
    if (!node.IsSequence() || node.size() != N)
    {
        return false; // Invalid YAML node for Eigen::VectorXd
    }

    // Fill out the elements of the Eigen::Vector one-by-one
    for (std::size_t i = 0; i < N; ++i)
    {
        vec(i) = node[i].as<double>();
    }

    return true; // Successfully decoded Eigen::VectorXd
}

template <int N> Node convert<Eigen::Matrix<double, N, 1>>::encode(const Eigen::Matrix<double, N, 1> &vec)
{
    Node node;
    if (vec.size() != N)
    {
        // Handle the case where the vector size doesn't match the expected size
        throw std::invalid_argument("Invalid vector size for encoding.");
    }

    for (int i = 0; i < N; ++i)
    {
        node.push_back(vec(i));
    }
    return node;
}
} // namespace YAML

namespace affordance_util
{

std::vector<double> compute_gripper_joint_trajectory(const GripperGoalType &gripper_goal_type,
                                                     const double &gripper_start_state, const double &gripper_end_state,
                                                     const int &trajectory_density)
{
    std::vector<double> trajectory(trajectory_density); // Preallocate vector with the correct size

    if (gripper_goal_type == GripperGoalType::CONSTANT)
    {
        // Fill the vector with the gripper_end_state
        std::fill(trajectory.begin(), trajectory.end(), gripper_end_state);
    }
    else // gripper_goal_type == GripperGoalType::CONTINUOUS
    {
        const double step = (gripper_end_state - gripper_start_state) / (trajectory_density - 1);

        for (int i = 0; i < trajectory_density; ++i)
        {
            trajectory[i] = gripper_start_state + i * step;
        }
    }

    return trajectory;
}

CcModel compose_cc_model_slist(const RobotDescription &robot_description, const ScrewInfo &aff_info,
                               const Eigen::MatrixXd &approach_end_pose, const VirtualScrewOrder &vir_screw_order)
{
    CcModel cc_model; // Output of the function

    // Compute robot Jacobian
    Eigen::MatrixXd robot_jacobian = JacobianSpace(robot_description.slist, robot_description.joint_states);

    // Compute approach screw
    const Eigen::Matrix4d approach_start_pose =
        FKinSpace(robot_description.M, robot_description.slist, robot_description.joint_states);
    const Eigen::Matrix<double, 6, 1> approach_twist =
        affordance_util::Adjoint(approach_start_pose) *
        affordance_util::se3ToVec(
            affordance_util::MatrixLog6(affordance_util::TransInv(approach_start_pose) * approach_end_pose));
    const Eigen::Matrix<double, 6, 1> approach_screw = approach_twist / approach_twist.norm();

    // Fill out the approach limit
    cc_model.approach_limit = approach_twist.norm();

    // In case aff screw is not set, compute it
    ScrewInfo aff = aff_info;
    if (aff.screw.hasNaN())
    {
        aff.screw = affordance_util::get_screw(aff);
    }

    // If pure rotation, the motion of the last closed-chain joint is in the opposite direction of the affordance
    // since the ground link is fixed and it is the affordance link that moves instead
    Eigen::VectorXd aff_screw(6);

    if (aff.type == ScrewType::ROTATION)
    {
        aff_screw = -aff.screw;
    }
    else
    {

        aff_screw = aff.screw;
    }

    if (vir_screw_order == VirtualScrewOrder::NONE)
    {
        const size_t nof_sjoints = 2; // approach screw and affordance
        cc_model.slist.conservativeResize(robot_description.slist.rows(),
                                          (robot_description.slist.cols() + nof_sjoints));
        cc_model.slist << robot_jacobian, approach_screw, aff_screw;
    }
    else
    {
        const size_t screw_length = 6;      // Length of the screw vector
        const size_t screw_axis_length = 3; // Length of the screw axis
        const size_t nof_vir_ee_joints = 3; // Number of virtual ee joints
        const size_t nof_sjoints =
            nof_vir_ee_joints + 2; // Number of joints to be appended, + 2 for approach and affordance screw

        // Append virtual EE screw axes as well as the affordance screw.
        // Note: In the future it might be desired to append the virtual EE screws as Jacobians as well. This would
        // allow one to track the orientation of the gripper as the magnitudes (joint angles) of these screws.
        // Currently, we model the Virtual EE screws as aligned with the space frame at the start pose of the
        // affordance. An advantage of this is physical intuition in controlling the gripper.
        Eigen::MatrixXd vir_slist(screw_length, nof_vir_ee_joints);

        // Extract robot palm location
        const Eigen::Vector3d q_vir = approach_start_pose.block<3, 1>(0, 3); // Translation part of the HTM

        // Virtual EE screw axes
        Eigen::Matrix<double, screw_axis_length, nof_vir_ee_joints> w_vir; // Virtual EE screw axes

        // Assign virtual EE screw axes in requested order
        if (vir_screw_order == VirtualScrewOrder::YZX)
        {
            w_vir.col(0) << 0, 1, 0; // y
            w_vir.col(1) << 0, 0, 1; // z
            w_vir.col(2) << 1, 0, 0; // x
        }
        else if (vir_screw_order == VirtualScrewOrder::ZXY)
        {
            w_vir.col(0) << 0, 0, 1; // z
            w_vir.col(1) << 1, 0, 0; // x
            w_vir.col(2) << 0, 1, 0; // y
        }
        else // vir_screw_order == VirtualScrewOrder::XYZ
        {
            w_vir.col(0) << 1, 0, 0; // x
            w_vir.col(1) << 0, 1, 0; // y
            w_vir.col(2) << 0, 0, 1; // z
        }

        // Compute virtual EE screws
        for (size_t i = 0; i < nof_vir_ee_joints; ++i)
        {
            vir_slist.col(i) = get_screw(w_vir.col(i), q_vir);
        }

        // Altogether
        cc_model.slist.conservativeResize(robot_description.slist.rows(),
                                          (robot_description.slist.cols() + nof_sjoints));
        cc_model.slist << robot_jacobian, vir_slist, approach_screw, aff_screw;
    }

    return cc_model;
}
Eigen::MatrixXd compose_cc_model_slist(const RobotDescription &robot_description, const ScrewInfo &aff_info,
                                       const VirtualScrewOrder &vir_screw_order)
{
    // Compute robot Jacobian
    Eigen::MatrixXd robot_jacobian = JacobianSpace(robot_description.slist, robot_description.joint_states);
    Eigen::MatrixXd slist;

    // In case aff screw is not set, compute it
    ScrewInfo aff = aff_info;
    if (aff.screw.hasNaN())
    {
        aff.screw = affordance_util::get_screw(aff);
    }

    // If pure rotation, the motion of the last closed-chain joint is in the opposite direction of the affordance
    // since the ground link is fixed and it is the affordance link that moves instead
    Eigen::VectorXd aff_screw(6);

    if (aff.type == ScrewType::ROTATION)
    {
        aff_screw = -aff.screw;
    }
    else
    {

        aff_screw = aff.screw;
    }

    if (vir_screw_order == VirtualScrewOrder::NONE)
    {
        const size_t nof_sjoints = 1; // 1 for one affordance
        slist.conservativeResize(robot_description.slist.rows(), (robot_description.slist.cols() + nof_sjoints));
        slist << robot_jacobian, aff_screw;
    }
    else
    {
        const size_t screw_length = 6;                    // Length of the screw vector
        const size_t screw_axis_length = 3;               // Length of the screw axis
        const size_t nof_vir_ee_joints = 3;               // Number of virtual ee joints
        const size_t nof_sjoints = nof_vir_ee_joints + 1; // Number of joints to be appended, + 1 for one affordance

        // Append virtual EE screw axes as well as the affordance screw.
        // Note: In the future it might be desired to append the virtual EE screws as Jacobians as well. This would
        // allow one to track the orientation of the gripper as the magnitudes (joint angles) of these screws.
        // Currently, we model the Virtual EE screws as aligned with the space frame at the start pose of the
        // affordance. An advantage of this is physical intuition in controlling the gripper.
        Eigen::MatrixXd vir_slist(screw_length, nof_vir_ee_joints);

        // Extract robot palm location
        const Eigen::Matrix4d ee_htm =
            FKinSpace(robot_description.M, robot_description.slist, robot_description.joint_states);
        const Eigen::Vector3d q_vir = ee_htm.block<3, 1>(0, 3); // Translation part of the HTM

        // Virtual EE screw axes
        Eigen::Matrix<double, screw_axis_length, nof_vir_ee_joints> w_vir; // Virtual EE screw axes

        // Assign virtual EE screw axes in requested order
        if (vir_screw_order == VirtualScrewOrder::YZX)
        {
            w_vir.col(0) << 0, 1, 0; // y
            w_vir.col(1) << 0, 0, 1; // z
            w_vir.col(2) << 1, 0, 0; // x
        }
        else if (vir_screw_order == VirtualScrewOrder::ZXY)
        {
            w_vir.col(0) << 0, 0, 1; // z
            w_vir.col(1) << 1, 0, 0; // x
            w_vir.col(2) << 0, 1, 0; // y
        }
        else // vir_screw_order == VirtualScrewOrder::XYZ
        {
            w_vir.col(0) << 1, 0, 0; // x
            w_vir.col(1) << 0, 1, 0; // y
            w_vir.col(2) << 0, 0, 1; // z
        }

        // Compute virtual EE screws
        for (size_t i = 0; i < nof_vir_ee_joints; ++i)
        {
            vir_slist.col(i) = get_screw(w_vir.col(i), q_vir);
        }

        // Altogether
        slist.conservativeResize(robot_description.slist.rows(), (robot_description.slist.cols() + nof_sjoints));
        slist << robot_jacobian, vir_slist, aff_screw;
    }

    return slist;
}
Eigen::Matrix4d urdfPoseToMatrix(const urdf::Pose &pose)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rotation_matrix;

    // Convert URDF rotation (quaternion) to roll, pitch, yaw
    double roll, pitch, yaw;
    pose.rotation.getRPY(roll, pitch, yaw);

    // Create rotation matrix from RPY values
    rotation_matrix =
        (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
            .matrix();

    // Set the rotation part of the matrix
    transform.block<3, 3>(0, 0) = rotation_matrix;

    // Set the translation part
    transform(0, 3) = pose.position.x;
    transform(1, 3) = pose.position.y;
    transform(2, 3) = pose.position.z;

    return transform;
}

// Function to compute the transform from the reference frame to a link
Eigen::Matrix4d computeTransformFromReferenceToLink(const urdf::ModelInterfaceSharedPtr &robot_model, const std::string &link_name,
                                                    const std::string &reference_frame)
{
    if (link_name == reference_frame)
    {
        return Eigen::Matrix4d::Identity();
    }

    urdf::LinkConstSharedPtr link = robot_model->getLink(link_name);
    if (!link)
    {
        throw std::runtime_error("Link not found: " + link_name);
    }

    if (!link->parent_joint)
    {
        throw std::runtime_error("Reached root link without finding reference frame");
    }

    const urdf::JointConstSharedPtr joint = link->parent_joint;
    const std::string &parent_link_name = joint->parent_link_name;

    // Compute the transform from the reference frame to the parent link
    Eigen::Matrix4d ref_to_parent_link_transform =
        computeTransformFromReferenceToLink(robot_model, parent_link_name, reference_frame);

    // Get the parent_to_joint_origin_transform
    urdf::Pose parent_to_joint = joint->parent_to_joint_origin_transform;
    Eigen::Matrix4d joint_transform = urdfPoseToMatrix(parent_to_joint);

    // The transform from the reference frame to this link
    Eigen::Matrix4d ref_to_link_transform = ref_to_parent_link_transform * joint_transform;

    return ref_to_link_transform;
}
// Function to compute the transform from the reference frame to a joint
Eigen::Matrix4d computeTransformFromReferenceToJoint(const urdf::ModelInterfaceSharedPtr &robot_model, const std::string &joint_name,
                                                     const std::string &reference_frame)
{
    const urdf::JointConstSharedPtr joint = robot_model->getJoint(joint_name);
    if (!joint)
    {
        throw std::runtime_error("Joint not found: " + joint_name);
    }

    // Get the parent link name
    const std::string &parent_link_name = joint->parent_link_name;

    // Compute the transform from the reference frame to the parent link
    Eigen::Matrix4d ref_to_parent_link_transform =
        computeTransformFromReferenceToLink(robot_model, parent_link_name, reference_frame);

    // Get the parent_to_joint_origin_transform
    urdf::Pose parent_to_joint = joint->parent_to_joint_origin_transform;
    Eigen::Matrix4d joint_transform = urdfPoseToMatrix(parent_to_joint);

    // The transform from the reference frame to the joint
    Eigen::Matrix4d ref_to_joint_transform = ref_to_parent_link_transform * joint_transform;

    return ref_to_joint_transform;
}
RobotConfig robot_builder(const std::string &config_file_path)
{

    RobotConfig robotConfig; // Output of the function

    // Load the YAML file
    const YAML::Node config = YAML::LoadFile(config_file_path);
    if (!config)
    {
        throw std::runtime_error("Robot screw list cannot be built without a valid robot config yaml file");
    }

    // Access the reference frame info
    const YAML::Node &refFrameNode = config["ref_frame"];
    const std::string &ref_frame_name = refFrameNode[0]["name"].as<std::string>(); // access with [0] since only
                                                                                   // one reference frame

    // Access the 'robot_joints' array
    const YAML::Node &robotJointsNode = config["robot_joints"];

    // Parse each joint
    std::vector<JointData> jointsData;
    for (const YAML::Node &jointNode : robotJointsNode)
    {
        JointData joint;
        joint.name = jointNode["name"].as<std::string>();
        joint.screw_info.axis = jointNode["w"].as<Eigen::Vector3d>();
        joint.screw_info.location = jointNode["q"].as<Eigen::Vector3d>();
        jointsData.push_back(joint);
    }

    // Access gripper joint info
    const YAML::Node &gripperJointNode = config["gripper_joint"];
    const std::string &gripper_joint_name =
        gripperJointNode[0]["name"]
            .as<std::string>(); // access with [0] since only one joint. In the future, we may wanna add more joints

    // Access the tool info
    const YAML::Node &toolNode = config["tool"];
    const std::string &tool_name = toolNode[0]["name"].as<std::string>(); // access with [0] since only one tool

    // Compute screw axes
    const size_t screwSize = 6;
    const size_t &totalNofJoints = jointsData.size();
    Eigen::MatrixXd Slist(screwSize, totalNofJoints);

    for (size_t i = 0; i < totalNofJoints; i++)
    {
        const JointData &joint = jointsData[i];
        Slist.col(i) << get_screw(joint.screw_info.axis, joint.screw_info.location);
        /* Start setting the output of the function */
        // Joint names
        robotConfig.joint_names.push_back(joint.name);
    }

    /* Fill out the remaining members of the output and return it*/
    // Screw list
    robotConfig.Slist = Slist;

    // EE homogenous transformation matrix
    const Eigen::Vector3d toolLocation = toolNode[0]["q"].as<Eigen::Vector3d>();
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M.block<3, 1>(0, 3) = toolLocation;
    robotConfig.M = M;

    // Reference frame name
    robotConfig.ref_frame_name = ref_frame_name;

    // Tool name
    robotConfig.gripper_joint_name = gripper_joint_name;

    // Tool name
    robotConfig.tool_name = tool_name;

    return robotConfig;
}
RobotConfig robot_builder(const std::string &urdf_file_path, const std::string &ref_frame_name, const std::string &base_joint_name, const std::string &ee_frame_name, const Eigen::Vector3d &tool_location)
{
    RobotConfig robot_config; // Output of the function

    const urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(urdf_file_path);
    if (!model)
    {
        throw std::runtime_error("Robot screw list cannot be built without a valid robot config URDF file");
    }
    if (!model->getJoint(base_joint_name))
    {
        throw std::runtime_error("Robot URDF does not contain specified base joint");
    }
    if (!model->getLink(ref_frame_name))
    {
        throw std::runtime_error("Robot URDF does not contain specified reference frame");
    }
    if (!model->getLink(ee_frame_name))
    {
        throw std::runtime_error("Robot URDF does not contain specified ee frame");
    }

    // Get vector of joints between ee_frame and base_joint
    std::vector<urdf::JointConstSharedPtr> chain_list;   
    const urdf::LinkConstSharedPtr root = model->getRoot();
    urdf::JointConstSharedPtr current_joint = model->getLink(ee_frame_name)->parent_joint;
    
    // if(current_joint->type != urdf::Joint::FIXED)
    // {
    //     chain_list.push_back(current_joint);
    // }
    while(current_joint->name != base_joint_name)
    {   
        std::string parent_name = current_joint->parent_link_name;
        urdf::LinkConstSharedPtr parent_link = model->getLink(parent_name);
        if(parent_link == root)
        {
            throw std::runtime_error("Base joint not found on path from end effector frame");
        }
        current_joint = parent_link->parent_joint;
        // if(current_joint->type != urdf::Joint::FIXED || current_joint->name == ee_frame_name)
        // {
        //     chain_list.insert(chain_list.begin(), model->getJoint(current_joint->name));
        // }
        chain_list.insert(chain_list.begin(), model->getJoint(current_joint->name));
    }

    // Sets transforms for ref frame and joint pose
    std::vector<JointData> joints_data;
    const Eigen::Matrix4d ref_frame_transform = computeTransformFromReferenceToJoint(model, base_joint_name, ref_frame_name);
    Eigen::Matrix4d joint_pose_in_ref_frame = ref_frame_transform;

    for (const auto& joint_node: chain_list)
    {
        // Get the parent_to_joint_origin_transform
        const urdf::Pose parent_to_joint = joint_node->parent_to_joint_origin_transform;
        Eigen::Matrix4d joint_transform = urdfPoseToMatrix(parent_to_joint);

        if ((joint_node->type != urdf::Joint::FIXED))
        {
            // Fill out joint info
            JointData joint;
            if(joint_node->type == urdf::Joint::REVOLUTE || joint_node->type == urdf::Joint::CONTINUOUS)
            {
                joint.screw_info.type = affordance_util::ROTATION;
            }
            else if(joint_node->type == urdf::Joint::PRISMATIC)
            {
                joint.screw_info.type = affordance_util::TRANSLATION;
            }
            else
            {   
                throw std::runtime_error("Kinematic chain contains a joint type not accounted for.");
            }
            joint.name = joint_node->name;
            joint.limits.lower = joint_node->limits->lower;
            joint.limits.upper = joint_node->limits->upper;

            if(joint_node->name != base_joint_name)
            {
                // The transform from the reference frame to the joint
                joint_pose_in_ref_frame = joint_pose_in_ref_frame * joint_transform;
                
                Eigen::Vector3d joint_position = joint_pose_in_ref_frame.block<3, 1>(0, 3);
                joint.screw_info.location = joint_position;

                // Compute joint axis in reference frame
                Eigen::Vector3d joint_axis(joint_node->axis.x, joint_node->axis.y, joint_node->axis.z);
                Eigen::Vector3d world_joint_axis = joint_pose_in_ref_frame.block<3, 3>(0, 0) * joint_axis;
                joint.screw_info.axis = world_joint_axis;

                joint.screw_info.screw << affordance_util::get_screw(joint.screw_info);
            } 
            else
            {
                // Compute joint axis in reference frame
                Eigen::Vector3d joint_axis(joint_node->axis.x, joint_node->axis.y, joint_node->axis.z);
                // Compute joint position in reference frame
                Eigen::Vector3d joint_position = joint_pose_in_ref_frame.block<3, 1>(0, 3);
                joint.screw_info.location = joint_position;
                Eigen::Vector3d world_joint_axis = joint_pose_in_ref_frame.block<3, 3>(0, 0) * joint_axis;
                joint.screw_info.axis = world_joint_axis;
            }
            joints_data.push_back(joint);
        }
    }

    // Access the tool info
    const std::string &tool_name = ee_frame_name;

    // Compute screw axes
    const size_t screw_size = 6;
    const size_t &total_no_of_joints = joints_data.size();
    Eigen::MatrixXd s_list(screw_size, total_no_of_joints);

    for (size_t i = 0; i < total_no_of_joints; i++)
    {
        const JointData &joint = joints_data[i];
        s_list.col(i) << affordance_util::get_screw(joint.screw_info);
        robot_config.joint_names.push_back(joint.name);
    }

    /* Fill out the remaining members of the output and return it*/
    // Screw list
    robot_config.Slist = s_list;

    // EE homogenous transformation matrix

    Eigen::Matrix4d M;

    //TODO: Deduce orientation of the tool or end effector from the URDF
    if (tool_location.hasNaN())
    {
        // At this point, the last joint transform in the chain list must be the end effector joint
        M = joint_pose_in_ref_frame;
    }
    else
    {
            M = Eigen::Matrix4d::Identity();

        M.block<3, 1>(0, 3) = tool_location;
    }

    robot_config.M = M;

    // Reference frame name
    robot_config.ref_frame_name = ref_frame_name;

    // Tool name
    robot_config.tool_name = tool_name;

    return robot_config;
}
Eigen::MatrixXd Adjoint(const Eigen::Matrix4d &htm)
{
    Eigen::MatrixXd adjoint(6, 6); // Output

    // Extract the rotation matrix (3x3) and translation vector (3x1) from htm
    Eigen::Matrix3d rotationMatrix = htm.block<3, 3>(0, 0);
    Eigen::Vector3d translationVector = htm.block<3, 1>(0, 3);

    // Construct the bottom-left 3x3 part
    Eigen::Matrix3d botLeft = VecToso3(translationVector) * rotationMatrix;

    // Build the adjoint matrix
    adjoint << rotationMatrix, Eigen::Matrix3d::Zero(), botLeft, rotationMatrix;

    return adjoint;
}
Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd &Slist, const Eigen::VectorXd &thetalist)
{

    const int jacColSize = thetalist.size();
    Eigen::MatrixXd Js(6, jacColSize);
    Js.col(0) = Slist.col(0); // first column is simply the first screw axis
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // Compute the Jacobian using the POE formula and adjoint representation
    for (int i = 1; i < jacColSize; i++)
    {
        T *= MatrixExp6(VecTose3(Slist.col(i - 1) * thetalist(i - 1)));
        Js.col(i) = Adjoint(T) * Slist.col(i);
    }

    return Js;
}

Eigen::Matrix4d MatrixExp6(const Eigen::Matrix4d &se3mat)
{

    // Compute the 3-vector exponential coordinate form of the rotation matrix
    const Eigen::Matrix3d so3mat = se3mat.block<3, 3>(0, 0);
    const Eigen::Vector3d &omgtheta = so3ToVec(so3mat);

    // If the norm of the 3-vector exponential coordinate form is very small,
    // return the rotation part as identity and the translation part as is from
    // se3mat
    if (NearZero(omgtheta.norm()))
    {
        return (Eigen::Matrix4d() << Eigen::Matrix3d::Identity(), se3mat.block<3, 1>(0, 3), 0, 0, 0, 1).finished();
    }
    else
    {
        // Else compute the HTM using the Rodriguez formula
        const auto &[ignore, theta] = AxisAng3(omgtheta);
        const Eigen::Matrix3d omgmat = so3mat / theta;
        const Eigen::Matrix3d &R = MatrixExp3(so3mat);
        const Eigen::Vector3d p =
            (Eigen::Matrix3d::Identity() * theta + (1 - cos(theta)) * omgmat + (theta - sin(theta)) * omgmat * omgmat) *
            (se3mat.block<3, 1>(0, 3) / theta);
        return (Eigen::Matrix4d() << R, p, 0, 0, 0, 1).finished();
    }
}
Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d &so3mat)
{
    // Compute the 3-vector exponential coordinate form of the 3x3 skew-symmetric
    // matrix so3mat
    const Eigen::Vector3d &omgtheta = so3ToVec(so3mat);

    // If the norm of the 3-vector exponential coordinate form is very small,
    // return rotation matrix as identity
    if (NearZero(omgtheta.norm()))
    {
        return Eigen::Matrix3d::Identity();
    }
    else
    {
        // Else compute the rotation matrix using the Rodriguez formula
        const auto &[ignore, theta] = AxisAng3(omgtheta);
        const Eigen::Matrix3d omgmat = so3mat / theta;
        return Eigen::Matrix3d::Identity() + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
    }
}
Eigen::Vector3d so3ToVec(const Eigen::Matrix3d &so3mat)
{
    // Extract and return the vector from the skew-symmetric matrix so3mat
    return Eigen::Vector3d(so3mat(2, 1), so3mat(0, 2), so3mat(1, 0));
}

std::tuple<Eigen::Vector3d, double> AxisAng3(const Eigen::Vector3d &expc3)
{
    // Angle is simply the norm of the 3-vector exponential coordinates of
    // rotation
    const double theta = expc3.norm();

    // Axis is simply the 3-vector exponential coordinates of rotation normalized
    // by the angle
    const Eigen::Vector3d omghat = expc3 / theta;
    return std::make_tuple(omghat, theta);
}

Eigen::Matrix4d FKinSpace(const Eigen::Matrix4d &M, const Eigen::MatrixXd &Slist, const Eigen::VectorXd &thetalist)
{
    // Compute space-form forward kinematics using the product of exponential
    // formula
    Eigen::Matrix4d T = M;
    for (int i = thetalist.size() - 1; i >= 0; --i)
    {
        Eigen::Matrix4d expMat = MatrixExp6(VecTose3(Slist.col(i) * thetalist(i)));
        T = expMat * T;
    }
    return T;
}

Eigen::Matrix4d VecTose3(const Eigen::VectorXd &V)
{
    // Extract the rotation part of the vector, V and get it's skew-symmetric form
    const Eigen::Matrix3d omgmat = VecToso3(V.segment<3>(0));

    // Extract the translation part of the vector, V
    const Eigen::Vector3d v(V.segment<3>(3));

    // Start the 4x4 se3 matrix as an Identity. Then, fill out the rotation and
    // translation parts
    Eigen::Matrix4d se3mat = Eigen::Matrix4d::Zero();
    se3mat.block<3, 3>(0, 0) = omgmat;
    se3mat.block<3, 1>(0, 3) = v;

    return se3mat;
}

Eigen::Matrix3d VecToso3(const Eigen::Vector3d &omg)
{
    // Fill out the 3x3 so3 matrix as the skew-symmetric form of the passed
    // 3-vector, omg
    const Eigen::Matrix3d so3mat =
        (Eigen::Matrix3d() << 0, -omg(2), omg(1), omg(2), 0, -omg(0), -omg(1), omg(0), 0).finished();
    return so3mat;
}

Eigen::Matrix4d TransInv(const Eigen::Matrix4d &T)
{

    // Extract rotation matrix and translation vector
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d p = T.block<3, 1>(0, 3);

    // Calculate the inverse transformation matrix
    Eigen::Matrix4d invT;
    Eigen::Matrix3d invR = R.transpose();
    Eigen::Vector3d invP = -invR * p;
    invT << invR, invP, 0, 0, 0, 1;

    return invT;
}

Eigen::VectorXd se3ToVec(const Eigen::Matrix4d &se3mat)
{
    // Extract and construct the vector from the skew-symmetric matrix
    const Eigen::VectorXd V =
        (Eigen::VectorXd(6) << se3mat(2, 1), se3mat(0, 2), se3mat(1, 0), se3mat.block<3, 1>(0, 3)).finished();

    return V;
}

Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d &R)
{

    const double acosinput = (R.trace() - 1) / 2;
    Eigen::Matrix3d so3mat;

    if (acosinput >= 1)
    {
        so3mat.setZero();
    }
    else if (acosinput <= -1)
    {
        Eigen::Vector3d omg;
        if (!NearZero(1 + R(2, 2)))
        {
            omg = (1 / sqrt(2 * (1 + R(2, 2)))) * Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
        }
        else if (!NearZero(1 + R(1, 1)))
        {
            omg = (1 / sqrt(2 * (1 + R(1, 1)))) * Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
        }
        else
        {
            omg = (1 / sqrt(2 * (1 + R(0, 0)))) * Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
        }
        so3mat = VecToso3(M_PI * omg);
    }
    else
    {
        const double theta = acos(acosinput);
        so3mat = theta * (1 / (2 * sin(theta))) * (R - R.transpose());
    }

    return so3mat;
}

Eigen::Matrix4d MatrixLog6(const Eigen::Matrix4d &T)
{
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const Eigen::Vector3d p = T.block<3, 1>(0, 3);
    const Eigen::Matrix3d omgmat = MatrixLog3(R);

    Eigen::Matrix<double, 4, 4> expmat;

    if (omgmat.isApprox(Eigen::Matrix3d::Zero()))
    {
        expmat.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
        expmat.block<3, 1>(0, 3) = p;
        expmat.block<1, 4>(3, 0) = Eigen::Matrix<double, 1, 4>::Zero();
    }
    else
    {
        const double theta = acos((R.trace() - 1) / 2);
        const Eigen::Matrix3d eye3 = Eigen::Matrix3d::Identity();
        expmat.block<3, 3>(0, 0) = omgmat;
        expmat.block<3, 1>(0, 3) =
            (eye3 - 0.5 * omgmat + (1.0 / theta - 1.0 / (2 * tan(0.5 * theta))) * omgmat * omgmat / theta) * p;
        expmat.block<1, 4>(3, 0) = Eigen::Matrix<double, 1, 4>::Zero();
    }

    return expmat;
}

Eigen::Vector3d get_axis_from_screw(const ScrewInfo &si)
{

    Eigen::Vector3d axis;

    if (si.type == ScrewType::TRANSLATION)
    {
        axis = si.screw.tail(3);
    }
    else // (si.type == ScrewType::ROTATION) || (si.type == ScrewType::SCREW)
    {
        axis = si.screw.head(3);
    }
    return axis;
}

Eigen::Matrix<double, 6, 1> get_screw(const ScrewInfo &si)
{

    Eigen::VectorXd screw(6); // Output of the function

    if (si.type == ScrewType::TRANSLATION)
    {
        screw << Eigen::Vector3d::Zero(), si.axis;
    }
    else if (si.type == ScrewType::ROTATION)
    {
        screw.head(3) = si.axis;
        screw.tail(3) = si.location.cross(si.axis);
    }
    else // si.type == ScrewType::SCREW
    {
        screw.head(3) = si.axis;
        screw.tail(3) = si.location.cross(si.axis) + si.pitch * si.axis;
    }

    return screw;
}

Eigen::Matrix<double, 6, 1> get_screw(const Eigen::Vector3d &w, const Eigen::Vector3d &q)
{

    Eigen::VectorXd screw(6); // Output of the function

    screw.head(3) = w;
    screw.tail(3) = -w.cross(q); // q cross w

    return screw;
}

bool NearZero(const double &near)
{
    const double nearZeroTol_ = 1e-6;
    return std::abs(near) < nearZeroTol_;
}
} // namespace affordance_util
