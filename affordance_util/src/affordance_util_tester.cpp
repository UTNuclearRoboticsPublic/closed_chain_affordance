#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <iomanip> // for std::precision
#include <iostream>

int main()
{

    using namespace affordance_util;
    // Precision for testing
    std::cout << std::fixed << std::setprecision(4); // Display up to 4 decimal places

    std::cout << "Testing VecToSo3" << std::endl;
    const Eigen::Vector3d omg(1.0, 2.0, 3.0);
    std::cout << "Input: \n" << omg << std::endl;
    const Eigen::Matrix3d &so3mat = VecToso3(omg);
    std::cout << "Output: \n" << so3mat << std::endl;

    std::cout << "\nTesting so3ToVec" << std::endl;
    std::cout << "Input: \n" << so3mat << std::endl;
    std::cout << "Output: \n" << so3ToVec(so3mat) << std::endl;

    std::cout << "\nTesting VecTose3" << std::endl;
    const Eigen::VectorXd V = (Eigen::VectorXd(6) << 1, 2, 3, 4, 5, 6).finished();
    std::cout << "Input: \n" << V << std::endl;
    std::cout << "Output: \n" << VecTose3(V) << std::endl;

    std::cout << "\nTesting AxisAng3" << std::endl;
    const Eigen::Vector3d expc3 = (Eigen::Vector3d() << 1, 2, 3).finished();
    std::cout << "Input: \n" << expc3 << std::endl;
    const auto &[expc3Axis, expc3Angle] = AxisAng3(expc3);
    std::cout << "Output axis: \n" << expc3Axis << std::endl;
    std::cout << "Output angle: \n" << expc3Angle << std::endl;

    std::cout << "\nTesting MatrixExp3" << std::endl;
    std::cout << "Input: \n" << so3mat << std::endl;
    std::cout << "Output: \n" << MatrixExp3(so3mat) << std::endl;

    const Eigen::Matrix4d se3mat =
        (Eigen::Matrix4d() << 0, 0, 0, 0, 0, 0, -1.5708, 2.3562, 0, 1.5708, 0, 2.3562, 0, 0, 0, 0).finished();
    std::cout << "\nTesting MatrixExp6" << std::endl;
    std::cout << "Input: \n" << se3mat << std::endl;
    std::cout << "Output: \n" << MatrixExp6(se3mat) << std::endl;

    const Eigen::Matrix4d M = (Eigen::Matrix4d() << -1, 0, 0, 0, 0, 1, 0, 6, 0, 0, -1, 2, 0, 0, 0, 1).finished();

    Eigen::MatrixXd Slist(6, 3);
    Slist << 0, 0, 0, 0, 0, 0, 1, 0, -1, 4, 0, -6, 0, 1, 0, 0, 0, -0.1;

    Eigen::Vector3d thetalist = (Eigen::Vector3d() << M_PI / 2, 3, M_PI).finished();
    std::cout << "\nTesting FKinSpace" << std::endl;
    std::cout << "Input M: \n" << M << std::endl;
    std::cout << "Input Slist: \n" << Slist << std::endl;
    std::cout << "Input thetalist: \n" << thetalist << std::endl;
    std::cout << "Output: \n" << FKinSpace(M, Slist, thetalist) << std::endl;

    Eigen::MatrixXd Slist2(6, 4);

    Slist2 << 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 2, 0, 0.2, 0.2, 0, 2, 0.3, 0.2, 3, 1, 0.4;
    Eigen::VectorXd thetalist2(4);
    thetalist2 << 0.2, 1.1, 0.1, 1.2;
    std::cout << "\nTesting JacobianSpace" << std::endl;
    std::cout << "Input Slist: \n" << Slist2 << std::endl;
    std::cout << "Input thetalist: \n" << thetalist2 << std::endl;
    std::cout << "Output: \n" << JacobianSpace(Slist2, thetalist2) << std::endl;

    Eigen::Matrix4d htm;
    htm << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 3, 0, 0, 0, 1;
    std::cout << "\nTesting Adjoint" << std::endl;
    std::cout << "Input htm: \n" << htm << std::endl;
    std::cout << "Output: \n" << Adjoint(htm) << std::endl;

    std::cout << "\nTesting TransInv" << std::endl;
    std::cout << "Input htm: \n" << htm << std::endl;
    std::cout << "Output: \n" << TransInv(htm) << std::endl;

    Eigen::Matrix4d se3mat2;
    se3mat2 << 0, -3, 2, 4, 3, 0, -1, 5, -2, 1, 0, 6, 0, 0, 0, 0;
    std::cout << "\nTesting se3ToVec" << std::endl;
    std::cout << "Input se3mat: \n" << se3mat2 << std::endl;
    std::cout << "Output: \n" << se3ToVec(se3mat2) << std::endl;

    const Eigen::Matrix3d R = (Eigen::Matrix3d() << 0, 0, 1, 1, 0, 0, 0, 1, 0).finished();
    std::cout << "\nTesting MatrixLog3" << std::endl;
    std::cout << "Input R: \n" << R << std::endl;
    std::cout << "Output: \n" << MatrixLog3(R) << std::endl;

    const double near = -1e-7;
    std::cout << "\nTesting NearZero" << std::endl;
    std::cout << "Input near: \n" << near << std::endl;
    std::cout << "Output: \n" << NearZero(near) << std::endl;

    const Eigen::Matrix4d T = (Eigen::Matrix4d() << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 3, 0, 0, 0, 1).finished();
    std::cout << "\nTesting MatrixLog6" << std::endl;
    std::cout << "Input T: \n" << T << std::endl;
    std::cout << "Output: \n" << MatrixLog6(T) << std::endl;

    std::cout << "\nTesting robot_builder YAML version" << std::endl;
    std::string filepath = "/home/john/random_tests/temp_cca_ws/cca_kinova_gen3_7dof_description.yaml";
    RobotConfig robot_config;
    try
    {
        robot_config = robot_builder(filepath);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error building robot: " << e.what() << std::endl;
    }
    std::cout << "Input filepath: \n" << filepath << std::endl;
    const Eigen::MatrixXd yaml_slist = robot_config.Slist;
    std::cout << "Output Slist: \n" << yaml_slist << std::endl;
    std::cout << "Output M: \n" << robot_config.M << std::endl;
    std::cout << "Output ref_frame_name: \n" << robot_config.ref_frame_name << std::endl;
    std::cout << "Output tool_name: \n" << robot_config.tool_name << std::endl;
    std::cout << "Output joint_names: ";
    for (const std::string &joint_name : robot_config.joint_names)
    {
        std::cout << joint_name << ",";
    }
    std::cout << std::endl;
    filepath = "/home/john/temp_cca_ws/GEN3_URDF_V12.urdf";
    const std::string ref_frame = "base_link";
    const std::string base_joint = "Actuator1";
    const std::string ee_frame = "EndEffector_Link";
    // const std::string filepath = "/home/john/temp_cca_ws/spot_arm.urdf";
    // const std::string ref_frame = "spot_arm_base_link";
    // const std::string base_joint = "spot_arm_shoulder_yaw";
    // const std::string ee_frame = "spot_arm_fingers";
    try
    {
        robot_config = robot_builder(filepath, ref_frame, base_joint, ee_frame);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error building robot: " << e.what() << std::endl;
    }
    std::cout << "\nTesting robot_builder URDF version" << std::endl;
    std::cout << "Input filepath: \n" << filepath << std::endl;
    std::cout << "Input ref_frame: \n" << ref_frame << std::endl;
    std::cout << "Input base_joint: \n" << base_joint << std::endl;
    std::cout << "Input ee_frame: \n" << ee_frame << std::endl;
    const Eigen::MatrixXd urdf_slist = robot_config.Slist;
    std::cout << "Output Slist: \n" << urdf_slist << std::endl;
    std::cout << "Output M: \n" << robot_config.M << std::endl;
    std::cout << "Output ref_frame_name: \n" << robot_config.ref_frame_name << std::endl;
    std::cout << "Output tool_name: \n" << robot_config.tool_name << std::endl;
    std::cout << "Output joint_names: ";
    for (const std::string &joint_name : robot_config.joint_names)
    {
        std::cout << joint_name << ",";
    }
    std::cout << std::endl;
    const double tolerance = 1e-3;
    bool are_equal = yaml_slist.isApprox(urdf_slist, tolerance); // Just compare solution, excluding affordance
    if (are_equal)
    {
        std::cout << "The URDF and YAML Slist match within " << tolerance << std::endl;
    }
    else
    {

        std::cerr << "The URDF and YAML Slist do not match within" << tolerance << std::endl;
    }

    ScrewInfo si;
    si.type = affordance_util::ScrewType::TRANSLATION;
    si.axis << 1.0, 0.0, 0.0;
    std::cout << "\n Testing get_screw" << std::endl;
    std::cout << "\n Translation input axis: " << si.axis.transpose() << std::endl;
    std::cout << "\n Translation output: " << get_screw(si).transpose() << std::endl;
    si.screw = get_screw(si);
    std::cout << "\n Testing get_axis_from_screw" << std::endl;
    std::cout << "\n Axis output for above screw: " << get_axis_from_screw(si).transpose() << std::endl;
    si = ScrewInfo();
    si.type = affordance_util::ScrewType::SCREW;
    si.axis << 0.0, 1.0, 0.0;
    si.location << 1.0, 2.0, 3.0;
    si.pitch = 0.2;
    std::cout << "\n Testing get_screw" << std::endl;
    std::cout << "\n Screw motion input axis: " << si.axis.transpose() << std::endl;
    std::cout << "\n Screw motion axis location: " << si.location.transpose() << std::endl;
    std::cout << "\n Screw motion pitch: " << si.pitch << std::endl;
    std::cout << "\n Screw motion output: " << get_screw(si).transpose() << std::endl;
    si.screw = get_screw(si);
    std::cout << "\n Testing get_axis_from_screw" << std::endl;
    std::cout << "\n Axis output for above screw: " << get_axis_from_screw(si).transpose() << std::endl;
    si = ScrewInfo();
    si.type = affordance_util::ScrewType::ROTATION;
    si.axis << 0.0, 0.0, 1.0;
    si.location << 1.0, 2.0, 3.0;
    std::cout << "\n Testing get_screw" << std::endl;
    std::cout << "\n Rotation input axis: " << si.axis.transpose() << std::endl;
    std::cout << "\n Rotation axis location: " << si.location.transpose() << std::endl;
    std::cout << "\n Rotation output: " << get_screw(si).transpose() << std::endl;
    si.screw = get_screw(si);
    std::cout << "\n Testing get_axis_from_screw" << std::endl;
    std::cout << "\n Axis output for above screw: " << get_axis_from_screw(si).transpose() << std::endl;

    Eigen::Vector3d screw_axis(1.0, 0.0, 0.0);
    Eigen::Vector3d screw_location(1.0, 2.0, 3.0);
    std::cout << "\n Testing get_screw rotation overload" << std::endl;
    std::cout << "\n Rotation input axis: " << screw_axis.transpose() << std::endl;
    std::cout << "\n Rotation axis location: " << screw_location.transpose() << std::endl;
    std::cout << "\n Rotation output: " << get_screw(screw_axis, screw_location).transpose() << std::endl;

    double gripper_start_state = 0;
    double gripper_end_state = -0.5;
    int trajectory_density = 5;
    affordance_util::GripperGoalType gripper_goal_type = affordance_util::GripperGoalType::CONSTANT;
    std::cout << "\n Testing compute_gripper_joint_trajectory " << std::endl;
    std::cout << "\n Gripper start state: " << gripper_start_state << std::endl;
    std::cout << "\n Gripper end state: " << gripper_end_state << std::endl;
    std::cout << "\n Trajectory density: " << trajectory_density << std::endl;
    std::cout << "\n Gripper goal type: CONSTANT" << std::endl;
    std::vector<double> gripper_trajectory =
        compute_gripper_joint_trajectory(gripper_goal_type, gripper_start_state, gripper_end_state, trajectory_density);
    std::cout << "\nFunction output:" << std::endl;
    for (const double &point : gripper_trajectory)
    {
        std::cout << point << ",";
    }
    std::cout << std::endl;

    gripper_goal_type = affordance_util::GripperGoalType::CONTINUOUS;
    std::cout << "\n Testing compute_gripper_joint_trajectory " << std::endl;
    std::cout << "\n Gripper start state: " << gripper_start_state << std::endl;
    std::cout << "\n Gripper end state: " << gripper_end_state << std::endl;
    std::cout << "\n Trajectory density: " << trajectory_density << std::endl;
    std::cout << "\n Gripper goal type: CONTINUOUS" << std::endl;
    gripper_trajectory =
        compute_gripper_joint_trajectory(gripper_goal_type, gripper_start_state, gripper_end_state, trajectory_density);
    std::cout << "\nFunction output:" << std::endl;
    for (const double &point : gripper_trajectory)
    {
        std::cout << point << ",";
    }
    std::cout << std::endl;

    return 0;
}
