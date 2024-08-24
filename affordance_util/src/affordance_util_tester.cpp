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

    /* const std::string filepath = "/home/crasun/spot_ws/src/cc_affordance_planner/config/robot_setup.yaml"; */
    /* const RobotConfig robot_config = robot_builder(filepath); */
    /* std::cout << "\n Testing robot_builder" << std::endl; */
    /* std::cout << "Input filepath: \n" << filepath << std::endl; */
    /* std::cout << "Output Slist: \n" << robot_config.Slist << std::endl; */
    /* std::cout << "Output M: \n" << robot_config.M << std::endl; */
    /* std::cout << "Output ref_frame_name: \n" << robot_config.ref_frame_name << std::endl; */
    /* std::cout << "Output tool_name: \n" << robot_config.tool_name << std::endl; */
    /* std::cout << "Output joint_names: "; */
    /* for (const std::string &joint_name : robot_config.joint_names) */
    /* { */
    /*     std::cout << joint_name << ","; */
    /* } */
    /* std::cout << std::endl; */

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

    return 0;
}
