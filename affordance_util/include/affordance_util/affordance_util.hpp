///////////////////////////////////////////////////////////////////////////////
//      Title     : affordance_util.hpp
//      Project   : affordance_util
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
/*
   Credits: Some functions are translated to Cpp from Kevin Lynch's Modern
   Robotics Matlab package
*/
#ifndef AFFORDANCE_UTIL
#define AFFORDANCE_UTIL

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream> // to print errors
#include <vector>
#include <yaml-cpp/yaml.h> // to read yaml files

namespace YAML
{
template <int N> struct convert<Eigen::Matrix<double, N, 1>>
{
    /**
     * @brief This template function is a custom addition to the YAML namespace
     * to decode a YAML node as an Eigen::VectorXd type. By default, the yaml-cpp
     * library does not support Eigen-type decoding.
     *
     * @param node YAML node
     * @param vec An Eigen::VectorXd type
     *
     * @return Boolean indicating whether decoding was successful
     */
    static bool decode(const Node &node, Eigen::Matrix<double, N, 1> &vec);
    /**
     * @brief This template function is a custom addition to the YAML namespace to
     * encode an Eigen::VectorXd type as a YAML node. By default, the yaml-cpp
     * library does not support Eigen-type encoding.
     *
     * @param vec An Eigen::VectorXd type
     *
     * @return YAML node
     */
    static Node encode(const Eigen::Matrix<double, N, 1> &vec);
};
} // namespace YAML

namespace affordance_util
{
/**
 * @brief Struct providing information about a screw
 */
struct ScrewInfo
{
    std::string type;           // Screw type. Possible fields are "translation", "rotation", "screw"
    Eigen::Vector3d axis;       // Screw axis
    Eigen::Vector3d location;   // Screw location from a frame of interest
    std::string location_frame; // Name of the screw frame, useful when looking up with apriltag
    double pitch;               // Pitch of the screw. Default is rotation
};
/**
 * @brief Struct to describe a joint with its name, axis, and location
 */
struct JointData
{
    std::string name;  // Joint name
    Eigen::Vector3d w; // Joint axis
    Eigen::Vector3d q; // Joint location
};

/**
 * @brief Struct describing a robotic arm
 */
struct RobotConfig
{
    Eigen::MatrixXd Slist;                // Space-form screw axes
    Eigen::Matrix4d M;                    // EE homogenous transformation matrix
    std::string ref_frame_name;           // Name of the reference frame
    std::vector<std::string> joint_names; // Name of the joints
    std::string tool_name;                // Name of the frame that mimicks where the robot
                                          // would grasp external objects
};

/**
 * @brief Given a robot screw list, robot palm HTM, robot joint states at affordance start pose, and affordance screw,
 * returns the closed-chain affordance model screw list.
 *
 * @param robot_slist Eigen::MatrixXd containing as columns 6x1 screws representing robot joints
 * @param thetalist Eigen::VectorXd containing robot joint states at affordance start pose
 * @param M Eigen::Matrix4d containing the HTM for the robot palm in home position
 * @param aff_screw Eigen::VectorXd containing 6x1 affordance screw
 *
 * @return Eigen::MatrixXd containing as columns all 6x1 screws encompassing the closed-chain affordance model
 */
Eigen::MatrixXd compose_cc_model_slist(const Eigen::MatrixXd &robot_slist, const Eigen::VectorXd &thetalist,
                                       const Eigen::Matrix4d &M, const Eigen::Matrix<double, 6, 1> &aff_screw);

/**
 * @brief Given a file path to a yaml file containing robot information,
 returns the robot space-form screw list, EE htm, space-frame name,
 joint_names, and tool name.
 * Below is an example of the information and formatting required in the YAML
 * file describing the robot. w is axis and q is location.
 * ref_frame:
 *   - name: arm0_base_link
 *
 * joints:
 *   - name: arm0_shoulder_yaw
 *     w: [0, 0, 1]
 *     q: [0, 0, 0]
 *
 *   - name: arm0_shoulder_pitch
 *     w: [0, 1, 0]
 *     q: [0, 0, 0]
 *
 *   - name: arm0_elbow_pitch
 *     w: [0, 1, 0]
 *     q: [0.3385, -0.0001, -0.0003]
 *
 *   - name: arm0_elbow_roll
 *     w: [1, 0, 0]
 *     q: [0.7428, -0.0003, 0.0693]
 *
 *   - name: arm0_wrist_pitch
 *     w: [0, 1, 0]
 *     q: [0.7428, -0.0003, 0.0693]
 *
 *   - name: arm0_wrist_roll
 *     w: [1, 0, 0]
 *     q: [0.7428, -0.0003, 0.0693]
 *
 * tool:
 *   - name: arm0_shoulder_yaw
 *     q: [0.9383, 0.0005, 0.0664]
 * @param config_file_path File path to the config file containing robot
 information
 *
 * @return Struct containing the robot space-form screw list, EE htm,
 space-frame name, joint_names, and tool name
 */
RobotConfig robot_builder(const std::string &config_file_path);
/**
 * @brief Given a homogenenous transformation matrix, computes its adjoint
 * representation
 *
 * @param htm Homogeneous transformation matrix
 *
 * @return
 */
Eigen::MatrixXd Adjoint(const Eigen::Matrix4d &htm);

/**
 * @brief Given a list of space-form screws and corresponding magnitudes,
 * computes the space-form Jacobian
 *
 * @param Slist Space-form screws as columns of a matrix
 * @param thetalist Screw magnitudes as a column vector
 *
 * @return Space-form Jacobian
 */
Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd &Slist, const Eigen::VectorXd &thetalist);
/**
 * @brief Given a 4x4 se(3) representation of exponential coordinates, returns
 * a T matrix in SE(3) that is achieved by traveling along/about the screw
 * axis S for a distance theta from an initial configuration T = I.
 *
 * @param se3mat 4x4 skew-symmetric representation of exponential coordinates
 *
 * @return 4x4 homogenous transformation matrix
 */
Eigen::Matrix4d MatrixExp6(const Eigen::Matrix4d &se3mat);

/**
 * @brief Given a 3x3 so(3) representation of exponential coordinates, returns
 * R in SO(3) that is achieved by rotating about omghat by theta from an
 * initial orientation R = I.
 *
 * @param so3mat 3x3 skew-symmetric representation of exponential coordinates
 *
 * @return 3x3 Rotation matrix
 */
Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d &so3mat);
/**
 * @brief Given a 3x3 skew-symmetric matrix (an element of so(3)), returns the
 * corresponding 3-vector (angular velocity).
 *
 * @param so3mat 3x3 skew-symmetric matrix
 *
 * @return 3-vector angular velocity
 */
Eigen::Vector3d so3ToVec(const Eigen::Matrix3d &so3mat);
/**
 * @brief Given a 3-vector of exponential coordinates for rotation, returns
 * the unit rotation axis omghat and the corresponding rotation angle theta.
 *
 * @param expc3 3-vector exponential coordinates for rotation
 *
 * @return Tuple of 3-vector axis and corresponding angle
 */
std::tuple<Eigen::Vector3d, double> AxisAng3(const Eigen::Vector3d &expc3);
/**
 * @brief Computes the space-form forward kinematics of the robot
 *
 * @param M Home configuration (position and orientation) of the
 * end-effector
 *
 * @param Slist Joint screw axes in the space frame when the manipulator
 * is at the home position
 *
 * @param thetalist List of joint coordinates
 *
 * @return T in SE(3) representing the end-effector frame, when the joints are
 * at the specified coordinates (i.t.o Space Frame)
 */
Eigen::Matrix4d FKinSpace(const Eigen::Matrix4d &M, const Eigen::MatrixXd &Slist, const Eigen::VectorXd &thetalist);
/**
 * @brief Given a 6-vector (representing a spatial velocity), returns the
 * corresponding 4x4 se(3) matrix.
 *
 * @param V 6-vector spatial velocity
 *
 * @return 4x4 se(3) matrix
 */
Eigen::Matrix4d VecTose3(const Eigen::VectorXd &V);
/**
 * @brief Given a 3-vector (angular velocity), returns the skew symmetric
 * matrix in so(3).
 *
 * @param omg 3-vector angular velocity
 *
 * @return skew-symmetric representation of the passed 3-vector
 */
Eigen::Matrix3d VecToso3(const Eigen::Vector3d &omg);

/**
 * @brief Given a transformation matrix, computes its inverse
 *
 * @param T transformation matrix
 *
 * @return inverse of the transformation matrix
 */
Eigen::Matrix4d TransInv(const Eigen::Matrix4d &T);

/**
 * @brief Given a se3mat a 4x4 se(3) matrix, returns the corresponding
 * 6-vector (representing spatial velocity).
 *
 * @param se3mat 4x4 se(3) matrix
 *
 * @return 6-vector respresenting spatial velocity
 */
Eigen::VectorXd se3ToVec(const Eigen::Matrix4d &se3mat);

/**
 * @brief Given R (rotation matrix), returns the corresponding so(3)
 * representation of exponential  coordinates.
 *
 * @param R rotation matrix
 *
 * @return so(3) representation of exponential coordinates
 */
Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d &R);

/**
 * @brief Given a transformation matrix T in SE(3), returns the corresponding
 * se(3) representation of exponential  coordinates.
 *
 * @param T transformation matrix
 *
 * @return exponential coordinate representation of the transformation matrix
 */
Eigen::Matrix4d MatrixLog6(const Eigen::Matrix4d &T);

/**
 * @brief Given a screw axis and its location, returns the 6x1 screw vector
 *
 * @param si affordance_util::ScrewInfo containing information about the screw. For translation, two fields are
 * necessary: si.type = "translation" and si.axis. For other cases, supply location and pitch as well.
 *
 * @return Eigen::Matrix<double, 6, 1> containing the 6x1 screw vector
 */
Eigen::Matrix<double, 6, 1> get_screw(const affordance_util::ScrewInfo &si);

/**
 * @brief For revolute screws. given a screw axis and location, returns the 6x1 screw vector
 *
 * @param w Eigen::Vector3d containing screw axis
 * @param q Eigen::Vector3d containing the location of the screw axis
 *
 * @return Eigen::Matrix<double, 6, 1> containing the 6x1 screw vector
 */
Eigen::Matrix<double, 6, 1> get_screw(const Eigen::Vector3d &w, const Eigen::Vector3d &q);

/**
 * @brief Given a scalar, checks if the scalar is small enough to be
 * neglected.
 *
 * @param near scalar value to be checked
 *
 * @return whether the scalar value is near zero
 */
bool NearZero(const double &near);

}; // namespace affordance_util

#endif // AFFORDANCE_UTIL
