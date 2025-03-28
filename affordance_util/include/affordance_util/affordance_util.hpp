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

#include "urdf_model/model.h"
#include "urdf_parser/urdf_parser.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <vector>
#include <yaml-cpp/yaml.h>

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
 * @brief Enum indicating how the location of a screw is provided or is to be determined, either as forward kinematics
 * to the tool, or looking up a TF with a frame name
 */
enum ScrewLocationMethod
{
    PROVIDED,
    FROM_FK,
    FROM_FRAME_NAME
};
/**
 * @brief Enum specifying the type of gripper goal along a joint trajectory as constant or continuous
 */
enum GripperGoalType
{
    CONSTANT,
    CONTINUOUS
};

/**
 * @brief Enum specifying the three screw types
 */
enum ScrewType
{
    ROTATION,
    TRANSLATION,
    SCREW,
    UNSET
};

/**
 * @brief Enum specifying the order of axes for a virtual spherical joint
 */
enum VirtualScrewOrder
{
    XYZ,
    YZX,
    ZXY,
    XY,
    YZ,
    ZX,
    NONE
};

/**
 * @brief Struct designed to contain the description of a robot in terms of its screw list, end-effector homogenous
 * transformation matrix, and current joint states
 */
struct RobotDescription
{
    Eigen::MatrixXd slist;
    Eigen::Matrix4d M = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::VectorXd joint_states;
    double gripper_state = std::numeric_limits<double>::quiet_NaN();
};

/**
 * @brief Struct providing information about a closed-chain affordance model
 */
struct CcModel
{
    Eigen::MatrixXd slist; // List of closed-chain screws
    double approach_limit; // Limit of the approach screw
};

/**
 * @brief Struct providing information about a screw
 */
struct ScrewInfo
{
    ScrewType type = ScrewType::UNSET;                                                          // Screw type
    Eigen::Vector3d axis = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()); // Screw axis
    Eigen::Vector3d location =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()); // Screw location from a frame of interest
    Eigen::VectorXd screw =
        Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN()); // Screw vector
    std::string location_frame; // Name of the screw frame, useful when looking up with apriltag
    double pitch = std::numeric_limits<double>::quiet_NaN(); // Pitch of the screw. Default is rotation, i.e. 0
    ScrewLocationMethod location_method =
        ScrewLocationMethod::PROVIDED; // Default assumes screw location is provided as an Eigen::Vector3d above
};

/**
 * @brief Struct to describe a joint with its name, axis, and location
 */
struct JointData
{

    struct Limits
    {
        double lower;
        double upper;
    };
    std::string name;
    ScrewInfo screw_info;
    Limits limits;
};

/**
 * @brief Returns the screw axes matrix corresponding to a given virtual screw order.
 *
 * This function provides an `Eigen::MatrixXd` where each column represents a screw axis
 * associated with the specified `VirtualScrewOrder`. The map is constructed once and accessed by key to ensure
 * efficient access.
 *
 * @param order The `VirtualScrewOrder` key for which to retrieve the corresponding screw axes matrix.
 * @return const Eigen::MatrixXd& A constant reference to an `Eigen::MatrixXd` where each column represents an axis for
 * the specified screw order.
 */
inline const Eigen::MatrixXd &get_vir_screw_axes(VirtualScrewOrder order)
{
    static const std::map<VirtualScrewOrder, Eigen::MatrixXd> vir_screw_order_map = {
        {VirtualScrewOrder::XYZ, (Eigen::MatrixXd(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished()},
        {VirtualScrewOrder::YZX, (Eigen::MatrixXd(3, 3) << 0, 0, 1, 1, 0, 0, 0, 1, 0).finished()},
        {VirtualScrewOrder::ZXY, (Eigen::MatrixXd(3, 3) << 0, 1, 0, 0, 0, 1, 1, 0, 0).finished()},
        {VirtualScrewOrder::XY, (Eigen::MatrixXd(3, 2) << 1, 0, 0, 1, 0, 0).finished()},
        {VirtualScrewOrder::YZ, (Eigen::MatrixXd(3, 2) << 0, 1, 1, 0, 0, 0).finished()},
        {VirtualScrewOrder::ZX, (Eigen::MatrixXd(3, 2) << 0, 0, 1, 0, 1, 0).finished()}};

    return vir_screw_order_map.at(order);
}

/**
 * @brief Struct describing a robotic arm
 */
struct RobotConfig
{
    struct JointNames
    {
        std::vector<std::string> robot; // Name of the joints
        std::string gripper;            // Name of the gripper joint
    };

    struct FrameNames
    {
        std::string ref;  // Name of the reference frame
        std::string ee;   // Name of the EE frame
        std::string tool; // Name of the frame that mimicks where the robot
                          // would grasp external objects
    };

    Eigen::MatrixXd Slist;             // Space-form screw axes
    Eigen::Matrix4d M;                 // EE homogenous transformation matrix
    JointNames joint_names;            // Joint names
    FrameNames frame_names;            // Frame names
    Eigen::Vector3d ee_to_tool_offset; // Location of the tool from the EE
};

/**
 * @brief Given gripper goal type, start and end state, and trajectory density, returns the gripper joint trajectory
 *
 * @param gripper_goal_type affordance_util::GripperGoalType indicating gripper goal type, i.e. constant or continuous
 * along the trajectory
 * @param gripper_start_state double specifying gripper joint start state
 * @param gripper_end_state double specifying gripper joint end state
 * @param trajectory_density int specifying the number of points for the gripper joint trajectory
 *
 * @return std::vector<double> containing the gripper joint trajectory
 */
std::vector<double> compute_gripper_joint_trajectory(const GripperGoalType &gripper_goal_type,
                                                     const double &gripper_start_state, const double &gripper_end_state,
                                                     const int &trajectory_density);

/**
 * @brief Given a robot description, affordance information, order of virtual screws, and HTM representing end of
 * approach motion, returns the closed-chain affordance model including approach motion.
 *
 * @param robot_description affordance_util::RobotDescription containing the description of the robot as robot screws,
 * robot palm homogeneous transformation matrix, and current joint states
 * @param aff ScrewInfo containing information about affordance
 * @param approach_end_pose Eigen::MatrixXd representing the approach motion end pose HTM
 * @param vir_screw_order affordance_util::VirtualScrewOrder indicating the order for the virtual EE screws. Possible
 * values are "xyz", "yzx" "zxy", and "none". Default is "xyz"
 *
 * @return affordance_util::CcModel containing the closed-chain affordance screws and approach limit.
 */
CcModel compose_cc_model_slist(const RobotDescription &robot_description, const ScrewInfo &aff_info,
                               const Eigen::MatrixXd &approach_end_pose,
                               const VirtualScrewOrder &vir_screw_order = VirtualScrewOrder::XYZ);
/**
 * @brief Given a robot description, affordance information, and order of virtual screws, returns the closed-chain
 * affordance model screw list.
 *
 * @param robot_description affordance_util::RobotDescription containing the description of the robot as robot screws,
 * robot palm homogeneous transformation matrix, and current joint states
 * @param aff ScrewInfo containing information about affordance
 * @param vir_screw_order affordance_util::VirtualScrewOrder indicating the order for the virtual EE screws. Possible
 * values are "xyz", "yzx" "zxy", and "none". Default is "xyz"
 *
 * @return Eigen::MatrixXd containing as columns all 6x1 screws encompassing the closed-chain affordance model.
 */
Eigen::MatrixXd compose_cc_model_slist(const RobotDescription &robot_description, const ScrewInfo &aff_info,
                                       const VirtualScrewOrder &vir_screw_order = VirtualScrewOrder::XYZ);
/**
 * @brief Given a urdf-type pose converts it to a homogeneous transformation matrix of Eigen::Matrix4d type.
 *
 * @param pose URDF pose for conversion.
 *
 * @return Eigen::Matrix4d matrix representation of pose
 */
Eigen::Matrix4d convert_urdf_pose_to_matrix(const urdf::Pose &pose);
/**
 * @brief Given a urdf model, desired link name, and reference frame name, returns the homogeneous transformation matrix
 * representing the link pose in the reference frame.
 *
 * @param robot_model Model containing information about robot.
 *
 * @param link_name Goal link name for transform.
 *
 * @param reference_frame Reference frame for transform.
 *
 * @return Eigen::Matrix4d matrix representation of transform
 */
Eigen::Matrix4d compute_transform_from_reference_to_link(const urdf::ModelInterfaceSharedPtr &robot_model,
                                                         const std::string &link_name,
                                                         const std::string &reference_frame);
/**
 * @brief Given a urdf model, desired joint name, and reference frame name, returns the homogeneous transformation
 * matrix representing the joint pose in the reference frame.
 *
 * @param robot_model Model containing information about robot.
 *
 * @param joint_name Goal joint name for transform.
 *
 * @param reference_frame Reference frame for transform.
 *
 * @return Eigen::Matrix4d matrix representation of transform
 */
Eigen::Matrix4d compute_transform_from_reference_to_joint(const urdf::ModelInterfaceSharedPtr &robot_model,
                                                          const std::string &joint_name,
                                                          const std::string &reference_frame);
/**
 * @brief Given a file path to a yaml file containing robot information,
 returns the robot space-form screw list, EE htm, space-frame name,
 joint_names, and tool name.
 * Below is an example of the information and formatting required in the YAML
 * file describing the robot. w is axis and q is location. Robot is Spot arm.
 *ref_frame:
 *  - name: arm0_base_link
 *
 *robot_joints:
 *  - name: arm0_shoulder_yaw
 *    w: [0, 0, 1]
 *    q: [0, 0, 0]
 *
 *  - name: arm0_shoulder_pitch
 *    w: [0, 1, 0]
 *    q: [0, 0, 0]
 *
 *  - name: arm0_elbow_pitch
 *    w: [0, 1, 0]
 *    q: [0.3385, -0.0001, -0.0003]
 *
 *  - name: arm0_elbow_roll
 *    w: [1, 0, 0]
 *    q: [0.7428, -0.0003, 0.0693]
 *
 *  - name: arm0_wrist_pitch
 *    w: [0, 1, 0]
 *    q: [0.7428, -0.0003, 0.0693]
 *
 *  - name: arm0_wrist_roll
 *    w: [1, 0, 0]
 *    q: [0.7428, -0.0003, 0.0693]
 *
 *end_effector:
 *  - gripper_joint_name: arm0_fingers
 *    frame_name: arm0_fingers
 *    q: [0.86025, -0.0003, 0.08412] # EE location
 *
 *tool:
 *  - name: arm0_tool0 # This is usually at the center of the palm
 *    offset_from_ee_frame: [0.07805, 0.0008, -0.01772] # Tool location from EE
 * @param config_file_path File path to the config file containing robot
 information
 *
 * @return Struct containing the robot space-form screw list, EE htm,
 space-frame name, joint_names, and tool name
 */
RobotConfig robot_builder(const std::string &config_file_path);
/**
 * @brief Given a file path to a URDF file containing robot information, a ref frame, a base joint, and an ee_frame,
 * returns the robot space-form screw list, EE htm, space-frame name, joint_names, tool name, and optionally, tool
 * location. Defaults to using EE link as tool but it is encouraged to provide a tool location that represents the
 * center of the robot palm.
 * @param urdf_file_path File path to the config file containing robot
 * information
 *
 * @param ref_frame_name Name of desired reference frame
 *
 * @param base_joint_name Name of desired base joint
 *
 * @param ee_frame_name Name of desired end effector frame
 *
 * @return Struct containing the robot space-form screw list, EE htm,
 * space-frame name, joint_names, and tool name
 */
RobotConfig robot_builder(
    const std::string &urdf_file_path, const std::string &ref_frame_name, const std::string &base_joint_name,
    const std::string &ee_frame_name,
    const Eigen::Vector3d &tool_location = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
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
 * @brief Given affordance information as a 6x1 screw vector and its type, returns its 3-vector screw axis
 *
 * @param si affordance_util::ScrewInfo with the screw vector and type portion filled out
 *
 * @return 3-vector screw axis representing given affordance information
 */
Eigen::Vector3d get_axis_from_screw(const ScrewInfo &si);

/**
 * @brief Given a screw axis and its location, returns the 6x1 screw vector
 *
 * @param si affordance_util::ScrewInfo containing information about the screw. For translation, two fields are
 * necessary: si.type = "translation" and si.axis. For other cases, supply location and pitch as well.
 *
 * @return Eigen::Matrix<double, 6, 1> containing the 6x1 screw vector. Also fills out screw axis and returns the
 * ScrewInfo by reference if screw vector is specified but axis is not.
 */
Eigen::Matrix<double, 6, 1> get_screw(const ScrewInfo &si);

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
