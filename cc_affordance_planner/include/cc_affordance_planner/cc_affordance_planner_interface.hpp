///////////////////////////////////////////////////////////////////////////////
//      Title     : cc_affordance_planner_interface.hpp
//      Project   : cc_affordance_planner
//      Created   : Fall 2024
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

#ifndef CC_AFFORDANCE_PLANNER_INTERFACE
#define CC_AFFORDANCE_PLANNER_INTERFACE

#include "jthread/jthread.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <condition_variable>
#include <mutex>

namespace cc_affordance_planner
{

/**
 * @brief Enum describing various planning types that the closed-chain affordance planner offers
 */
enum class PlanningType
{
    APPROACH,
    AFFORDANCE,
    EE_ORIENTATION_ONLY,
    CARTESIAN_GOAL
};

/**
 * @brief Enum describing various motion types that the closed-chain affordance model offers
 */
enum class MotionType
{
    APPROACH,
    AFFORDANCE,
};

/**
 * @brief Struct describing the goals for the Closed-chain affordance planner in terms of affordance state, ee
 * orientation state, grasp_pose, and gripper state.
 */
struct Goal
{

    double affordance = std::numeric_limits<double>::quiet_NaN();
    Eigen::VectorXd ee_orientation;
    Eigen::Matrix4d grasp_pose;
    double gripper = std::numeric_limits<double>::quiet_NaN();
};

/**
 * @brief Struct describing a task for the Closed-Chain Affordance planner in terms of affordance info, goal state,
 * trajectory density, motion type, virtual screw order, grasp pose, and gripper goal type.
 */
struct TaskDescription
{
    affordance_util::ScrewInfo affordance_info;
    Goal goal;
    int trajectory_density = 10;
    MotionType motion_type = MotionType::AFFORDANCE;
    affordance_util::VirtualScrewOrder vir_screw_order = affordance_util::VirtualScrewOrder::XYZ;
    affordance_util::GripperGoalType gripper_goal_type = affordance_util::GripperGoalType::CONSTANT;
    Eigen::VectorXd force_correction;

    /**
     * @brief Given a planning type, constructs a cca task description with necessary parameters. This constructor is
     * especially useful for CARTESIAN_GOAL and EE_ORIENTATION_ONLY planning, which are special cases of APPROACH and
     * AFFORDANCE motions respectively.
     *
     * @param task_type cc_affordance_planner::PlanningType indicating what type of planning is intended.
     */
    explicit TaskDescription(const PlanningType &planningType);

    /**
     * @brief Default constructor for cca task description
     */
    TaskDescription() = default;
};

/**
 * @class CcAffordancePlannerInterface
 * @brief Interface class for the Closed-Chain (CC) Affordance Planner, providing a method to generate
 *        joint trajectories for robot affordance and approach motions.
 */
class CcAffordancePlannerInterface
{
  public:
    /// Typedefs for function pointers to to CcAffordancePlanner::generate_approach_motion_joint_trajectory or
    /// CcAffordancePlanner::generate_affordance_motion_joint_trajectory
    using Gsmt = PlannerResult (CcAffordancePlanner::*)(const Eigen::MatrixXd &, const Eigen::VectorXd &,
                                                        const size_t &, const int &);
    using Gsmt_st = PlannerResult (CcAffordancePlanner::*)(const Eigen::MatrixXd &, const Eigen::VectorXd &,
                                                           const size_t &, const int &,
                                                           std::stop_token); // Gsmt with additional stop_token argument

    CcAffordancePlannerInterface();
    /**
     * @brief Constructs the CcAffordancePlannerInterface object with the provided configuration.
     *
     * @param planner_config Struct containing the settings for the Cc Affordance planner:
     * - `accuracy`: Defines the threshold for the affordance goal.
     *   For instance, for a 5 rad goal with 10% accuracy, set this parameter to 0.1 to achieve an affordance goal of 5
     * ± 0.5.
     * - `ik_max_itr`: (Advanced) Specifies the maximum iterations for the closed-chain inverse kinematics solver.
     * Default is 200.
     * - `update_method`: (Advanced) Specifies the update method to use. Possible values are `INVERSE`, `TRANSPOSE`, and
     * `BEST`. Default is `BEST`.
     * - `closure_error_threshold_ang`: (Advanced) Specifies the angular error threshold for closed-chain closure.
     * Default is 1e-4.
     * - `closure_error_threshold_lin`: (Advanced) Specifies the linear error threshold for closed-chain closure.
     * Default is 1e-5.
     */
    explicit CcAffordancePlannerInterface(const PlannerConfig &planner_config);

    /**
     * @brief Generates a robot joint trajectory using the closed-chain affordance model for performing desired tasks
     * based on the provided robot and task descriptions.
     *
     * @param robot_description `affordance_util::RobotDescription` containing the description of the robot with fields:
     * - `slist`: Eigen::MatrixXd with 6x1 Screws as columns, representing the joints of the robot in order.
     * - `M`: Eigen::Matrix4d representing the end-effector's homogenous transformation matrix.
     * - `joint_states`: Eigen::VectorXd representing the current joint states.
     *
     * @param task_description `cc_affordance_planner::TaskDescription` describing the task with fields:
     * - `motion_type`: `cc_affordance_planner::MotionType` describing the motion type to consider. The options are
     *   cc_affordance_planner::APPROACH or cc_affordance_planner::AFFORDANCE. AFFORDANCE is default.
     * - `affordance_info`: `affordance_util::ScrewInfo` describing the affordance with type, and
     * 	 axis/location or screw as mandatory fields.
     * - `trajectory_density`: Specifies the density of the trajectory as the number of points.
     *   For example, an affordance goal of 0.5 rad could have 5 points, each with a step of 0.1 rad.
     * - `goal`: goal to achieve with planning
     *     `affordance`: affordance goal
     *     `ee_orientation`: EE orientation goal for 1 or more axes per vir_screw_order.
     *     `grasp_pose`: cartesian goal for the EE to achieve during APPROACH motion
     *     `gripper`: goal state for the gripper
     * - `vir_screw_order`: affordance_util::VirtualScrewOrder describing the order of the joints in the virtual
     *   spherical joint of the closed-chain model. This joint describes the orientation freedom of the gripper. Default
     *   value is affordance_util::VirtualScrewOrder::XYZ.
     *
     * @return `cc_affordance_planner::PlannerResult` containing the solved closed-chain joint trajectory and additional
     * planning information.
     * - `success`: bool indicates if the planner succeeded.
     * - `trajectory_description`: cc_affordance_planner::TrajectoryDescription describes whether the trajectory is
     *   FULL, PARTIAL, or UNSET.
     * - `joint_trajectory`: std::vector<Eigen::VectorXd> containing the solved joint trajectory.
     * - `planning_time`: std::chrono::microseconds indicating the planning time.
     * - `update_method`: Update method used (pseudoinverse, transpose, or best of the two in concurrent planning).
     * - `update_trail`: Trail of update methods used in concurrent planning.
     * - `includes_gripper_trajectory`: Indication of whether the solution contains gripper trajectory.
     */
    PlannerResult generate_joint_trajectory(const affordance_util::RobotDescription &robot_description,
                                            const TaskDescription &task_description);

  private:
    PlannerConfig planner_config_;                              ///< Configuration settings for the planner.
    CcAffordancePlannerInverse ccAffordancePlannerInverse_;     ///< Inverse planner object
    CcAffordancePlannerTranspose ccAffordancePlannerTranspose_; ///< Transpose planner object

    /**
     * @brief Generates a closed-chain differential joint trajectory to reach desired goals using specified motion
     * generation functions.
     *
     * @param generate_specified_motion_joint_trajectory Function pointer to
     * `CcAffordancePlanner::generate_approach_motion_joint_trajectory` or
     * `CcAffordancePlanner::generate_affordance_motion_joint_trajectory`.
     * @param generate_specified_motion_joint_trajectory_st Function pointer to the same methods overloaded with an
     * additional `std::stop_token` arg.
     * @param slist Eigen::MatrixXd with 6x1 Screws as columns, representing all joints of the closed-chain model
     * (robot joints, virtual EE joint, approach joint (if approach motion), and affordance joint).
     * @param secondary_joint_goals Eigen::VectorXd with secondary joint angle goals, including EE orientation and
     * affordance (affordance goal as the last element).
     * @param nof_secondary_joints Specifies the number of secondary joints:
     *   - For affordance motion:
     *     - 1: Affordance control only.
     *     - 2: Control of affordance along with EE orientation about one axis (x, y, or z whichever is first in the
     * 		vir_screw_order specified).
     *     - 3: Control of affordance along with EE orientation about two axes (the first two specified in
     * 		vir_screw_order).
     *     - 4: Affordance and full EE orientation control.
     *   - For approach motion:
     *     - Minimum 2: Controls approach motion in the context of the affordance.
     *     - 3: Adds gripper orientation control about the next axis as specified in vir_screw_order.
     *     - 4: Adds gripper orientation control about the next two axes as specified in vir_screw_order.
     *     - 5: Adds full EE orientation.
     * @param trajectory_density Specifies the density for the trajectory in terms of number of points.
     *
     * @return `cc_affordance_planner::PlannerResult` containing the solved differential closed-chain joint trajectory
     * and additional planning information.
     */
    PlannerResult generate_specified_motion_joint_trajectory_(
        const Gsmt &generate_specified_motion_joint_trajectory,
        const Gsmt_st &generate_specified_motion_joint_trajectory_st, const Eigen::MatrixXd &slist,
        const Eigen::VectorXd &secondary_joint_goals, const size_t &nof_secondary_joints,
        const int &trajectory_density);

    /**
     * @brief Converts a differential joint trajectory into an absolute joint trajectory by referencing a starting joint
     * state. If optionally a gripper joint trajectory is provided, inserts it between the robot and secondary joint
     * states.
     *
     * @param cc_trajectory std::vector<Eigen::VectorXd> containing the differential joint trajectory (modified by
     * reference to absolute trajectory).
     * @param start_joint_states Eigen::VectorXd containing the reference start joint states.
     * @param gripper_joint_trajectory std::vector<double> containing the gripper joint trajectory.
     */
    void convert_cc_traj_to_robot_traj_(std::vector<Eigen::VectorXd> &cc_trajectory,
                                        const Eigen::VectorXd &start_joint_states,
                                        const std::vector<double> &gripper_joint_trajectory = std::vector<double>());

    /**
     * @brief Validates the robot and task descriptions provided for CC Affordance planning.
     *
     * @param robot_description `affordance_util::RobotDescription` containing the robot's description.
     * @param task_description `cc_affordance_planner::TaskDescription` containing the task description for CC
     * Affordance planning.
     */
    void validate_input_(const affordance_util::RobotDescription &robot_description,
                         const TaskDescription &task_description);
};

} // namespace cc_affordance_planner

#endif // CC_AFFORDANCE_PLANNER_INTERFACE
