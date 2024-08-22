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

class CcAffordancePlannerInterface
{

  public:
    using Gsmt = PlannerResult (CcAffordancePlanner::*)(
        const Eigen::MatrixXd &, const Eigen::VectorXd &,
        const size_t &); // function pointer to CcAffordancePlanner::generate_approach_motion_joint_trajectory or
                         // CcAffordancePlanner::generate_affordance_motion_joint_trajectory
    using Gsmt_st = PlannerResult (CcAffordancePlanner::*)(const Eigen::MatrixXd &, const Eigen::VectorXd &,
                                                           const size_t &,
                                                           std::stop_token); // Gsmt with st
    /**
     * @brief Given cc affordance planner configuration information constructs the CcAffordancePlannerInterface object
     *
     * @param plannerConfig Struct containing planner settings:
     *        - Use the parameter trajectory_density to specify the trajectory density as number of points in the
     *trajectory. For example, an affordance goal of 0.5 rad could have an affordance step of 0.1 rad, resulting in a
     *trajectory with 5 points, where from start to end, the intermediate affordance goals are 0.1, 0.2, 0.3, 0.4, and
     *0.5 rad.
     *        - Another important parameter is accuracy, which represents the threshold for the affordance goal (and
     *          step). For instance, if 10% accuracy is desired for 1 rad goal, set accuracy to 0.1. This will produce
     *joint solutions that result in an affordance goal of 1 +- 0.1.
     *        - Advanced users can utilize three additional parameters:
     *          - closure_error_threshold: Specify the error threshold for the closed-chain closure error.
     *          - ik_max_itr: Specify the maximum iterations for the closed-chain inverse kinematics solver.
     *          - update_method: Specify which update method to use. Possible values are cc_affordance_planner::INVERSE,
     *	      cc_affordance_planner::TRANSPOSE, and cc_affordance_planner::BEST. The default value is
     *	      cc_affordance_planner::BEST.
     */
    explicit CcAffordancePlannerInterface(const PlannerConfig &plannerConfig);
    /**
     * @brief Given robot and task description generates a closed-chain joint trajectory to
     * perform desired tasks. For each point in the closed-chain joint trajectory, the first n joint positions are
     *simply the robot joint states for the robot with n joints.
     *
     * @param robot_description affordance_util::RobotDescription containing the description of a robot in terms of its
     *screw list, end-effector homogeneous transformation matrix, and the current joint states
     * @param task_description cc_affordance_planner::TaskDescription describing a task in terms of affordance info,
     *number of secondary joints, secondary goal, virtual screw order, grasp pose, and post-grasp affordance goal.
     *
     * @return cc_affordance_planner::PlannerResult containing the solved closed-chain joint trajectory along with
     *additional planning process information. Note that for each point in the closed-chain joint trajectory, the first
     *n joint positions are simply the robot joint states, and the rest the corresponding secondary joint states in the
     *closed-chain model.
     */
    PlannerResult generate_joint_trajectory(const affordance_util::RobotDescription &robot_description,
                                            const TaskDescription &task_description);

  private:
    PlannerConfig plannerConfig_;

    /**
     * @brief Given a function pointer to approach or affordance type joint trajectory generator in CcAffordancePlanner,
     closed-chain
     * screws, secondary joint (affordance , approach, and/or gripper orientation) goals, and gripper orientation
     control parameter,
     * generates a differential joint trajectory to reach desired goals.
     *
     * @param generate_specified_motion_joint_trajectory (CcAffordancePlanner::*)(const Eigen::MatrixXd &, const
     Eigen::VectorXd &, const size_t &) const function pointer to
     CcAffordancePlanner::generate_approach_motion_joint_trajectory or
     CcAffordancePlanner::generate_affordance_motion_joint_trajectory
     * @param generate_specified_motion_joint_trajectory_st (CcAffordancePlanner::*)(const Eigen::MatrixXd &, const
     Eigen::VectorXd &, const size_t &, std::stop_token) const function pointer to
     CcAffordancePlanner::generate_approach_motion_joint_trajectory or
     CcAffordancePlanner::generate_affordance_motion_joint_trajectory
     * @param slist Eigen::MatrixXd containing as columns 6x1 Screws representing all joints of the closed-chain model,
     * i.e., robot joints, virtual ee joint, affordance joint.
     * @param theta_sdf Eigen::VectorXd containing secondary joint angle goals including EE orientation and affordance
     * such that the affordance goal is the end element.
     * @param task_offset_tau A numeric parameter indicating the length of the secondary joint vector:
     *        - A value of 1 implies only affordance control.
     *        - A value of 2 represents affordance control along with controlling the gripper orientation about the next
     *	    adjacent virtual gripper axis (x, y, or z).
     *        - A value of 3 involves controlling affordance along with the EE orientation about the next two virtual
     *          gripper axes.
     *        - A value of 4 refers to affordance control along with all aspects of EE orientation.
     *
     * @return cc_affordance_planner::PlannerResult containing the solved differential closed-chain joint trajectory
     along with additional *planning process information
     */
    PlannerResult generate_specified_motion_joint_trajectory_(
        const Gsmt &generate_specified_motion_joint_trajectory,
        const Gsmt_st &generate_specified_motion_joint_trajectory_st, const Eigen::MatrixXd &slist,
        const Eigen::VectorXd &theta_sdf, const size_t &task_offset_tau);

    /**
     * @brief Given a differential joint trajectory and a reference joint state, returns by reference the absolute joint
     * trajectory by inserting the reference joint state in the beginning of the trajectory and adding it to the rest of
     * the trajectory points.
     *
     * @param cc_trajectory std::vector<Eigen::VectorXd> containing a differential joint trajectory. This parameter is
     * returned by reference as the absolute trajectory
     * @param start_joint_states Eigen::VectorXd containing the reference start joint states
     */
    void convert_cc_traj_to_robot_traj_(std::vector<Eigen::VectorXd> &cc_trajectory,
                                        const Eigen::VectorXd &start_joint_states);

    /**
     * @brief Given a robot and task description for the CC Affordance Planner, checks the descriptions for potential
     * errors
     *
     * @param robot_description affordance_uti::RobotDescription containing description of the robot
     * @param task_description cc_affordance_planner::TaskDescription containing the description of the task for CC
     * Affordance planning
     */
    void validate_input_(const affordance_util::RobotDescription &robot_description,
                         const TaskDescription &task_description);
};

} // namespace cc_affordance_planner
#endif // CC_AFFORDANCE_PLANNER_INTERFACE
