///////////////////////////////////////////////////////////////////////////////
//      Title     : cc_affordance_planner.hpp
//      Project   : cc_affordance_planner
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

#ifndef CC_AFFORDANCE_PLANNER
#define CC_AFFORDANCE_PLANNER

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <chrono>
#include <iostream>
#include <optional>
#include <vector>

// Struct to store result from planner
/**
 * @brief Designed to contain the result of an IK planner with fields:
 * success indicating success; traj_full_or_partial indicating full or partial trajectory with values, "Full",
 * "Partial", or "Unset"; joint_traj representing the joint trajectory; and planning_time representing time taken for
 * planning in microseconds.
 */
struct PlannerResult
{
    bool success;
    std::string traj_full_or_partial;
    std::vector<Eigen::VectorXd> joint_traj;
    std::chrono::microseconds planning_time; // ms
};

/**
 * @brief Differential joint trajectory planner for a given affordance.
 *        Construct this planner with an empty constructor and set the following planner parameters, which are available
 *        as public variables:
 *
 *        - Use the parameter p_aff_step_deltatheta_a to specify the trajectory density with an affordance step.
 *          For example, an affordance goal of 0.5 rad could have an affordance step of 0.1 rad,
 *          resulting in a trajectory with 5 points, where from start to end, the intermediate affordance goals
 *          are 0.1, 0.2, 0.3, 0.4, and 0.5 rad.
 *
 *        - Another important parameter is p_task_err_threshold_eps_s, representing the error threshold
 *          for the affordance step. For instance, if 10% accuracy is desired for a 0.1 rad step, set the error
 * 	    threshold to 0.01 rad.
 *
 *        - Advanced users can utilize two additional parameters:
 *          - p_closure_err_threshold_eps_r: Specify the error threshold for the closed-chain closure error.
 *          - p_max_itr_l: Specify the maximum iterations for the closed-chain inverse kinematics solver.
 *
 * @note The provided parameters allow users to control the trajectory density, accuracy, and solver behavior.
 */

class CcAffordancePlanner
{
  public:
    // See Doxygen brief above
    double p_aff_step_deltatheta_a = 0.1;
    double p_task_err_threshold_eps_s = 0.01;
    double p_closure_err_threshold_eps_r = 1e-6;
    int p_max_itr_l = 50;

    // Constructor
    CcAffordancePlanner();

    // Methods
    /**
     * @brief After setting the planner parameters described in the class documentation, call this function to generate
     * the differential joint trajectory to execute a given affordance.
     *
     * @param slist Eigen::MatrixXd containing as columns 6x1 Screws representing all joints of the closed-chain model,
     * i.e. robot joints, virtual ee joint, affordance joint
     * @param theta_adf Affordance goal for the affordance
     * @param task_offset_tau A numeric parameter indicating the length of the secondary joint vector.
     * The value 1 implies only affordance control, 2 represents affordance control
     * along with fixing the gripper x-axis, 3 involves fixing the gripper x and y axes,
     * and 4 involves fixing the gripper x, y, and z axes.
     *
     * @return Struct containing the result of the planning with fields: success indicating success;
     * traj_full_or_partial indicating full or partial trajectory with values, "Full", "Partial", or "Unset"; joint_traj
     * representing the joint trajectory; and planning_time representing time taken for planning in microseconds.
     */
    PlannerResult affordance_stepper(const Eigen::MatrixXd &slist, const double &theta_adf,
                                     const size_t &task_offset_tau);
    /**
     * @brief Given a list of closed-chain Screws, initial primary and secondary joint guesses, and desired secondary
     * joint goals, computes the closed-chain inverse kinematics solution.
     *
     * @param slist Eigen::MatrixXd containing as columns 6x1 Screws of the closed-chain mechanism
     * @param theta_pg Eigen::VectorXd containing guesses for the primary joints
     * @param theta_sg Eigen::VectorXd containing guesses for the secondary joints
     * @param theta_sd Eigen::VectorXd containing goals for the secondary joints
     *
     * @return Eigen::VectorXd containing the inverse kinematics solution (including the exact secondary joint goals as
     * well)
     */
    std::optional<Eigen::VectorXd> cc_ik_solver(const Eigen::MatrixXd &slist, const Eigen::VectorXd &theta_pg,
                                                const Eigen::VectorXd &theta_sg, const Eigen::VectorXd &theta_sd);

    /**
     * @brief Given a list of closed-chain screw axes, primary and secondary network matrices, and primary and secondary
     * joint angles, returns joint angles corrected for closed-chain closure error
     *
     * @param slist Eigen::MatrixXd containing as columns 6x1 Screws of the closed-chain mechanism
     * @param Np Eigen::MatrixXd representing primary network matrix
     * @param Ns Eigen::MatrixXd representing secondary network matrix
     * @param theta_p Eigen::VectorXd containing primary joint angles
     * @param theta_s Eigen::VectorXd containing secondary joint angles
     */
    void closure_error_optimizer(const Eigen::MatrixXd &slist, const Eigen::MatrixXd &Np, const Eigen::MatrixXd &Ns,
                                 Eigen::VectorXd &theta_p,
                                 Eigen::VectorXd &theta_s); // theta_sg and theta_p returned by referece

  private:
    constexpr static size_t twist_length_ = 6; // length of a twist vector
    size_t nof_pjoints_;                       // number of primary joints
    size_t nof_sjoints_;                       // number of secondary joints
};
#endif // CC_AFFORDANCE_PLANNER