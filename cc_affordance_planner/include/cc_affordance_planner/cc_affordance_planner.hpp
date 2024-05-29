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

#include "jthread/jthread.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <vector>

namespace cc_affordance_planner
{

/**
 * @brief Designed to contain the result of the Closed-chain Affordance planner with fields:
 * success indicating success; traj_full_or_partial indicating full or partial trajectory with values, "Full",
 * "Partial", or "Unset"; joint_traj representing the joint trajectory; planning_time representing time taken for
 * planning in microseconds; and update_method indicating the type of update scheme used, for instance "inverse",
 * "inverse with dls", and "transpose".
 */
struct PlannerResult
{
    bool success;
    std::string traj_full_or_partial;
    std::vector<Eigen::VectorXd> joint_traj;
    std::chrono::microseconds planning_time;
    std::string update_method = ""; // Default is empty
};

/**
 * @brief Designed to contain the configuration settings for the Closed-chain Affordance planner with fields: aff_step
 * indicating the density of the generated trajectory as the distance between two points on the affordance path;
 * accuracy indicating the accuracy of the planner as percentage; closure_err_threshold indicating the threshold for the
 * closed-chain closure error; and max_itr indicating the maximum interations for the closed-chain IK solver.
 */
struct PlannerConfig
{

    double aff_step = 0.1;
    double accuracy = 10.0 / 100.0;
    double closure_err_threshold = 1e-6;
    int max_itr = 200;
};

/**
 * @brief Given planner configuration, closed-chain
 * screws, secondary joint (affordance and/or gripper orientation) goals, and gripper orientation control parameter,
 * generates a differential joint trajectory to reach desired goals.
 *
 * @param plannerConfig Struct containing planner settings:
 *        - Use the parameter aff_step to specify the trajectory density with an affordance step.
 *          For example, an affordance goal of 0.5 rad could have an affordance step of 0.1 rad,
 *          resulting in a trajectory with 5 points, where from start to end, the intermediate affordance goals
 *          are 0.1, 0.2, 0.3, 0.4, and 0.5 rad.
 *        - Another important parameter is accuracy, which represents the threshold for the affordance goal (and
 *          step). For instance, if 10% accuracy is desired for 1 rad goal, set accuracy to 0.1. This will produce joint
 *          solutions that result in an affordance goal of 1 +- 0.1.
 *        - Advanced users can utilize two additional parameters:
 *          - closure_error_threshold: Specify the error threshold for the closed-chain closure error.
 *          - max_itr: Specify the maximum iterations for the closed-chain inverse kinematics solver.
 * @param slist Eigen::MatrixXd containing as columns 6x1 Screws representing all joints of the closed-chain model,
 * i.e., robot joints, virtual ee joint, affordance joint.
 * @param theta_sdf Eigen::VectorXd containing secondary joint angle goals including EE orientation and affordance
 * such that the affordance goal is the end element.
 * @param task_offset_tau A numeric parameter indicating the length of the secondary joint vector.
 * The value 1 implies only affordance control, 2 represents affordance control
 * along with fixing the gripper x-axis, 3 involves fixing the gripper x and y axes,
 * and 4 involves fixing the gripper x, y, and z axes.
 * @param slist Eigen::MatrixXd containing as columns 6x1 Screws of the closed-chain mechanism.
 *
 * @return Struct containing the result of the planning with fields:
 * success indicating success;
 * traj_full_or_partial indicating full or partial trajectory with values, "Full", "Partial", or "Unset";
 * joint_traj representing the joint trajectory;
 * planning_time representing time taken for planning in microseconds; and
 * update_method indicating whether transpose or inverse method was used.
 */

PlannerResult generate_joint_trajectory(const PlannerConfig &plannerConfig, const Eigen::MatrixXd &slist,
                                        const Eigen::VectorXd &theta_sdf, const size_t &task_offset_tau);

/**
 * @brief Base Class for the Closed-Chain Affordance Planner
 */
class CcAffordancePlanner
{
  public:
    // Constructor
    explicit CcAffordancePlanner(const PlannerConfig &plannerConfig);

    // Methods
    /**
     * @brief After setting the planner parameters described in the class documentation, call this function to generate
     * the differential joint trajectory to execute a given affordance.
     *
     * @param slist Eigen::MatrixXd containing as columns 6x1 Screws representing all joints of the closed-chain model,
     * i.e. robot joints, virtual ee joint, affordance joint
     * @param theta_sdf Eigen::VectorXd containing secondary joint angle goals including EE orientation and affordance
     * such that the affordance goal is the end element.
     * @param task_offset_tau A numeric parameter indicating the length of the secondary joint vector.
     * The value 1 implies only affordance control, 2 represents affordance control
     * along with fixing the gripper x-axis, 3 involves fixing the gripper x and y axes,
     * and 4 involves fixing the gripper x, y, and z axes.
     *
     * @return Struct containing the result of the planning with fields: success indicating success;
     * traj_full_or_partial indicating full or partial trajectory with values, "Full", "Partial", or "Unset"; joint_traj
     * representing the joint trajectory; and planning_time representing time taken for planning in microseconds.
     */
    PlannerResult generate_joint_trajectory(const Eigen::MatrixXd &slist, const Eigen::VectorXd &theta_sdf,
                                            const size_t &task_offset_tau);

    /**
     * @brief After setting the planner parameters described in the class documentation, call this function to generate
     * the differential joint trajectory to execute a given affordance.
     *
     * @param slist Eigen::MatrixXd containing as columns 6x1 Screws representing all joints of the closed-chain model,
     * i.e. robot joints, virtual ee joint, affordance joint
     * @param theta_sdf Eigen::VectorXd containing secondary joint angle goals including EE orientation and affordance
     * such that the affordance goal is the end element.
     * @param task_offset_tau A numeric parameter indicating the length of the secondary joint vector.
     * The value 1 implies only affordance control, 2 represents affordance control
     * along with fixing the gripper x-axis, 3 involves fixing the gripper x and y axes,
     * and 4 involves fixing the gripper x, y, and z axes.
     * @param st std::stop_token for cooperative interruption when multi-threading
     *
     * @return Struct containing the result of the planning with fields: success indicating success;
     * traj_full_or_partial indicating full or partial trajectory with values, "Full", "Partial", or "Unset"; joint_traj
     * representing the joint trajectory; and planning_time representing time taken for planning in microseconds.
     */
    PlannerResult generate_joint_trajectory(const Eigen::MatrixXd &slist, const Eigen::VectorXd &theta_sdf,
                                            const size_t &task_offset_tau, std::stop_token st);

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
    std::optional<Eigen::VectorXd> call_cc_ik_solver(const Eigen::MatrixXd &slist, const Eigen::VectorXd &theta_pg,
                                                     const Eigen::VectorXd &theta_sg, const Eigen::VectorXd &theta_sd);

    /**
     * @brief Given a list of closed-chain Screws, initial primary and secondary joint guesses, and desired secondary
     * joint goals, computes the closed-chain inverse kinematics solution.
     *
     * @param slist Eigen::MatrixXd containing as columns 6x1 Screws of the closed-chain mechanism
     * @param theta_pg Eigen::VectorXd containing guesses for the primary joints
     * @param theta_sg Eigen::VectorXd containing guesses for the secondary joints
     * @param theta_sd Eigen::VectorXd containing goals for the secondary joints
     * @param st std::stop_token for cooperative interruption when multi-threading
     *
     * @return Eigen::VectorXd containing the inverse kinematics solution (including the exact secondary joint goals as
     * well)
     */
    std::optional<Eigen::VectorXd> call_cc_ik_solver(const Eigen::MatrixXd &slist, const Eigen::VectorXd &theta_pg,
                                                     const Eigen::VectorXd &theta_sg, const Eigen::VectorXd &theta_sd,
                                                     std::stop_token st);

  protected:
    size_t nof_pjoints_;            // number of primary joints
    size_t nof_sjoints_;            // number of secondary joints
    double cond_N_threshold_ = 100; // condition number threshold for N to be considered singular
    bool dls_flag_ = false;         // flag indicating whether DLS method was used
    double lambda_ = 1.1;           // damping factor for the DLS method

  private:
    //--Planner config parameters
    double deltatheta_a_; // affordance step
    double accuracy_;     // accuracy of the affordance goal
    double eps_r_;        // closure error threshold
    int max_itr_l_;       // max interations for IK solver
    //--EOF Planner config parameters
    constexpr static size_t twist_length_ = 6; // length of a twist vector

    /**
     * @brief Given a list of closed-chain screw axes, primary and secondary network matrices, and primary and secondary
     * joint angles, returns by reference joint angles corrected for closed-chain closure error.
     *
     * @param slist Eigen::MatrixXd containing as columns 6x1 Screws of the closed-chain mechanism
     * @param Np Eigen::MatrixXd representing primary network matrix
     * @param Ns Eigen::MatrixXd representing secondary network matrix
     * @param theta_p Eigen::VectorXd containing primary joint angles
     * @param theta_s Eigen::VectorXd containing secondary joint angles
     */
    void adjust_for_closure_error(const Eigen::MatrixXd &slist, const Eigen::MatrixXd &Np, const Eigen::MatrixXd &Ns,
                                  Eigen::VectorXd &theta_p, Eigen::VectorXd &theta_s);

    /**
     * @brief Given the primary joint angles, desired and current secondary joint angles, and constraint Jacobian,
     * returns by reference updated primary joint angles. This function must be overriden in child class implementation.
     *
     * @param theta_p Eigen::VectorXd containing primary joint angles
     * @param theta_sd Eigen::VectorXd containing desired secondary joint angles
     * @param theta_s Eigen::VectorXd containing secondary joint angles
     * @param N Eigen::MatrixXd representing the constraint Jacobian
     */
    virtual void update_theta_p(Eigen::VectorXd &theta_p, const Eigen::VectorXd &theta_sd,
                                const Eigen::VectorXd &theta_s, const Eigen::MatrixXd &N) = 0;
};

/**
 * @brief Child class for the Closed-Chain Affordance Planner implementing the transpose method
 */
class CcAffordancePlannerTranspose : public CcAffordancePlanner
{
  public:
    explicit CcAffordancePlannerTranspose(const PlannerConfig &plannerConfig) : CcAffordancePlanner(plannerConfig) {}

  private:
    /**
     * @brief Given the primary joint angles, desired and current secondary joint angles, and constraint Jacobian,
     * returns by reference updated primary joint angles using the transpose method.
     *
     * @param theta_p Eigen::VectorXd containing primary joint angles
     * @param theta_sd Eigen::VectorXd containing desired secondary joint angles
     * @param theta_s Eigen::VectorXd containing secondary joint angles
     * @param N Eigen::MatrixXd representing the constraint Jacobian
     */
    virtual void update_theta_p(Eigen::VectorXd &theta_p, const Eigen::VectorXd &theta_sd,
                                const Eigen::VectorXd &theta_s, const Eigen::MatrixXd &N) override;
};

/**
 * @brief Child class for the Closed-Chain Affordance Planner implementing the inverse method
 */
class CcAffordancePlannerInverse : public CcAffordancePlanner
{
  public:
    explicit CcAffordancePlannerInverse(const PlannerConfig &plannerConfig) : CcAffordancePlanner(plannerConfig) {}

  private:
    /**
     * @brief Given the primary joint angles, desired and current secondary joint angles, and constraint Jacobian,
     * returns by reference updated primary joint angles using the pseudoinverse method or DLS if near singularities.
     *
     * @param theta_p Eigen::VectorXd containing primary joint angles
     * @param theta_sd Eigen::VectorXd containing desired secondary joint angles
     * @param theta_s Eigen::VectorXd containing secondary joint angles
     * @param N Eigen::MatrixXd representing the constraint Jacobian
     */
    virtual void update_theta_p(Eigen::VectorXd &theta_p, const Eigen::VectorXd &theta_sd,
                                const Eigen::VectorXd &theta_s, const Eigen::MatrixXd &N) override;
};

} // namespace cc_affordance_planner
#endif // CC_AFFORDANCE_PLANNER
