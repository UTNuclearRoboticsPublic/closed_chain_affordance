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
#include <optional>
#include <vector>

namespace cc_affordance_planner
{

/**
 * @brief Enum describing the closed-chain affordance motion type
 */
enum MotionType
{
    APPROACH,
    AFFORDANCE
};

/**
 * @brief Enum qualitatively describing a trajectory length as FULL, PARTIAL, or UNSET
 */
enum TrajectoryDescription
{
    FULL,
    PARTIAL,
    UNSET
};

/**
 * @brief Enum representing update methods for the closed-chain affordance planner
 */
enum UpdateMethod
{
    INVERSE,
    TRANSPOSE,
    BEST
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
};

/**
 * @brief Designed to contain the result of the Closed-chain Affordance planner with fields:
 * success indicating success; trajectory_description indicating full or partial trajectory with values, "FULL",
 * "PARTIAL", or "UNSET"; joint_trajectory representing the joint trajectory; planning_time representing time taken for
 * planning in microseconds; update_method indicating the type of update scheme used, for instance "inverse",
 * "inverse with dls", and "transpose"; and includes_gripper_trajectory indicating whether the result includes gripper
 * trajectory
 */
struct PlannerResult
{
    bool success = false;
    TrajectoryDescription trajectory_description;
    std::vector<Eigen::VectorXd> joint_trajectory;
    std::chrono::microseconds planning_time;
    UpdateMethod update_method;
    std::string update_trail = "";
    bool includes_gripper_trajectory = false;
};

/**
 * @brief Designed to contain the configuration settings for the Closed-chain Affordance planner with fields:
 * accuracy indicating the accuracy of the planner as percentage; closure_err_threshold indicating the threshold for the
 * closed-chain closure error; and ik_max_itr indicating the maximum interations for the closed-chain IK solver.
 */
struct PlannerConfig
{
    double accuracy = 10.0 / 100.0;
    double closure_err_threshold_ang = 1e-4;
    double closure_err_threshold_lin = 1e-5;
    int ik_max_itr = 200;
    UpdateMethod update_method = UpdateMethod::BEST;
};

/**
 * @brief Base Class for the Closed-Chain Affordance Planner
 */
class CcAffordancePlanner
{
  public:
    // Constructor
    explicit CcAffordancePlanner(const PlannerConfig &plannerConfig = PlannerConfig());

    // Methods
    /**
     * @brief After setting the planner parameters described in the class documentation, call this function to generate
     * the differential joint trajectory to execute a given approach motion.
     *
     * @param slist Eigen::MatrixXd containing as columns 6x1 Screws representing all joints of the closed-chain model,
     * i.e. robot joints, virtual ee joint, affordance joint
     * @param theta_sdf Eigen::VectorXd containing secondary joint angle goals including EE orientation and affordance
     * such that the affordance goal is the end element.
     * @param task_offset_tau A numeric parameter indicating the length of the secondary joint vector.
     * The value 1 implies only affordance control, 2 represents affordance control
     * along with fixing the gripper x-axis, 3 involves fixing the gripper x and y axes,
     * and 4 involves fixing the gripper x, y, and z axes.
     * @param stepper_max_itr_m int denoting the density for the trajectory in terms of number of points
     *
     * @return Struct containing the result of the planning with fields: success indicating success;
     * traj_full_or_partial indicating full or partial trajectory with values, "Full", "Partial", or "Unset";
     * joint_trajectory representing the joint trajectory; and planning_time representing time taken for planning in
     * microseconds.
     */
    PlannerResult generate_approach_motion_joint_trajectory(const Eigen::MatrixXd &slist,
                                                            const Eigen::VectorXd &theta_sdf,
                                                            const size_t &task_offset_tau,
                                                            const int &stepper_max_itr_m);

    /**
     * @brief After setting the planner parameters described in the class documentation, call this function to generate
     * the differential joint trajectory to execute a given approach motion.
     *
     * @param slist Eigen::MatrixXd containing as columns 6x1 Screws representing all joints of the closed-chain model,
     * i.e. robot joints, virtual ee joint, affordance joint
     * @param theta_sdf Eigen::VectorXd containing secondary joint angle goals including EE orientation and affordance
     * such that the affordance goal is the end element.
     * @param task_offset_tau A numeric parameter indicating the length of the secondary joint vector.
     * The value 1 implies only affordance control, 2 represents affordance control
     * along with fixing the gripper x-axis, 3 involves fixing the gripper x and y axes,
     * and 4 involves fixing the gripper x, y, and z axes.
     * @param stepper_max_itr_m int denoting the density for the trajectory in terms of number of points
     * @param st std::stop_token for cooperative interruption when multi-threading
     *
     * @return Struct containing the result of the planning with fields: success indicating success;
     * traj_full_or_partial indicating full or partial trajectory with values, "Full", "Partial", or "Unset";
     * joint_trajectory representing the joint trajectory; and planning_time representing time taken for planning in
     * microseconds.
     */
    PlannerResult generate_approach_motion_joint_trajectory(const Eigen::MatrixXd &slist,
                                                            const Eigen::VectorXd &theta_sdf,
                                                            const size_t &task_offset_tau, const int &stepper_max_itr_m,
                                                            std::stop_token st);
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
     * @param stepper_max_itr_m int denoting the density for the trajectory in terms of number of points
     *
     * @return Struct containing the result of the planning with fields: success indicating success;
     * traj_full_or_partial indicating full or partial trajectory with values, "Full", "Partial", or "Unset";
     * joint_trajectory representing the joint trajectory; and planning_time representing time taken for planning in
     * microseconds.
     */
    PlannerResult generate_affordance_motion_joint_trajectory(const Eigen::MatrixXd &slist,
                                                              const Eigen::VectorXd &theta_sdf,
                                                              const size_t &task_offset_tau,
                                                              const int &stepper_max_itr_m);

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
     * traj_full_or_partial indicating full or partial trajectory with values, "Full", "Partial", or "Unset";
     * joint_trajectory representing the joint trajectory; and planning_time representing time taken for planning in
     * microseconds.
     */
    PlannerResult generate_affordance_motion_joint_trajectory(const Eigen::MatrixXd &slist,
                                                              const Eigen::VectorXd &theta_sdf,
                                                              const size_t &task_offset_tau,
                                                              const int &stepper_max_itr_m, std::stop_token st);

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
    double accuracy_; // accuracy of the secondary goals
    double eps_rw_;   // closure error threshold for angular part
    double eps_rv_;   // closure error threshold for linear part
    int max_itr_l_;   // max interations for IK solver
    //--EOF Planner config parameters
    constexpr static size_t twist_length_ = 6; // length of a twist vector
    Eigen::VectorXd theta_s_tol_;              // IK tolerance for secondary joint goal vector

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
                                  Eigen::VectorXd &theta_p, Eigen::VectorXd &theta_s, Eigen::VectorXd &rho);

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
    explicit CcAffordancePlannerTranspose() : CcAffordancePlanner() {}
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
    explicit CcAffordancePlannerInverse() : CcAffordancePlanner() {}
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
