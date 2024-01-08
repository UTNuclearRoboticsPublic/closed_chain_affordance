/*
   Author: Crasun Jans
*/
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
 * success of type bool, traj_full_or_partial of type string with values: "Full",
 * "Partial", or "Unset", and joint_traj of type std::vector<Eigen::VectorXd
 */
struct PlannerResult
{
    bool success;
    std::string traj_full_or_partial;
    std::vector<Eigen::VectorXd> joint_traj;
};

/**
 * @brief Given a list of closed-chain screw axes, corresponding starting joint
 * angles, and affordance goal, this planner computes the inverse kinematics
 * joint trajectory solution.
 */
class CcAffordancePlanner
{
  public:
    //* Algorithm control parameters (non-const to adjust from outside)
    // For stepper loop
    double p_aff_step_deltatheta_a = 0.1;
    // For IK loop
    /* double accuracy = 10.0 * (1.0 / 100.0); // accuracy for error threshold */
    double p_task_err_threshold_eps_s = 0.01;
    double p_closure_err_threshold_eps_r = 1e-6;
    int p_max_itr_l = 50;

    // Constructor
    /**
     * @brief Given a list of closed-chain screw axes, corresponding starting
     joint angles, and affordance goal, this planner computes the inverse
     kinematics joint trajectory solution.
     *
     * @param slist List of all closed-chain screw axes
     * @param thetalist0 Corresponding joint angles for starting configuration
     * @param affGoal Desired affordance goal
     */
    CcAffordancePlanner();

    // Methods
    /**
     * @brief Steps the IK solver towards the affordance goal affGoal by an
     * affordance step (p_aff_step_deltatheta_a)
     *
     * @return Struct containing the result of the planning with fields:
     success of type bool, traj_full_or_partial of type string with values: "Full",
     "Partial", or "Unset", and joint_traj of type std::vector<Eigen::VectorXd
     */

    PlannerResult affordance_stepper(const Eigen::MatrixXd &slist, const double &theta_adf,
                                     const size_t &task_offset_tau);
    /**
     * @brief Closed-chain IK solver. Not standalone.
     *
     * @return
     */
    std::optional<Eigen::VectorXd> cc_ik_solver(const Eigen::MatrixXd &slist, const Eigen::VectorXd &theta_p,
                                                const Eigen::VectorXd &theta_sg, const Eigen::VectorXd &theta_sd);

    /**
     * @brief Closed-chain error corrector. Not standalone.
     */
    void closure_error_optimizer(const Eigen::MatrixXd &slist, const Eigen::MatrixXd &Np, const Eigen::MatrixXd &Ns,
                                 Eigen::VectorXd &theta_p,
                                 Eigen::VectorXd &theta_sg); // theta_sg and theta_p returned by referece

  private:
    constexpr static size_t twist_length_ = 6; // length of a twist vector
    const double dt_ = 1e-2;                   // time step to compute joint velocities

    // Helper variables to share info between functions
    Eigen::VectorXd thetalist_; // helper vector variable holding theta_p, theta_sg
    size_t nof_pjoints_;
    size_t nof_sjoints_;
    Eigen::Matrix4d des_endlink_htm_ = Eigen::Matrix4d::Identity();
};
#endif // CC_AFFORDANCE_PLANNER
