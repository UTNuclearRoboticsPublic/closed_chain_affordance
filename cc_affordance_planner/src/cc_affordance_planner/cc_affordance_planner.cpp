#include <cc_affordance_planner/cc_affordance_planner.hpp>

namespace cc_affordance_planner
{

void CcAffordancePlannerTranspose::update_theta_p(Eigen::VectorXd &theta_p, const Eigen::VectorXd &theta_sd,
                                                  const Eigen::VectorXd &theta_s, const Eigen::MatrixXd &N)
{

    //**Alg2:L11: Update theta_p using Eqn. 24 but approximate the inverse with transpose
    const Eigen::VectorXd delta_theta_p = N.transpose() * (theta_sd - theta_s);

    // Update thate_p using Newton-Raphson
    theta_p += delta_theta_p;
}

void CcAffordancePlannerInverse::update_theta_p(Eigen::VectorXd &theta_p, const Eigen::VectorXd &theta_sd,
                                                const Eigen::VectorXd &theta_s, const Eigen::MatrixXd &N)
{

    const Eigen::MatrixXd pinv_N = N.completeOrthogonalDecomposition().pseudoInverse(); // pseudo-inverse of N
    const double cond_N = N.norm() * pinv_N.norm();
    Eigen::VectorXd delta_theta_p(nof_pjoints_);

    // If N is near-singular use Damped Least Squares
    if (cond_N > cond_N_threshold_)
    {
        dls_flag_ = true;
        const Eigen::MatrixXd NNt = N * N.transpose();
        const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(NNt.rows(), NNt.cols());

        // Compute delta_theta_p using the Damped-Least-Squares method
        delta_theta_p = N.transpose() *
                        ((NNt + std::pow(lambda_, 2) * I).completeOrthogonalDecomposition().pseudoInverse()) *
                        (theta_sd - theta_s);
    }
    else // Use the regular inverse method
    {
        //**Alg2:L11: Update theta_p using Eqn. 24
        delta_theta_p = pinv_N * (theta_sd - theta_s);
    }

    // Update thate_p using Newton-Raphson
    theta_p += delta_theta_p;
}
CcAffordancePlanner::CcAffordancePlanner(const PlannerConfig &plannerConfig)
    : stepper_max_itr_m_(plannerConfig.trajectory_density),
      accuracy_(plannerConfig.accuracy),
      eps_rw_(plannerConfig.closure_err_threshold_ang),
      eps_rv_(plannerConfig.closure_err_threshold_lin),
      max_itr_l_(plannerConfig.ik_max_itr)
{
}

PlannerResult CcAffordancePlanner::generate_approach_motion_joint_trajectory(const Eigen::MatrixXd &slist,
                                                                             const Eigen::VectorXd &theta_sdf,
                                                                             const size_t &task_offset_tau,
                                                                             std::stop_token st)
{

    auto start_time = std::chrono::high_resolution_clock::now(); // Monitor clock to track planning time

    PlannerResult plannerResult;                   // Result of the planner
    const double theta_adf = theta_sdf.tail(1)(0); // affordance screw goal
    const double theta_pdf = theta_sdf.tail(2)(0); // approach screw goal

    //**Alg1:L1: Define affordance step, deltatheta_a
    const double deltatheta_a = theta_adf / stepper_max_itr_m_;
    const double deltatheta_p = theta_pdf / stepper_max_itr_m_;

    //** Alg1:L2: Determine relevant matrix and vector sizes based on task_offset_tau
    nof_pjoints_ = slist.cols() - task_offset_tau;
    nof_sjoints_ = task_offset_tau;

    //**Alg1:L3 and Alg1:L2: Set start guesses and step goal
    Eigen::VectorXd theta_sg = Eigen::VectorXd::Zero(nof_sjoints_);
    Eigen::VectorXd theta_pg = Eigen::VectorXd::Zero(nof_pjoints_);
    Eigen::VectorXd theta_sd = theta_sdf; // We set the affordance goal in the loop in reference to the start state
    theta_sd.tail(2).setConstant(
        start_guess_); // start approach and affordance goals at 0 but gripper orientation as specified

    //**Alg1:L4: Compute no. of iterations, stepper_max_itr_m_ to final goal: Passed in as planner config

    //**Alg1:L5: Initialize loop counter, loop_counter_k; success counter, success_counter_s
    int loop_counter_k = 0;
    int success_counter_s = 0;

    while (loop_counter_k < stepper_max_itr_m_ && !st.stop_requested()) //**Alg1:L6
    {
        loop_counter_k = loop_counter_k + 1; //**Alg1:L7:

        //**Alg1:L8: Update aff and approach step goal:
        // Set the affordance step goal as aff_step away from the current pose. Affordance is the last element of
        // theta_sd
        theta_sd(nof_sjoints_ - 1) = theta_sd(nof_sjoints_ - 1) - deltatheta_a;
        theta_sd(nof_sjoints_ - 2) = theta_sd(nof_sjoints_ - 2) - deltatheta_p;

        //**Alg1:L13: Call Algorithm 2 with args, theta_sd, theta_pg, theta_sg, slist
        std::optional<Eigen::VectorXd> ik_result = this->call_cc_ik_solver(slist, theta_pg, theta_sg, theta_sd, st);

        if (ik_result.has_value()) //**Alg1:L14
        {
            //**Alg1:L15: Record solution, theta_p, theta_sg
            const Eigen::VectorXd &traj_point = ik_result.value();
            plannerResult.joint_trajectory.push_back(traj_point); // recorded as a point in the trajectory solution

            //**Alg1:L16 Update guesses, theta_pg, theta_sg
            theta_sg = traj_point.tail(nof_sjoints_);
            theta_pg = traj_point.head(nof_pjoints_);

            success_counter_s = success_counter_s + 1; //**Alg1:L17
        }                                              //**Alg1:L18

    } //**Alg1:L19

    // Capture the planning time
    auto end_time = std::chrono::high_resolution_clock::now();
    plannerResult.planning_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    // Set planning result
    if (!plannerResult.joint_trajectory.empty())
    {
        plannerResult.success = true;

        if (loop_counter_k == success_counter_s)
        {
            plannerResult.trajectory_description = TrajectoryDescription::FULL;
        }
        else
        {
            plannerResult.trajectory_description = TrajectoryDescription::PARTIAL;
        }
    }
    else
    {
        plannerResult.success = false;
        plannerResult.trajectory_description = TrajectoryDescription::UNSET;
    }

    // Indicate if DLS was used
    plannerResult.update_trail = dls_flag_ ? "dls and " : plannerResult.update_trail;

    return plannerResult;
}

PlannerResult CcAffordancePlanner::generate_approach_motion_joint_trajectory(const Eigen::MatrixXd &slist,
                                                                             const Eigen::VectorXd &theta_sdf,
                                                                             const size_t &task_offset_tau)
{

    auto start_time = std::chrono::high_resolution_clock::now(); // Monitor clock to track planning time

    PlannerResult plannerResult;                   // Result of the planner
    const double theta_adf = theta_sdf.tail(1)(0); // affordance screw goal
    const double theta_pdf = theta_sdf.tail(2)(0); // approach screw goal

    //**Alg1:L1: Define affordance step, deltatheta_a
    const double deltatheta_a = theta_adf / stepper_max_itr_m_;
    const double deltatheta_p = theta_pdf / stepper_max_itr_m_;

    //** Alg1:L2: Determine relevant matrix and vector sizes based on task_offset_tau
    nof_pjoints_ = slist.cols() - task_offset_tau;
    nof_sjoints_ = task_offset_tau;

    //**Alg1:L3 and Alg1:L2: Set start guesses and step goal
    Eigen::VectorXd theta_sg = Eigen::VectorXd::Zero(nof_sjoints_);
    Eigen::VectorXd theta_pg = Eigen::VectorXd::Zero(nof_pjoints_);
    Eigen::VectorXd theta_sd = theta_sdf; // We set the affordance goal in the loop in reference to the start state
    theta_sd.tail(2).setConstant(
        start_guess_); // start approach and affordance goals at 0 but gripper orientation as specified

    //**Alg1:L4: Compute no. of iterations, stepper_max_itr_m_ to final goal: Passed in as planner config

    //**Alg1:L5: Initialize loop counter, loop_counter_k; success counter, success_counter_s
    int loop_counter_k = 0;
    int success_counter_s = 0;

    while (loop_counter_k < stepper_max_itr_m_) //**Alg1:L6
    {

        loop_counter_k = loop_counter_k + 1; //**Alg1:L7:

        //**Alg1:L8: Update aff and approach step goal:
        // Set the affordance step goal as aff_step away from the current pose. Affordance is the last element of
        // theta_sd
        theta_sd(nof_sjoints_ - 1) = theta_sd(nof_sjoints_ - 1) - deltatheta_a;
        theta_sd(nof_sjoints_ - 2) = theta_sd(nof_sjoints_ - 2) - deltatheta_p;

        //**Alg1:L13: Call Algorithm 2 with args, theta_sd, theta_pg, theta_sg, slist
        std::optional<Eigen::VectorXd> ik_result = this->call_cc_ik_solver(slist, theta_pg, theta_sg, theta_sd);

        if (ik_result.has_value()) //**Alg1:L14
        {
            //**Alg1:L15: Record solution, theta_p, theta_sg
            const Eigen::VectorXd &traj_point = ik_result.value();
            plannerResult.joint_trajectory.push_back(traj_point); // recorded as a point in the trajectory solution

            //**Alg1:L16 Update guesses, theta_pg, theta_sg
            theta_sg = traj_point.tail(nof_sjoints_);
            theta_pg = traj_point.head(nof_pjoints_);

            success_counter_s = success_counter_s + 1; //**Alg1:L17
        }                                              //**Alg1:L18

    } //**Alg1:L19

    // Capture the planning time
    auto end_time = std::chrono::high_resolution_clock::now();
    plannerResult.planning_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    // Set planning result
    if (!plannerResult.joint_trajectory.empty())
    {
        plannerResult.success = true;

        if (loop_counter_k == success_counter_s)
        {
            plannerResult.trajectory_description = TrajectoryDescription::FULL;
        }
        else
        {
            plannerResult.trajectory_description = TrajectoryDescription::PARTIAL;
        }
    }
    else
    {
        plannerResult.success = false;
        plannerResult.trajectory_description = TrajectoryDescription::UNSET;
    }

    // Indicate if DLS was used
    plannerResult.update_trail = dls_flag_ ? "dls and " : plannerResult.update_trail;

    return plannerResult;
}

PlannerResult CcAffordancePlanner::generate_affordance_motion_joint_trajectory(const Eigen::MatrixXd &slist,
                                                                               const Eigen::VectorXd &theta_sdf,
                                                                               const size_t &task_offset_tau,
                                                                               std::stop_token st)
{

    auto start_time = std::chrono::high_resolution_clock::now(); // Monitor clock to track planning time

    PlannerResult plannerResult; // Result of the planner
    const double theta_adf = theta_sdf.tail(1)(0);

    //**Alg1:L1: Define affordance step, deltatheta_a
    const double deltatheta_a = theta_adf / stepper_max_itr_m_;

    //** Alg1:L2: Determine relevant matrix and vector sizes based on task_offset_tau
    nof_pjoints_ = slist.cols() - task_offset_tau;
    nof_sjoints_ = task_offset_tau;

    //**Alg1:L3 and Alg1:L2: Set start guesses and step goal
    Eigen::VectorXd theta_sg = Eigen::VectorXd::Zero(nof_sjoints_);
    Eigen::VectorXd theta_pg = Eigen::VectorXd::Zero(nof_pjoints_);
    Eigen::VectorXd theta_sd = theta_sdf; // We set the affordance goal in the loop in reference to the start state
    theta_sd.tail(1).setConstant(start_guess_); // start affordance at 0 but gripper orientation as specified

    //**Alg1:L4: Compute no. of iterations, stepper_max_itr_m_ to final goal: Passed in as planner config

    //**Alg1:L5: Initialize loop counter, loop_counter_k; success counter, success_counter_s
    int loop_counter_k = 0;
    int success_counter_s = 0;

    while (loop_counter_k < stepper_max_itr_m_ && !st.stop_requested()) //**Alg1:L6
    {
        loop_counter_k = loop_counter_k + 1; //**Alg1:L7:

        //**Alg1:L8: Update aff step goal:
        // Set the affordance step goal as aff_step away from the current pose. Affordance is the last element of
        // theta_sd
        theta_sd(nof_sjoints_ - 1) = theta_sd(nof_sjoints_ - 1) - deltatheta_a;

        //**Alg1:L13: Call Algorithm 2 with args, theta_sd, theta_pg, theta_sg, slist
        std::optional<Eigen::VectorXd> ik_result = this->call_cc_ik_solver(slist, theta_pg, theta_sg, theta_sd, st);

        if (ik_result.has_value()) //**Alg1:L14
        {
            //**Alg1:L15: Record solution, theta_p, theta_sg
            const Eigen::VectorXd &traj_point = ik_result.value();
            plannerResult.joint_trajectory.push_back(traj_point); // recorded as a point in the trajectory solution

            //**Alg1:L16 Update guesses, theta_pg, theta_sg
            theta_sg = traj_point.tail(nof_sjoints_);
            theta_pg = traj_point.head(nof_pjoints_);

            success_counter_s = success_counter_s + 1; //**Alg1:L17
        }                                              //**Alg1:L18

    } //**Alg1:L19

    // Capture the planning time
    auto end_time = std::chrono::high_resolution_clock::now();
    plannerResult.planning_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    // Set planning result
    if (!plannerResult.joint_trajectory.empty())
    {
        plannerResult.success = true;

        if (loop_counter_k == success_counter_s)
        {
            plannerResult.trajectory_description = TrajectoryDescription::FULL;
        }
        else
        {
            plannerResult.trajectory_description = TrajectoryDescription::PARTIAL;
        }
    }
    else
    {
        plannerResult.success = false;
        plannerResult.trajectory_description = TrajectoryDescription::UNSET;
    }

    // Indicate if DLS was used
    plannerResult.update_trail = dls_flag_ ? "dls and " : plannerResult.update_trail;

    return plannerResult;
}

PlannerResult CcAffordancePlanner::generate_affordance_motion_joint_trajectory(const Eigen::MatrixXd &slist,
                                                                               const Eigen::VectorXd &theta_sdf,
                                                                               const size_t &task_offset_tau)
{

    auto start_time = std::chrono::high_resolution_clock::now(); // Monitor clock to track planning time

    const double theta_adf = theta_sdf.tail(1)(0);
    PlannerResult plannerResult; // Result of the planner

    //**Alg1:L1: Define affordance step, deltatheta_a
    const double deltatheta_a = theta_adf / stepper_max_itr_m_;

    //** Alg1:L2: Determine relevant matrix and vector sizes based on task_offset_tau
    nof_pjoints_ = slist.cols() - task_offset_tau;
    nof_sjoints_ = task_offset_tau;

    //**Alg1:L3 and Alg1:L2: Set start guesses and step goal
    Eigen::VectorXd theta_sg = Eigen::VectorXd::Zero(nof_sjoints_);
    Eigen::VectorXd theta_pg = Eigen::VectorXd::Zero(nof_pjoints_);
    Eigen::VectorXd theta_sd = theta_sdf; // We set the affordance goal in the loop in reference to the start state
    theta_sd.tail(1).setConstant(start_guess_); // start affordance at 0 but gripper orientation as specified

    //**Alg1:L4: Compute no. of iterations, stepper_max_itr_m_ to final goal: Passed in as planner config

    //**Alg1:L5: Initialize loop counter, loop_counter_k; success counter, success_counter_s
    int loop_counter_k = 0;
    int success_counter_s = 0;

    while (loop_counter_k < stepper_max_itr_m_) //**Alg1:L6
    {

        loop_counter_k = loop_counter_k + 1; //**Alg1:L7:

        //**Alg1:L8: Update aff step goal:
        // Set the affordance step goal as aff_step away from the current pose. Affordance is the last element of
        // theta_sd
        theta_sd(nof_sjoints_ - 1) = theta_sd(nof_sjoints_ - 1) - deltatheta_a;

        //**Alg1:L13: Call Algorithm 2 with args, theta_sd, theta_pg, theta_sg, slist
        std::optional<Eigen::VectorXd> ik_result = this->call_cc_ik_solver(slist, theta_pg, theta_sg, theta_sd);

        if (ik_result.has_value()) //**Alg1:L14
        {
            //**Alg1:L15: Record solution, theta_p, theta_sg
            const Eigen::VectorXd &traj_point = ik_result.value();
            plannerResult.joint_trajectory.push_back(traj_point); // recorded as a point in the trajectory solution

            //**Alg1:L16 Update guesses, theta_pg, theta_sg
            theta_sg = traj_point.tail(nof_sjoints_);
            theta_pg = traj_point.head(nof_pjoints_);

            success_counter_s = success_counter_s + 1; //**Alg1:L17
        }                                              //**Alg1:L18

    } //**Alg1:L19

    // Capture the planning time
    auto end_time = std::chrono::high_resolution_clock::now();
    plannerResult.planning_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    // Set planning result
    if (!plannerResult.joint_trajectory.empty())
    {
        plannerResult.success = true;

        if (loop_counter_k == success_counter_s)
        {
            plannerResult.trajectory_description = TrajectoryDescription::FULL;
        }
        else
        {
            plannerResult.trajectory_description = TrajectoryDescription::PARTIAL;
        }
    }
    else
    {
        plannerResult.success = false;
        plannerResult.trajectory_description = TrajectoryDescription::UNSET;
    }

    // Indicate if DLS was used
    plannerResult.update_trail = dls_flag_ ? "dls and " : plannerResult.update_trail;

    return plannerResult;
}

std::optional<Eigen::VectorXd> CcAffordancePlanner::call_cc_ik_solver(const Eigen::MatrixXd &slist,
                                                                      const Eigen::VectorXd &theta_pg,
                                                                      const Eigen::VectorXd &theta_sg,
                                                                      const Eigen::VectorXd &theta_sd,
                                                                      std::stop_token st)
{

    /* Eigen resizings */
    Eigen::VectorXd thetalist; // helper variable holding theta_p, theta_s
    thetalist.conservativeResize(slist.cols());

    //**Alg2:L1: Set max. no. of iterations, max_itr_l_, and error thresholds, p_task_err_threshold_eps_s,
    // eps_r_: Defined as class public variables

    //** Alg2:L2: Set dt as small time increment
    const double dt = 1e-2; // time step to compute joint velocities

    // Alg2:L3: Capture guesses
    Eigen::VectorXd theta_p = theta_pg;
    Eigen::VectorXd theta_s = theta_sg;

    Eigen::VectorXd oldtheta_p =
        Eigen::VectorXd::Zero(nof_pjoints_); // capture theta_p to compute joint velocities below for Alg2:L8

    //**Alg2:L4: Initialize loop counter, loop_counter_i
    int loop_counter_i = 0;

    //**Alg2:L5: Start closure error at 0
    Eigen::Matrix<double, twist_length_, 1> rho = Eigen::VectorXd::Zero(twist_length_); // twist length is 6

    // Compute error
    const Eigen::VectorXd theta_s_tol =
        accuracy_ * theta_sd.cwiseAbs();                           // elementwise tolerance for secondary joint goal
    Eigen::VectorXd theta_s_err = (theta_sd - theta_s).cwiseAbs(); // secondary joint goal error

    bool err = ((theta_s_err.array() > theta_s_tol.array()).any() || rho.head(3).norm() > eps_rw_ ||
                rho.tail(3).norm() > eps_rv_);

    while (err && loop_counter_i < max_itr_l_ && !st.stop_requested()) //**Alg2:L6
    {
        loop_counter_i = loop_counter_i + 1; //**Alg2:L7

        //**Alg2:L8: Compute Np, Ns as screw-based Jacobians
        thetalist << theta_p, theta_s;
        Eigen::MatrixXd jac = affordance_util::JacobianSpace(slist, thetalist);
        Eigen::MatrixXd Np = jac.leftCols(nof_pjoints_);
        Eigen::MatrixXd Ns = jac.rightCols(nof_sjoints_);

        //**Alg2:L9: Compute theta_pdot using Eqn. 23
        Eigen::MatrixXd theta_pdot = (theta_p - oldtheta_p) / dt;

        //**Alg2:L10: Compute N using Eqn. 22
        Eigen::MatrixXd pinv_Ns; // pseudo-inverse of Ns
        pinv_Ns = Ns.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd pinv_theta_pdot; // pseudo-inverse of theta_pdot
        pinv_theta_pdot = theta_pdot.completeOrthogonalDecomposition().pseudoInverse();

        Eigen::MatrixXd N = -pinv_Ns * (Np + rho * pinv_theta_pdot);

        oldtheta_p = theta_p; // capture theta_p to compute joint velocities below for Alg2:L8

        // Update theta_p using Newton-Raphson
        this->update_theta_p(theta_p, theta_sd, theta_s, N); // returned by reference

        //**Alg2:L12: Call Algorithm 3 with args, theta_s, theta_p, slist, Np, Ns
        this->adjust_for_closure_error(slist, Np, Ns, theta_p,
                                       theta_s); // theta_s and theta_p returned by reference

        // Check error
        theta_s_err = (theta_sd - theta_s).cwiseAbs(); // secondary joint goal error
        err = ((theta_s_err.array() > theta_s_tol.array()).any() || rho.head(3).norm() > eps_rw_ ||
               rho.tail(3).norm() > eps_rv_);

    } //**Alg2:L13

    if (!err) //**Alg2:L14
    {
        thetalist << theta_p, theta_s; // return thetalist corrected by closure_error_optimizer
        return thetalist;              //** Alg2:L15
    }
    else
    {
        return std::nullopt; //** Alg2:L15 // Represents no value (similar to nullptr for pointers)
    }
}

std::optional<Eigen::VectorXd> CcAffordancePlanner::call_cc_ik_solver(const Eigen::MatrixXd &slist,
                                                                      const Eigen::VectorXd &theta_pg,
                                                                      const Eigen::VectorXd &theta_sg,
                                                                      const Eigen::VectorXd &theta_sd)
{

    /* Eigen resizings */
    Eigen::VectorXd thetalist; // helper variable holding theta_p, theta_s
    thetalist.conservativeResize(slist.cols());

    //**Alg2:L1: Set max. no. of iterations, max_itr_l_, and error thresholds, p_task_err_threshold_eps_s,
    // eps_r_: Defined as class public variables

    //** Alg2:L2: Set dt as small time increment
    const double dt = 1e-2; // time step to compute joint velocities

    // Alg2:L3: Capture guesses
    Eigen::VectorXd theta_p = theta_pg;
    Eigen::VectorXd theta_s = theta_sg;

    Eigen::VectorXd oldtheta_p =
        Eigen::VectorXd::Zero(nof_pjoints_); // capture theta_p to compute joint velocities below for Alg2:L8

    //**Alg2:L4: Initialize loop counter, loop_counter_i
    int loop_counter_i = 0;

    //**Alg2:L5: Start closure error at 0
    Eigen::Matrix<double, twist_length_, 1> rho = Eigen::VectorXd::Zero(twist_length_); // twist length is 6

    // Compute error
    const Eigen::VectorXd theta_s_tol =
        accuracy_ * theta_sd.cwiseAbs();                           // elementwise tolerance for secondary joint goal
    Eigen::VectorXd theta_s_err = (theta_sd - theta_s).cwiseAbs(); // secondary joint goal error

    bool err = ((theta_s_err.array() > theta_s_tol.array()).any() || rho.head(3).norm() > eps_rw_ ||
                rho.tail(3).norm() > eps_rv_);

    while (err && loop_counter_i < max_itr_l_) //**Alg2:L6
    {
        loop_counter_i = loop_counter_i + 1; //**Alg2:L7

        //**Alg2:L8: Compute Np, Ns as screw-based Jacobians
        thetalist << theta_p, theta_s;
        Eigen::MatrixXd jac = affordance_util::JacobianSpace(slist, thetalist);
        Eigen::MatrixXd Np = jac.leftCols(nof_pjoints_);
        Eigen::MatrixXd Ns = jac.rightCols(nof_sjoints_);

        //**Alg2:L9: Compute theta_pdot using Eqn. 23
        Eigen::MatrixXd theta_pdot = (theta_p - oldtheta_p) / dt;

        //**Alg2:L10: Compute N using Eqn. 22
        Eigen::MatrixXd pinv_Ns; // pseudo-inverse of Ns
        pinv_Ns = Ns.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd pinv_theta_pdot; // pseudo-inverse of theta_pdot
        pinv_theta_pdot = theta_pdot.completeOrthogonalDecomposition().pseudoInverse();

        Eigen::MatrixXd N = -pinv_Ns * (Np + rho * pinv_theta_pdot);

        oldtheta_p = theta_p; // capture theta_p to compute joint velocities below for Alg2:L8

        // Update theta_p using Newton-Raphson
        this->update_theta_p(theta_p, theta_sd, theta_s, N); // returned by reference

        //**Alg2:L12: Call Algorithm 3 with args, theta_s, theta_p, slist, Np, Ns
        this->adjust_for_closure_error(slist, Np, Ns, theta_p,
                                       theta_s); // theta_s and theta_p returned by reference

        // Check error
        theta_s_err = (theta_sd - theta_s).cwiseAbs(); // secondary joint goal error
        err = ((theta_s_err.array() > theta_s_tol.array()).any() || rho.head(3).norm() > eps_rw_ ||
               rho.tail(3).norm() > eps_rv_);

    } //**Alg2:L13

    if (!err) //**Alg2:L14
    {
        thetalist << theta_p, theta_s; // return thetalist corrected by closure_error_optimizer
        return thetalist;              //** Alg2:L15
    }
    else
    {
        return std::nullopt; //** Alg2:L15 // Represents no value (similar to nullptr for pointers)
    }
}

void CcAffordancePlanner::adjust_for_closure_error(
    const Eigen::MatrixXd &slist, const Eigen::MatrixXd &Np, const Eigen::MatrixXd &Ns, Eigen::VectorXd &theta_p,
    Eigen::VectorXd &theta_s) //**Alg3:L5 // theta_s and theta_p returned by reference
{

    /* Eigen resizings */
    Eigen::VectorXd thetalist;
    thetalist.conservativeResize(slist.cols()); // helper variable holding theta_p, theta_s

    //**Alg3:L1: Compute forward kinematics to chain's end link, Tse
    const Eigen::Matrix4d des_endlink_htm_ = Eigen::Matrix4d::Identity(); // Desired HTM for the end link
    thetalist << theta_p, theta_s;
    Eigen::Matrix4d Tse =
        affordance_util::FKinSpace(des_endlink_htm_, slist, thetalist); // HTM of actual end of ground link

    //**Alg3:L2: Compute closure error
    Eigen::Matrix<double, twist_length_, 1> rho =
        affordance_util::Adjoint(Tse) *
        affordance_util::se3ToVec(affordance_util::MatrixLog6(affordance_util::TransInv(Tse)));

    //**Alg3:L3: Adjust joint angles for closure error
    Eigen::MatrixXd Nc(Np.rows(), Np.cols() + Ns.cols());
    Nc << Np, Ns; // Combine Np and Ns horizontally
    Eigen::MatrixXd pinv_Nc;
    pinv_Nc = Nc.completeOrthogonalDecomposition().pseudoInverse(); // pseudo-inverse of N
    const Eigen::VectorXd delta_theta = pinv_Nc * rho;              // correction differential

    // Correct the joint angles
    theta_p = theta_p + delta_theta.head(nof_pjoints_);
    theta_s = theta_s + delta_theta.tail(nof_sjoints_);

    // Compute final error
    thetalist << theta_p, theta_s;
    Tse = affordance_util::FKinSpace(des_endlink_htm_, slist, thetalist);
    rho = affordance_util::Adjoint(Tse) *
          affordance_util::se3ToVec(affordance_util::MatrixLog6(affordance_util::TransInv(Tse)));
}

} // namespace cc_affordance_planner
