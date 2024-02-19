#include <cc_affordance_planner/cc_affordance_planner.hpp>

CcAffordancePlanner::CcAffordancePlanner() {}
PlannerResult CcAffordancePlanner::affordance_stepper(const Eigen::MatrixXd &slist, const double &theta_adf,
                                                      const size_t &task_offset_tau)
{

    PlannerResult plannerResult; // Result of the planner

    auto start_time = std::chrono::high_resolution_clock::now(); // Monitor clock to track planning time

    //**Alg1:L1: Define affordance step, p_aff_step_deltatheta_a : Defined as class public variable

    //** Alg1:L2: Determine relevant matrix and vector sizes based on task_offset_tau
    nof_pjoints_ = slist.cols() - task_offset_tau;
    nof_sjoints_ = task_offset_tau;

    //**Alg1:L3 and Alg1:L2: Set start guesses and step goal
    Eigen::VectorXd theta_sg = Eigen::VectorXd::Zero(nof_sjoints_);
    Eigen::VectorXd theta_pg = Eigen::VectorXd::Zero(nof_pjoints_);
    Eigen::VectorXd theta_sd = theta_sg; // We set the affordance goal in the loop in reference to the start state

    //**Alg1:L4: Compute no. of iterations, stepper_max_itr_m to final goal, theta_adf
    const int stepper_max_itr_m = theta_adf / p_aff_step_deltatheta_a + 1;

    //**Alg1:L5: Initialize loop counter, loop_counter_k; success counter, success_counter_s
    int loop_counter_k = 0;
    int success_counter_s = 0;

    while (loop_counter_k < stepper_max_itr_m) //**Alg1:L6
    {

        loop_counter_k = loop_counter_k + 1; //**Alg1:L7:

        //**Alg1:L8: Update aff step goal:
        // If last iteration, adjust affordance step accordingly
        if (loop_counter_k == (stepper_max_itr_m)) //**Alg1:L9
        {
            p_aff_step_deltatheta_a = theta_adf - p_aff_step_deltatheta_a * (stepper_max_itr_m - 1); //**Alg1:L10
        }                                                                                            //**Alg1:L11

        // Set the affordance step goal as aff_step away from the current pose. Affordance is the last element of
        // theta_sd
        theta_sd(nof_sjoints_ - 1) = theta_sd(nof_sjoints_ - 1) + p_aff_step_deltatheta_a; //**Alg1:L12

        //**Alg1:L13: Call Algorithm 2 with args, theta_sd, theta_pg, theta_sg, slist
        std::optional<Eigen::VectorXd> ik_result =
            CcAffordancePlanner::cc_ik_solver(slist, theta_pg, theta_sg, theta_sd);

        if (ik_result.has_value()) //**Alg1:L14
        {
            //**Alg1:L15: Record solution, theta_p, theta_sg
            const Eigen::VectorXd &traj_point = ik_result.value();
            plannerResult.joint_traj.push_back(traj_point); // recorded as a point in the trajectory solution

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
    if (!plannerResult.joint_traj.empty())
    {
        plannerResult.success = true;

        if (loop_counter_k == success_counter_s)
        {
            plannerResult.traj_full_or_partial = "Full";
        }
        else
        {
            plannerResult.traj_full_or_partial = "Partial";
        }
    }
    else
    {
        plannerResult.success = false;
        plannerResult.traj_full_or_partial = "Unset";
    }

    return plannerResult;
}

std::optional<Eigen::VectorXd> CcAffordancePlanner::cc_ik_solver(const Eigen::MatrixXd &slist,
                                                                 const Eigen::VectorXd &theta_pg,
                                                                 const Eigen::VectorXd &theta_sg,
                                                                 const Eigen::VectorXd &theta_sd)
{

    /* Eigen resizings */
    Eigen::VectorXd thetalist; // helper variable holding theta_p, theta_s
    thetalist.conservativeResize(slist.cols());

    //**Alg2:L1: Set max. no. of iterations, p_max_itr_l, and error thresholds, p_task_err_threshold_eps_s,
    // p_closure_err_threshold_eps_r: Defined as class public variables

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
    bool err =
        (((theta_sd - theta_s).norm() > p_task_err_threshold_eps_s) || rho.norm() > p_closure_err_threshold_eps_r);

    while (err && loop_counter_i < p_max_itr_l) //**Alg2:L6
    {
        loop_counter_i = loop_counter_i + 1; //**Alg2:L7

        //**Alg2:L8: Compute Np, Ns as screw-based Jacobians
        thetalist << theta_p, theta_s;
        Eigen::MatrixXd jac = AffordanceUtil::JacobianSpace(slist, thetalist);
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

        //**Alg2:L11: Update theta_p using Eqn. 24
        Eigen::MatrixXd pinv_N;
        pinv_N = N.completeOrthogonalDecomposition().pseudoInverse(); // pseudo-inverse of N

        theta_p = theta_p + pinv_N * (theta_sd - theta_s); // Update using Newton-Raphson

        //**Alg2:L12: Call Algorithm 3 with args, theta_s, theta_p, slist, Np, Ns
        CcAffordancePlanner::closure_error_optimizer(slist, Np, Ns, theta_p,
                                                     theta_s); // theta_s and theta_p returned by reference

        // Check error
        err =
            (((theta_sd - theta_s).norm() > p_task_err_threshold_eps_s) || rho.norm() > p_closure_err_threshold_eps_r);

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

void CcAffordancePlanner::closure_error_optimizer(
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
        AffordanceUtil::FKinSpace(des_endlink_htm_, slist, thetalist); // HTM of actual end of ground link

    //**Alg3:L2: Compute closure error
    Eigen::Matrix<double, twist_length_, 1> rho =
        AffordanceUtil::Adjoint(Tse) *
        AffordanceUtil::se3ToVec(AffordanceUtil::MatrixLog6(AffordanceUtil::TransInv(Tse)));

    //**Alg3:L3: Adjust joint angles for closure error
    Eigen::MatrixXd Nc(Np.rows(), Np.cols() + Ns.cols());
    Nc << Np, Ns; // Combine Np and Ns horizontally
    Eigen::MatrixXd pinv_Nc;
    pinv_Nc = Nc.completeOrthogonalDecomposition().pseudoInverse(); // pseudo-inverse of N
    const Eigen::VectorXd delta_theta = pinv_Nc * rho;              // correction differential

    theta_p = theta_p + delta_theta.head(nof_pjoints_);
    theta_s = theta_s + delta_theta.tail(nof_sjoints_);

    // Compute final error
    thetalist << theta_p, theta_s;
    Tse = AffordanceUtil::FKinSpace(des_endlink_htm_, slist, thetalist);
    rho = AffordanceUtil::Adjoint(Tse) *
          AffordanceUtil::se3ToVec(AffordanceUtil::MatrixLog6(AffordanceUtil::TransInv(Tse)));
}
