#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <iomanip>

// This tester includes hardcoded UR5 robot info to test the CC Affordance
// planner
/*
   Author: Crasun Jans
*/
int main()
{
    // Precision for testing
    std::cout << std::fixed << std::setprecision(4); // Display up to 4 decimal places

    //** Geometry
    const double mconv = 1000.0; // conversion factor from mm to m
    const double W1 = 109.0 / mconv, W2 = 82.0 / mconv, L1 = 425.0 / mconv, L2 = 392.0 / mconv, H1 = 89.0 / mconv,
                 H2 = 95.0 / mconv, W3 = 135.85 / mconv, W4 = 119.7 / mconv,
                 W6 = 93.0 / mconv;           // Link lengths
    const double aff_offset = -100.0 / mconv; // affordance location from the last joint

    const size_t nof_joints = 6, nof_virtual_joints = 3,
                 nof_affordance = 1; // number of joints
    const size_t nof_joints_total = nof_joints + nof_virtual_joints + nof_affordance;

    //** Creation of screw axes
    // Screw axis locations from base frame in home position
    Eigen::MatrixXd q(3, nof_joints);
    q.col(0) << 0, 0, H1;
    q.col(1) << 0, W3, H1;
    q.col(2) << L1, W3 - W4, H1;
    q.col(3) << L1 + L2, W3 - W4, H1;
    q.col(4) << L1 + L2, W3 - W4 + W6, H1;
    q.col(5) << L1 + L2, W3 - W4 + W6, H1 - H2;

    // Type and alignment of screw axes
    Eigen::MatrixXd w(3, nof_joints);
    w.col(0) << 0, 0, 1;
    w.col(1) << 0, 1, 0;
    w.col(2) << 0, 1, 0;
    w.col(3) << 0, 1, 0;
    w.col(4) << 0, 0, -1;
    w.col(5) << 0, 1, 0;

    Eigen::MatrixXd robot_slist(6, nof_joints);

    // Construct screw axes and frames
    for (int i = 0; i < w.cols(); i++)
    {
        Eigen::Vector3d wcurr = w.col(i); // required to convert to VectorXd type for use with cross
        Eigen::Vector3d qcurr = q.col(i); // required to convert to VectorXd type for use with cross
        robot_slist.col(i) << wcurr, -wcurr.cross(qcurr);
    }

    // Matrix representing EE transformation
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M.block<3, 1>(0, 3) << L1 + L2, W3 - W4 + W6 + W2, H1 - H2;

    // Affordance
    affordance_util::ScrewInfo aff;
    aff.type = "rotation";
    aff.axis = Eigen::Vector3d(1, 0, 0);
    aff.location = Eigen::Vector3d(L1 + L2, W3 - W4 + W6 + W2 + aff_offset, H1 - H2);

    // Robot start configuration, home position in this case
    Eigen::VectorXd thetalist(6);
    thetalist = Eigen::VectorXd::Zero(6);

    // Closed-chain screws
    Eigen::MatrixXd cc_slist = affordance_util::compose_cc_model_slist(robot_slist, thetalist, M, aff);

    // Output screw axes for debugging purposes
    /* std::cout << "Here is the list of screws: \n" << cc_slist << std::endl; */

    // Start angles
    const double aff_goal = 0.35;

    // Configure the planner
    cc_affordance_planner::PlannerConfig plannerConfig;
    plannerConfig.trajectory_density = 4;
    /* plannerConfig.aff_step = 0.1; */
    plannerConfig.accuracy = 0.01;
    plannerConfig.update_method = cc_affordance_planner::UpdateMethod::INVERSE;
    /* plannerConfig.motion_type = cc_affordance_planner::MotionType::AFFORDANCE; */
    /* plannerConfig.closure_err_threshold_ang = 1e-5; */
    /* plannerConfig.closure_err_threshold_lin = 1e-6; */
    /* plannerConfig.ik_max_itr = 200; */

    // Set algorithm parameters
    const int task_offset = 1;
    Eigen::VectorXd sec_goal(1);
    sec_goal.tail(1).setConstant(aff_goal);

    // Run the planner
    cc_affordance_planner::PlannerResult plannerResult =
        cc_affordance_planner::generate_joint_trajectory(plannerConfig, cc_slist, sec_goal, task_offset);

    // Print the first point in the trajectory if planner succeeds and display the Matlab solution as well
    if (plannerResult.success)
    {
        std::vector<Eigen::VectorXd> solution = plannerResult.joint_trajectory;
        std::cout << "Planner succeeded with update trail '" << plannerResult.update_trail
                  << "', and here is the first point in the trajectory: \n"
                  << solution.at(0) << std::endl;
        std::cout << "The entire planning took " << plannerResult.planning_time.count() << " microseconds\n";
        Eigen::VectorXd matlab_solution(9); // Excluding affordance
        /* matlab_solution << -0.0006, 0.0118, 0.0008, -0.0093, 0.0031, -0.0017, -0.0994, -0.0019, 0.0036, */
        /*     0.0994; // including affordance */
        matlab_solution << -0.0006, 0.0118, 0.0008, -0.0093, 0.0031, -0.0017, -0.0994, -0.0019, 0.0036;
        std::cout << "The first point of the solution from Matlab for the same setup, i.e. UR5 pure rotation with "
                     "affordance step 0.1, "
                     "and accuracy 1% (or "
                     "0.001) is as follows: \n"
                  << matlab_solution << std::endl;

        // Check if the planner and matlab solution are equal upto 3 decimal places
        bool are_equal =
            solution.at(0).head(9).isApprox(matlab_solution, 1e-3); // Just compare solution, excluding affordance
        if (are_equal)
        {
            std::cout << "The planner solution first point matches the one from Matlab." << std::endl;
        }
        else
        {

            std::cout << "The planner solution does not match Matlab solution." << std::endl;
        }
    }
    else
    {
        std::cout << "Planner did not find a solution." << std::endl;
    }
    return 0;
}
