#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cc_affordance_planner/cc_affordance_planner.hpp>

// This tester includes hardcoded UR5 robot info to test the CC Affordance
// planner
/*
   Author: Crasun Jans
*/
int main()
{

    //** Geometry
    const double mconv = 1000.0; // conversion factor from mm to m
    const double W1 = 109.0 / mconv, W2 = 82.0 / mconv, L1 = 425.0 / mconv, L2 = 392.0 / mconv, H1 = 89.0 / mconv,
                 H2 = 95.0 / mconv, W3 = 135.85 / mconv, W4 = 119.7 / mconv,
                 W6 = 93.0 / mconv;    // Link lengths
    const double aff = -100.0 / mconv; // affordance location from the last joint

    const size_t nof_joints = 6, nof_virtual_joints = 3,
                 nof_affordance = 1; // number of joints
    const size_t nof_joints_total = nof_joints + nof_virtual_joints + nof_affordance;

    //** Creation of screw axes
    // Screw axis locations from base frame in home position
    Eigen::MatrixXd q(3, nof_joints_total);
    q.col(0) << 0, 0, H1;
    q.col(1) << 0, W3, H1;
    q.col(2) << L1, W3 - W4, H1;
    q.col(3) << L1 + L2, W3 - W4, H1;
    q.col(4) << L1 + L2, W3 - W4 + W6, H1;
    q.col(5) << L1 + L2, W3 - W4 + W6, H1 - H2;
    q.col(6) << L1 + L2, W3 - W4 + W6 + W2, H1 - H2; // Imaginary joint
    q.col(7) << L1 + L2, W3 - W4 + W6 + W2, H1 - H2; // Imaginary joint
    q.col(8) << L1 + L2, W3 - W4 + W6 + W2, H1 - H2; // Imaginary joint
    q.col(9) << L1 + L2, W3 - W4 + W6 + W2 + aff,
        H1 - H2; // Location of affordance frame

    // Type and alignment of screw axes
    Eigen::MatrixXd w(3, 10);
    w.col(0) << 0, 0, 1;
    w.col(1) << 0, 1, 0;
    w.col(2) << 0, 1, 0;
    w.col(3) << 0, 1, 0;
    w.col(4) << 0, 0, -1;
    w.col(5) << 0, 1, 0;
    w.col(6) << 1, 0, 0; // Imaginary joint
    w.col(7) << 0, 1, 0; // Imaginary joint
    w.col(8) << 0, 0, 1; // Imaginary joint
    w.col(9) << 1, 0, 0; // Affordance

    Eigen::MatrixXd slist(6, nof_joints_total);

    // Construct screw axes and frames
    for (int i = 0; i < w.cols(); i++)
    {
        Eigen::Vector3d wcurr = w.col(i); // required to convert to VectorXd type for use with cross
        Eigen::Vector3d qcurr = q.col(i); // required to convert to VectorXd type for use with cross
        slist.col(i) << wcurr, -wcurr.cross(qcurr);
    }

    // Output screw axes for debugging purposes
    /* std::cout << "Here is the list of screws: \n" << slist << std::endl; */
    /* ; */

    // Start angles
    const double aff_goal = 0.3;

    // Construct the planner object
    CcAffordancePlanner ccAffordancePlanner;

    // Set algorithm parameters
    ccAffordancePlanner.p_aff_step_deltatheta_a = 0.1;
    ccAffordancePlanner.p_task_err_threshold_eps_s = 0.001;
    const int task_offset = 1;

    // Run the planner
    PlannerResult plannerResult = ccAffordancePlanner.affordance_stepper(slist, aff_goal, task_offset);

    // Print the first point in the trajectory if planner succeeds and display the Matlab solution as well
    if (plannerResult.success)
    {
        std::vector<Eigen::VectorXd> solution = plannerResult.joint_traj;
        std::cout << "Planner succeeded with " << plannerResult.traj_full_or_partial
                  << " solution, and here is the first point in the trajectory: \n"
                  << solution.at(0) << std::endl;
        std::cout << "The entire planning took " << plannerResult.planning_time.count() << " microseconds."
                  << std::endl;
        Eigen::Matrix<double, 10, 1> matlab_solution;
        matlab_solution << -0.0006, 0.0118, 0.0008, -0.0093, 0.0031, -0.0017, -0.0994, -0.0019, 0.0036, 0.0994;
        std::cout << "The first point of the solution from Matlab for the same setup, i.e. UR5 pure rotation with "
                     "affordance step 0.1, "
                     "and accuracy 1% (or "
                     "0.001) is as follows: \n"
                  << matlab_solution << std::endl;

        // Check if the planner and matlab solution are equal upto 3 decimal places
        bool are_equal = solution.at(0).isApprox(matlab_solution, 1e-3);
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
