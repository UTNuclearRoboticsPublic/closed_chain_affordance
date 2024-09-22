#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
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
    aff.type = affordance_util::ScrewType::ROTATION;
    aff.axis = Eigen::Vector3d(1, 0, 0);
    aff.location = Eigen::Vector3d(L1 + L2, W3 - W4 + W6 + W2 + aff_offset, H1 - H2);

    // Robot start configuration, home position in this case
    Eigen::VectorXd thetalist(6);
    thetalist = Eigen::VectorXd::Zero(6);

    // Output robot screws for debugging purposes
    /* std::cout << "Here is the list of screws: \n" << robot_slist << std::endl; */

    // Configure the planner
    cc_affordance_planner::PlannerConfig plannerConfig;
    plannerConfig.accuracy = 0.01;
    plannerConfig.update_method = cc_affordance_planner::UpdateMethod::INVERSE;
    /* plannerConfig.closure_err_threshold_ang = 1e-4; */
    /* plannerConfig.closure_err_threshold_lin = 1e-5; */
    /* plannerConfig.ik_max_itr = 200; // Default */

    // Robot description
    affordance_util::RobotDescription robot_description;
    robot_description.slist = robot_slist;
    robot_description.M = M;
    robot_description.joint_states = thetalist;
    robot_description.gripper_state = 0;

    // Task description
    cc_affordance_planner::TaskDescription task_description;
    task_description.affordance_info = aff;
    task_description.trajectory_density = 5; // with 0.4 aff_goal, we get 0.1 aff step as MATLAB
    /* task_description.motion_type = cc_affordance_planner::MotionType::AFFORDANCE; // Default */
    task_description.gripper_goal_type = affordance_util::GripperGoalType::CONTINUOUS;
    task_description.goal.affordance = 0.4;
    task_description.goal.gripper = 0.4;

    // Run the planner
    // Construct the planner interface object
    cc_affordance_planner::CcAffordancePlannerInterface ccAffordancePlannerInterface(plannerConfig);
    cc_affordance_planner::PlannerResult plannerResult;

    try
    {
        plannerResult = ccAffordancePlannerInterface.generate_joint_trajectory(robot_description, task_description);
    }
    catch (const std::invalid_argument &e)
    {
        std::cerr << "Planner returned exception: " << e.what() << std::endl;
    }

    // Print the first point in the trajectory if planner succeeds and display the Matlab solution as well
    if (plannerResult.success)
    {
        std::vector<Eigen::VectorXd> solution = plannerResult.joint_trajectory;
        std::cout << "Planner succeeded with update trail '" << plannerResult.update_trail
                  << "', and here is the first point in the trajectory: \n"
                  << solution.at(1) << std::endl; // Look at the second point since the first one is 0 in this case cuz
                                                  // we started at home position
        std::cout << "The entire planning took " << plannerResult.planning_time.count() << " microseconds\n";
        Eigen::VectorXd matlab_solution(10); // Including affordance
        matlab_solution << -0.0006, 0.0118, 0.0008, -0.0093, 0.0031, -0.0017, -0.0994, -0.0019, 0.0036,
            -0.0994; // including affordance
        std::cout << "The first point of the solution from Matlab for the same setup, i.e. UR5 pure rotation with "
                     "affordance step 0.1, "
                     "and accuracy 1% (or "
                     "0.001) is as follows: \n"
                  << matlab_solution << std::endl;
        const Eigen::VectorXd solution_comp_point =
            (Eigen::VectorXd(nof_joints_total) << solution.at(1).head(nof_joints),
             solution.at(1).tail(nof_virtual_joints + nof_affordance))
                .finished(); // remove the gripper joint for comparison

        // Check if the planner and matlab solution are equal upto 3 decimal places
        bool are_equal = solution_comp_point.isApprox(matlab_solution,
                                                      1e-3); // Just compare solution, excluding affordance
        if (are_equal)
        {
            std::cout << "The planner solution first point matches the one from Matlab." << std::endl;
        }
        else
        {

            std::cerr << "The planner solution does not match Matlab solution." << std::endl;
        }
    }
    else
    {
        std::cerr << "Planner did not find a solution." << std::endl;
    }
    return 0;
}
