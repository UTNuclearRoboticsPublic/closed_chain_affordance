#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>

namespace cc_affordance_planner
{
CcAffordancePlannerInterface::CcAffordancePlannerInterface(const PlannerConfig &plannerConfig)
    : plannerConfig_(plannerConfig)
{
}

PlannerResult CcAffordancePlannerInterface::generate_joint_trajectory(
    const affordance_util::RobotDescription &robot_description, const TaskDescription &task_description)
{
    // Check the input for any potential errors
    validate_input(robot_description, task_description);

    // Extract task description
    const affordance_util::ScrewInfo aff = task_description.affordance_info;
    const Eigen::VectorXd theta_sdf = task_description.secondary_joint_goals;
    const size_t task_offset_tau = task_description.nof_secondary_joints;
    const affordance_util::VirtualScrewOrder vir_screw_order = task_description.vir_screw_order;

    if (task_description.motion_type == MotionType::APPROACH)
    {
        // Extract additional task description
        const Eigen::Matrix4d grasp_pose = task_description.grasp_pose;

        // Compose the closed-chain model screws and determine the limit for the approach screw
        const affordance_util::CcModel cc_model =
            affordance_util::compose_cc_model_slist(robot_description, aff, grasp_pose, vir_screw_order);

        // Set the secondary joint goals
        Eigen::VectorXd approach_theta_sdf = theta_sdf;
        approach_theta_sdf.tail(2)(0) = cc_model.approach_limit; // approach screw is second to the last

        std::cout << std::fixed << std::setprecision(4); // Display up to 4 decimal places
        std::cout << "Here is the task description" << std::endl;
        std::cout << "Grasp pose: \n" << grasp_pose << std::endl;
        std::cout << "Approach theta sdf: \n" << approach_theta_sdf << std::endl;
        std::cout << "cc_model.slist: \n" << cc_model.slist << std::endl;
        std::cout << "cc_model.slist: \n" << cc_model.slist.block(0, 0, 6, 10) << std::endl;
        std::cout << "affordace: \n" << cc_model.slist.col(10) << std::endl;
        std::cout << "task offset tau: \n" << task_offset_tau << std::endl;

        PlannerResult plannerResult = this->generate_specified_motion_joint_trajectory_(
            &CcAffordancePlanner::generate_approach_motion_joint_trajectory,
            &CcAffordancePlanner::generate_approach_motion_joint_trajectory, cc_model.slist, approach_theta_sdf,
            task_offset_tau);

        std::cout << "Past planner" << task_offset_tau << std::endl;
        std::cout << "Here is robot joint states: " << robot_description.joint_states << std::endl;
        std::cout << "Here is the planner success: " << plannerResult.success << std::endl;
        // Convert the differential closed-chain joint trajectory to robot joint trajectory
        convert_cc_traj_to_robot_traj_(plannerResult.joint_trajectory, robot_description.joint_states);
        std::cout << "Past lambda" << task_offset_tau << std::endl;

        return plannerResult;
    }

    else if (task_description.motion_type == MotionType::AFFORDANCE)
    {
        // Compose the closed-chain model screws
        const Eigen::MatrixXd cc_slist =
            affordance_util::compose_cc_model_slist(robot_description, aff, vir_screw_order);
        std::cout << std::fixed << std::setprecision(4); // Display up to 4 decimal places
        std::cout << "Here is the task description" << std::endl;
        std::cout << "cc_model.slist: \n" << cc_slist.block(0, 0, 6, 10) << std::endl;
        std::cout << "affordace: \n" << cc_slist.col(10) << std::endl;
        std::cout << "task offset tau: \n" << task_offset_tau << std::endl;
        PlannerResult plannerResult = this->generate_specified_motion_joint_trajectory_(
            &CcAffordancePlanner::generate_affordance_motion_joint_trajectory,
            &CcAffordancePlanner::generate_affordance_motion_joint_trajectory, cc_slist, theta_sdf, task_offset_tau);

        std::cout << "Past planner" << task_offset_tau << std::endl;
        std::cout << "Here is robot joint states: " << robot_description.joint_states << std::endl;
        std::cout << "Here is the planner success: " << plannerResult.success << std::endl;
        // Convert the differential closed-chain joint trajectory to robot joint trajectory
        convert_cc_traj_to_robot_traj_(plannerResult.joint_trajectory, robot_description.joint_states);
        std::cout << "Past lambda" << task_offset_tau << std::endl;

        return plannerResult;
    }
    else // task_description.motion_type == MotionType::APPROACH_AND_AFFORDANCE
    {
        // Extract additional task description
        const Eigen::Matrix4d grasp_pose = task_description.grasp_pose;
        const double post_grasp_aff_goal = task_description.post_grasp_affordance_goal;

        // Compose the closed-chain model screws and determine the limit for the approach screw
        const affordance_util::CcModel cc_model =
            affordance_util::compose_cc_model_slist(robot_description, aff, grasp_pose, vir_screw_order);

        // Set the secondary joint goals
        Eigen::VectorXd approach_theta_sdf = theta_sdf;
        approach_theta_sdf.tail(2)(0) = cc_model.approach_limit; // approach screw is second to the last
        PlannerResult approachPlannerResult = this->generate_specified_motion_joint_trajectory_(
            &CcAffordancePlanner::generate_approach_motion_joint_trajectory,
            &CcAffordancePlanner::generate_approach_motion_joint_trajectory, cc_model.slist, approach_theta_sdf,
            task_offset_tau);

        // Convert the differential closed-chain joint trajectory to robot joint trajectory
        if (!approachPlannerResult.success)
        {

            return approachPlannerResult;
        }

        convert_cc_traj_to_robot_traj_(approachPlannerResult.joint_trajectory, robot_description.joint_states);

        // Adjust starting joint states for affordance motion
        affordance_util::RobotDescription affordance_robot_description = robot_description;
        affordance_robot_description.joint_states = approachPlannerResult.joint_trajectory.back();

        // Set goals for affordance motion
        Eigen::VectorXd affordance_theta_sdf = theta_sdf;
        affordance_theta_sdf.tail(1)(0) = post_grasp_aff_goal;

        // Compose the closed-chain model screws
        const Eigen::MatrixXd cc_slist =
            affordance_util::compose_cc_model_slist(affordance_robot_description, aff, vir_screw_order);
        PlannerResult affordancePlannerResult = this->generate_specified_motion_joint_trajectory_(
            &CcAffordancePlanner::generate_affordance_motion_joint_trajectory,
            &CcAffordancePlanner::generate_affordance_motion_joint_trajectory, cc_slist, affordance_theta_sdf,
            task_offset_tau);

        // If affordance motion failed, return approach motion result, which must have succeeded at this point in code
        if (!affordancePlannerResult.success)
        {
            return approachPlannerResult;
        }

        // Convert the differential closed-chain joint trajectory to robot joint trajectory
        convert_cc_traj_to_robot_traj_(affordancePlannerResult.joint_trajectory,
                                       affordance_robot_description.joint_states);

        // Create a combined result
        PlannerResult plannerResult = approachPlannerResult;
        plannerResult.joint_trajectory.insert(plannerResult.joint_trajectory.end(),
                                              affordancePlannerResult.joint_trajectory.begin(),
                                              affordancePlannerResult.joint_trajectory.end());
        plannerResult.planning_time += affordancePlannerResult.planning_time;
        plannerResult.update_trail =
            plannerResult.update_trail + " --> motion transition --> " + affordancePlannerResult.update_trail;

        // If either motion returned a partial trajectory then, the combined result will also say partial
        if (approachPlannerResult.trajectory_description != cc_affordance_planner::TrajectoryDescription::FULL &&
            affordancePlannerResult.trajectory_description != cc_affordance_planner::TrajectoryDescription::FULL)
        {
            plannerResult.trajectory_description = cc_affordance_planner::TrajectoryDescription::PARTIAL;
        }

        return plannerResult;
    }
} // namespace cc_affordance_planner

PlannerResult CcAffordancePlannerInterface::generate_specified_motion_joint_trajectory_(
    const Gsmt &generate_specified_motion_joint_trajectory,
    const Gsmt_st &generate_specified_motion_joint_trajectory_st, const Eigen::MatrixXd &slist,
    const Eigen::VectorXd &theta_sdf, const size_t &task_offset_tau)

{
    PlannerResult transposeResult; // Result from the transpose planner
    PlannerResult inverseResult;   // Result from the inverse planner
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> transpose_result_obtained{false};
    std::atomic<bool> inverse_result_obtained{false};

    // Construct inverse and transpose planner objects
    CcAffordancePlannerInverse ccAffordancePlannerInverse(plannerConfig_);
    CcAffordancePlannerTranspose ccAffordancePlannerTranspose(plannerConfig_);
    CcAffordancePlanner *ccAffordancePlannerInversePtr(&ccAffordancePlannerInverse);
    CcAffordancePlanner *ccAffordancePlannerTransposePtr(&ccAffordancePlannerTranspose);

    // If a specific update method is requested, run the planner using that
    if (plannerConfig_.update_method == UpdateMethod::INVERSE)
    {
        inverseResult = (ccAffordancePlannerInversePtr->*generate_specified_motion_joint_trajectory)(slist, theta_sdf,
                                                                                                     task_offset_tau);
        inverseResult.update_method = UpdateMethod::INVERSE;
        inverseResult.update_trail += "inverse";
        return inverseResult;
    }

    if (plannerConfig_.update_method == UpdateMethod::TRANSPOSE)
    {
        transposeResult = (ccAffordancePlannerTransposePtr->*generate_specified_motion_joint_trajectory)(
            slist, theta_sdf, task_offset_tau);
        transposeResult.update_method = UpdateMethod::TRANSPOSE;
        transposeResult.update_trail += "transpose";
        return transposeResult;
    }

    // If a specific update method is not requested, run the inverse and transpose planners concurrently in separate
    // threads
    std::jthread inverse_thread([&](std::stop_token st) {
        inverseResult = (ccAffordancePlannerInversePtr->*generate_specified_motion_joint_trajectory_st)(
            slist, theta_sdf, task_offset_tau, st);
        {
            std::unique_lock<std::mutex> lock(mtx);
            inverseResult.update_method = UpdateMethod::INVERSE;
            inverseResult.update_trail += "inverse";
            inverse_result_obtained = true;
        }
        cv.notify_all();
    });

    std::jthread transpose_thread([&](std::stop_token st) {
        transposeResult = (ccAffordancePlannerTransposePtr->*generate_specified_motion_joint_trajectory_st)(
            slist, theta_sdf, task_offset_tau, st);
        {
            std::unique_lock<std::mutex> lock(mtx);
            inverseResult.update_method = UpdateMethod::TRANSPOSE;
            transposeResult.update_trail += "transpose";
            transpose_result_obtained = true;
        }
        cv.notify_all();
    });

    // Wake the main thread up when a result is found
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [&inverse_result_obtained, &transpose_result_obtained]() {
            return (inverse_result_obtained.load() || transpose_result_obtained.load());
        });
    }

    // Analyze the result
    if (inverse_result_obtained.load()) // Inverse planner returned first
    {
        if (inverseResult.trajectory_description == TrajectoryDescription::FULL)
        {

            transpose_thread.request_stop();
            return inverseResult;
        }
        else
        // If inverse planner returned partial or no trajectory then, wait for the transpose planner
        {
            transpose_thread.join();
        }
    }
    else // Transpose planner must have returned first
    {
        if (transposeResult.trajectory_description == TrajectoryDescription::FULL)
        {

            inverse_thread.request_stop();
            return transposeResult;
        }
        else
        // If transpose planner returned partial or no trajectory then, wait for the inverse planner
        {
            inverse_thread.join();
        }
    }

    // At this point, both planners have run. Analyze which trajectory is fuller.
    if ((inverseResult.trajectory_description == TrajectoryDescription::PARTIAL) &&
        (transposeResult.trajectory_description == TrajectoryDescription::UNSET))
    {
        inverseResult.update_trail += " --> transpose unset --> inverse partial";
        return inverseResult;
    }
    else if ((inverseResult.trajectory_description == TrajectoryDescription::UNSET) &&
             (transposeResult.trajectory_description == TrajectoryDescription::PARTIAL))
    {
        transposeResult.update_trail += " --> inverse unset --> transpose partial";
        return transposeResult;
    }
    else // both must be partial so, return whichever has a longer trajectory
    {
        inverseResult.update_trail += " --> transpose and inverse partial --> inverse longer traj";
        transposeResult.update_trail += " --> transpose and inverse partial --> transpose longer traj";
        return (inverseResult.joint_trajectory.size() > transposeResult.joint_trajectory.size()) ? inverseResult
                                                                                                 : transposeResult;
    }
}

void CcAffordancePlannerInterface::convert_cc_traj_to_robot_traj_(std::vector<Eigen::VectorXd> &cc_trajectory,
                                                                  const Eigen::VectorXd &start_joint_states)
{
    // Return if the trajectory is empty or start joint states are empty or 0
    if (cc_trajectory.empty() || start_joint_states.squaredNorm() == 0)
    {
        return;
    }
    // Compute the closed-chain trajectory absolute start state
    Eigen::VectorXd cc_start_joint_states = Eigen::VectorXd::Zero(cc_trajectory[0].size());
    cc_start_joint_states.head(start_joint_states.size()) = start_joint_states;

    // convert the differential joint states to absolute states
    for (Eigen::VectorXd &point : cc_trajectory)
    {
        point += cc_start_joint_states;
    }

    cc_trajectory.insert(cc_trajectory.begin(), cc_start_joint_states);
}
void CcAffordancePlannerInterface::validate_input(const affordance_util::RobotDescription &robot_description,
                                                  const TaskDescription &task_description)
{
    /* Validate robot description */
    if (robot_description.slist.size() == 0)
    {
        throw std::invalid_argument("Robot description: 'slist' cannot be empty.");
    }

    if (robot_description.M.hasNaN())
    {
        throw std::invalid_argument("Robot description: 'M' (palm HTM) must be specified.");
    }

    double tolerance = 1e-4;
    if (((!robot_description.M.block<3, 3>(0, 0).isUnitary(tolerance)) ||
         (std::abs(robot_description.M(3, 3) - 1.0) > tolerance) ||
         (!robot_description.M.row(3).head(3).isZero(tolerance))))
    {
        throw std::invalid_argument("Robot description: 'M' is not a valid transformation matrix.");
    }

    if (robot_description.joint_states.size() == 0)
    {
        throw std::invalid_argument("Robot description: 'joint_states' cannot be empty.");
    }

    if (robot_description.slist.cols() != robot_description.joint_states.size())
    {
        throw std::invalid_argument(
            "Robot description: 'joint_states' size must match the number of columns in 'slist'.");
    }

    /* Validate task description */
    if (task_description.affordance_info.type == affordance_util::ScrewType::UNSET)
    {
        throw std::invalid_argument("Task description: 'affordance_info.type' must be specified.");
    }

    if (task_description.affordance_info.axis.hasNaN() ||
        task_description.affordance_info.location.hasNaN() && task_description.affordance_info.screw.hasNaN())
    {
        throw std::invalid_argument("Task description: Either 'affordance_info.axis' and 'affordance_info.location', "
                                    "or 'affordance_info.screw' must be specified.");
    }

    if (task_description.affordance_info.type == affordance_util::ScrewType::SCREW &&
        task_description.affordance_info.screw.hasNaN() && std::isnan(task_description.affordance_info.pitch))
    {
        throw std::invalid_argument("Task description: For 'SCREW' type affordance_info, if screw is not filled out, "
                                    "'pitch' must be specified.");
    }

    if (!task_description.affordance_info.axis.hasNaN() &&
        std::abs(task_description.affordance_info.axis.norm() - 1) > tolerance)
    {
        throw std::invalid_argument("Task description: 'affordance_info.axis' must be a unit vector");
    }

    if (task_description.nof_secondary_joints < 1)
    {
        throw std::invalid_argument("Task description: 'nof_secondary_joints' cannot be less than 1.");
    }

    if (task_description.secondary_joint_goals.size() == 0)
    {
        throw std::invalid_argument("Task description: 'secondary_joint_goals' cannot be empty.");
    }

    if (task_description.nof_secondary_joints != task_description.secondary_joint_goals.size())
    {
        throw std::invalid_argument(
            "Task description: 'nof_secondary_joints' must match the size of 'secondary_joint_goals'.");
    }

    if (((task_description.motion_type == MotionType::APPROACH) ||
         (task_description.motion_type == MotionType::APPROACH_AND_AFFORDANCE)) &&
        ((!task_description.grasp_pose.block<3, 3>(0, 0).isUnitary(tolerance)) ||
         (std::abs(task_description.grasp_pose(3, 3) - 1.0) > tolerance) ||
         (!task_description.grasp_pose.row(3).head(3).isZero(tolerance))))
    {
        throw std::invalid_argument("Task description: 'grasp_pose' is not a valid transformation matrix.");
    }

    if (task_description.motion_type == MotionType::APPROACH_AND_AFFORDANCE &&
        task_description.post_grasp_affordance_goal < 0.0)
    {
        throw std::invalid_argument("Task description: 'post_grasp_affordance_goal' must be non-negative.");
    }
}

} // namespace cc_affordance_planner
