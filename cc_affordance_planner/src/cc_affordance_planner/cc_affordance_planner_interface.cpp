#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>

namespace cc_affordance_planner
{
explicit CcAffordancePlannerInterface(const PlannerConfig &plannerConfig)
    : plannerConfig_(plannerConfig),
      ccAffordancePlannerInverse_(plannerConfig),
      ccAffordancePlannerTranspose_(plannerConfig),
      ccAffordancePlannerInversePtr_(&ccAffordancePlannerInverse_),
      ccAffordancePlannerTransposePtr_(&ccAffordancePlannerTranspose_)
{
}
// Pass approach or affordance args as task description? They are tasks anyways. Also decide whether to pass robot
// starta state as a task description or have it still with robot description? Maybe robot description is better since
// that way you can run the planner with specified settings for any robot. An example is you might wanna run it for the
// arm one time then, in another instant run it for the whole body

PlannerResult CcAffordancePlannerInterface::generate_joint_trajectory(
    const affordance_util::RobotDescription &robot_description, const TaskDescription &task_description)
{
    // Extract task description
    const affordance_util::ScrewInfo aff = task_description.affordance_info;
    const Eigen::VectorXd theta_sdf = task_description.secondary_joint_goals;
    const size_t task_offset_tau = task_description.nof_secondary_joints;
    const affordance_util::VirtualScrewOrder vir_screw_order = task_description.vir_screw_order;

    if (plannerConfig_.motion_type == MotionType::APPROACH)
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

        PlannerResult plannerResult =
            this->generate_xx_motion_joint_trajectory_(&CcAffordancePlanner::generate_approach_motion_joint_trajectory,
                                                       cc_model.slist, approach_theta_sdf, task_offset_tau);

        std::cout << "Past planner" << task_offset_tau << std::endl;
        std::cout << "Here is robot joint states: " << robot_description.joint_states << std::endl;
        std::cout << "Here is the planner success: " << plannerResult.success << std::endl;
        // Convert the differential closed-chain joint trajectory to robot joint trajectory
        if (plannerResult.success)
        {
            convert_cc_traj_to_robot_traj_(plannerResult.joint_trajectory, robot_description.joint_states);
        }
        std::cout << "Past lambda" << task_offset_tau << std::endl;

        return plannerResult;
    }

    else if (plannerConfig_.motion_type == MotionType::AFFORDANCE)
    {
        // Compose the closed-chain model screws
        const Eigen::MatrixXd cc_slist =
            affordance_util::compose_cc_model_slist(robot_description, aff, vir_screw_order);
        PlannerResult plannerResult = this->generate_xx_motion_joint_trajectory_(
            &CcAffordancePlanner::generate_affordance_motion_joint_trajectory, cc_slist, theta_sdf, task_offset_tau);

        // Convert the differential closed-chain joint trajectory to robot joint trajectory
        if (plannerResult.success)
        {
            convert_cc_traj_to_robot_traj_(plannerResult.joint_trajectory, robot_description.joint_states);
        }

        return plannerResult;
    }
    else // plannerConfig_.motion_type == MotionType::APPROACH_AND_AFFORDANCE
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
        PlannerResult approachPlannerResult =
            this->generate_xx_motion_joint_trajectory_(&CcAffordancePlanner::generate_approach_motion_joint_trajectory,
                                                       cc_model.slist, approach_theta_sdf, task_offset_tau);

        // Convert the differential closed-chain joint trajectory to robot joint trajectory
        if (approachPlannerResult.success)
        {
            convert_cc_traj_to_robot_traj_(approachPlannerResult.joint_trajectory, robot_description.joint_states);
        }
        else
        {
            return approachPlannerResult;
        }

        // Adjust starting joint states for affordance motion
        affordance_util::RobotDescription affordance_robot_description = robot_description;
        affordance_robot_description.joint_states = approachPlannerResult.joint_trajectory.back();

        // Set goals for affordance motion
        Eigen::VectorXd affordance_theta_sdf = theta_sdf;
        affordance_theta_sdf.tail(1)(0) = post_grasp_aff_goal;

        // Compose the closed-chain model screws
        const Eigen::MatrixXd cc_slist =
            affordance_util::compose_cc_model_slist(affordance_robot_description, aff, vir_screw_order);
        PlannerResult affordancePlannerResult = this->generate_xx_motion_joint_trajectory_(
            &CcAffordancePlanner::generate_affordance_motion_joint_trajectory, cc_slist, affordance_theta_sdf,
            task_offset_tau);

        // Convert the differential closed-chain joint trajectory to robot joint trajectory
        if (affordancePlannerResult.success)
        {
            convert_cc_traj_to_robot_traj_(affordancePlannerResult.joint_trajectory,
                                           affordance_robot_description.joint_states);
        }
        else
        {
            // If affordance motion failed but approach succeeded, return approach result
            if (approachPlannerResult.success)
            {
                return approachPlannerResult;
            }
            else
            {

                return affordancePlannerResult;
            }
        }

        PlannerResult plannerResult = affordancePlannerResult;
        plannerResult.joint_trajectory.insert(plannerResult.joint_trajectory.end(),
                                              affordancePlannerResult.joint_trajectory.begin(),
                                              affordancePlannerResult.joint_trajectory.end());
        plannerResult.update_trail =
            plannerResult.update_trail + " --> motion transition --> " + affordancePlannerResult.update_trail;

        return plannerResult;
    }
}

PlannerResult CcAffordancePlannerInterface::generate_xx_motion_joint_trajectory_(
    const GenerateXxMotionTrajectory &generate_xx_motion_joint_trajectory, const Eigen::MatrixXd &slist,
    const Eigen::VectorXd &theta_sdf, const size_t &task_offset_tau)

{
    PlannerResult transposeResult; // Result from the transpose planner
    PlannerResult inverseResult;   // Result from the inverse planner
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> transpose_result_obtained{false};
    std::atomic<bool> inverse_result_obtained{false};

    // If a specific update method is requested, run the planner using that
    if (plannerConfig_.update_method == UpdateMethod::INVERSE)
    {
        inverseResult =
            ccAffordancePlannerInversePtr->generate_xx_motion_joint_trajectory(slist, theta_sdf, task_offset_tau);
        inverseResult.update_method = UpdateMethod::INVERSE;
        inverseResult.update_trail += "inverse";
        return inverseResult;
    }

    if (plannerConfig_.update_method == UpdateMethod::TRANSPOSE)
    {
        transposeResult =
            ccAffordancePlannerTransposePtr->generate_xx_motion_joint_trajectory(slist, theta_sdf, task_offset_tau);
        transposeResult.update_method = UpdateMethod::TRANSPOSE;
        transposeResult.update_trail += "transpose";
        return transposeResult;
    }

    // If a specific update method is not requested, run the inverse and transpose planners concurrently in separate
    // threads
    std::jthread inverse_thread([&](std::stop_token st) {
        inverseResult =
            ccAffordancePlannerInversePtr->generate_xx_motion_joint_trajectory(slist, theta_sdf, task_offset_tau, st);
        {
            std::unique_lock<std::mutex> lock(mtx);
            inverseResult.update_method = UpdateMethod::INVERSE;
            inverseResult.update_trail += "inverse";
            inverse_result_obtained = true;
        }
        cv.notify_all();
    });

    std::jthread transpose_thread([&](std::stop_token st) {
        transposeResult =
            ccAffordancePlannerTransposePtr->generate_xx_motion_joint_trajectory(slist, theta_sdf, task_offset_tau, st);
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

void convert_cc_traj_to_robot_traj_(const std::vector<Eigen::VectorXd> &cc_trajectory,
                                    const Eigen::VectorXd &start_joint_states)
{
    // Compute the closed-chain trajectory absolute start state
    Eigen::VectorXd cc_start_joint_states = Eigen::VectorXd::Zero(cc_trajectory[0].size());
    cc_start_joint_states.head(start_joint_states.size()) = start_joint_states;

    // convert the differential joint states to absolute states
    for (Eigen::VectorXd &point : cc_trajectory)
    {
        point += cc_start_joint_states;
    }

    cc_trajectory.insert(cc_trajectory.begin(), cc_start_joint_states);
};

} // namespace cc_affordance_planner
