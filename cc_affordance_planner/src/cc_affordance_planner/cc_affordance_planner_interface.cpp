#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <iomanip> // for std::setprecision and std::fixed

namespace cc_affordance_planner
{

CcAffordancePlannerInterface::CcAffordancePlannerInterface()
    : ccAffordancePlannerInverse_(), ccAffordancePlannerTranspose_()
{
}

CcAffordancePlannerInterface::CcAffordancePlannerInterface(const PlannerConfig &planner_config)
    : planner_config_(planner_config),
      ccAffordancePlannerInverse_(planner_config),
      ccAffordancePlannerTranspose_(planner_config)

{
    // Validate planner config

    if ((planner_config.accuracy <= 0) || (planner_config.accuracy > 1.0))
    {
        throw std::invalid_argument("Planner config: 'accuracy' must be in the range (0,1]");
    }

    if (planner_config.closure_err_threshold_ang < 1e-10)
    {

        throw std::invalid_argument(
            "Planner config: 'closure_err_threshold_ang' cannot be unrealistically small, i.e. less than 1e-10.");
    }

    if (planner_config.closure_err_threshold_lin < 1e-10)
    {

        throw std::invalid_argument(
            "Planner config: 'closure_err_threshold_lin' cannot be unrealistically small, i.e. less than 1e-10.");
    }

    if ((planner_config.ik_max_itr < 2) || (planner_config.accuracy > 10000))
    {

        throw std::invalid_argument("Planner config: 'ik_max_itr' must be in the range [2, 10000]");
    }
}

PlannerResult CcAffordancePlannerInterface::generate_joint_trajectory(
    const affordance_util::RobotDescription &robot_description, const TaskDescription &task_description)
{

    // Check the input for any potential errors
    this->validate_input_(robot_description, task_description);

    // Output of the function
    PlannerResult plannerResult;

    // Extract task description
    const affordance_util::ScrewInfo aff = task_description.affordance_info;
    const affordance_util::VirtualScrewOrder vir_screw_order = task_description.vir_screw_order;

    // Compute the nof secondary joints
    size_t nof_secondary_joints = 1; // secondary joints contain at least the affordance
    nof_secondary_joints += task_description.goal.ee_orientation.size(); // add relevant virtual ee joints
    // TODO: Remove the need to pass nof_secondary_joints to the planner and get it directly from the size of the
    // theta_sdf vector.

    if (task_description.motion_type == MotionType::APPROACH)
    {
        // Extract additional task description
        const Eigen::Matrix4d grasp_pose = task_description.goal.grasp_pose;

        // Compose the closed-chain model screws and determine the limit for the approach screw
        const affordance_util::CcModel cc_model =
            affordance_util::compose_cc_model_slist(robot_description, aff, grasp_pose, vir_screw_order);

        // Extract and construct the secondary joint goals
        nof_secondary_joints += 1; // add approach joint
        const Eigen::VectorXd secondary_joint_goals =
            (Eigen::VectorXd(nof_secondary_joints) << task_description.goal.ee_orientation, cc_model.approach_limit,
             task_description.goal.affordance)
                .finished();

        // Solve the joint trajectory for the given task. This function calls the Cc Affordance Planner inside.
        plannerResult = this->generate_specified_motion_joint_trajectory_(
            &CcAffordancePlanner::generate_approach_motion_joint_trajectory,
            &CcAffordancePlanner::generate_approach_motion_joint_trajectory, cc_model.slist, secondary_joint_goals,
            nof_secondary_joints, task_description.trajectory_density);
    }

    else // task_description.motion_type == MotionType::AFFORDANCE
    {
        // Compose the closed-chain model screws
        const Eigen::MatrixXd cc_slist =
            affordance_util::compose_cc_model_slist(robot_description, aff, vir_screw_order);

        // Extract and construct the secondary joint goals
        const Eigen::VectorXd secondary_joint_goals =
            (Eigen::VectorXd(nof_secondary_joints) << task_description.goal.ee_orientation,
             task_description.goal.affordance)
                .finished();

        // Solve the joint trajectory for the given task. This function calls the Cc Affordance Planner inside.
        plannerResult = this->generate_specified_motion_joint_trajectory_(
            &CcAffordancePlanner::generate_affordance_motion_joint_trajectory,
            &CcAffordancePlanner::generate_affordance_motion_joint_trajectory, cc_slist, secondary_joint_goals,
            nof_secondary_joints, task_description.trajectory_density);
    }

    std::vector<double> gripper_joint_trajectory;

    // Compute gripper trajectory if gripper goal is provided
    if (!std::isnan(task_description.goal.gripper))
    {
        const int trajectory_size = static_cast<int>(plannerResult.joint_trajectory.size()) + 1;
        gripper_joint_trajectory = affordance_util::compute_gripper_joint_trajectory(
            task_description.gripper_goal_type, robot_description.gripper_state, task_description.goal.gripper,
            trajectory_size);

        // Indicate result includes gripper trajectory
        plannerResult.includes_gripper_trajectory = true;
    }

    // Convert the differential closed-chain joint trajectory to robot joint trajectory
    this->convert_cc_traj_to_robot_traj_(
        plannerResult.joint_trajectory, robot_description.joint_states,
        gripper_joint_trajectory); // Will insert gripper_joint_trajectory if task description included a gripper goal

    return plannerResult;
}

PlannerResult CcAffordancePlannerInterface::generate_specified_motion_joint_trajectory_(
    const Gsmt &generate_specified_motion_joint_trajectory,
    const Gsmt_st &generate_specified_motion_joint_trajectory_st, const Eigen::MatrixXd &slist,
    const Eigen::VectorXd &secondary_joint_goals, const size_t &nof_secondary_joints, const int &trajectory_density)

{
    PlannerResult transposeResult; // Result from the transpose planner
    PlannerResult inverseResult;   // Result from the inverse planner
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> transpose_result_obtained{false};
    std::atomic<bool> inverse_result_obtained{false};

    // Construct inverse and transpose planner objects
    CcAffordancePlanner *ccAffordancePlannerInversePtr(&ccAffordancePlannerInverse_);
    CcAffordancePlanner *ccAffordancePlannerTransposePtr(&ccAffordancePlannerTranspose_);

    // If a specific update method is requested, run the planner using that
    if (planner_config_.update_method == UpdateMethod::INVERSE)
    {
        inverseResult = (ccAffordancePlannerInversePtr->*generate_specified_motion_joint_trajectory)(
            slist, secondary_joint_goals, nof_secondary_joints, trajectory_density);
        inverseResult.update_method = UpdateMethod::INVERSE;
        inverseResult.update_trail += "inverse";
        return inverseResult;
    }

    if (planner_config_.update_method == UpdateMethod::TRANSPOSE)
    {
        transposeResult = (ccAffordancePlannerTransposePtr->*generate_specified_motion_joint_trajectory)(
            slist, secondary_joint_goals, nof_secondary_joints, trajectory_density);
        transposeResult.update_method = UpdateMethod::TRANSPOSE;
        transposeResult.update_trail += "transpose";
        return transposeResult;
    }

    // If a specific update method is not requested, run the inverse and transpose planners concurrently in separate
    // threads
    std::jthread inverse_thread([&](std::stop_token st) {
        inverseResult = (ccAffordancePlannerInversePtr->*generate_specified_motion_joint_trajectory_st)(
            slist, secondary_joint_goals, nof_secondary_joints, trajectory_density, st);
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
            slist, secondary_joint_goals, nof_secondary_joints, trajectory_density, st);
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
                                                                  const Eigen::VectorXd &start_joint_states,
                                                                  const std::vector<double> &gripper_joint_trajectory)
{
    // Return if the trajectory is empty, no need to attempt conversion
    if (cc_trajectory.empty())
    {
        return;
    }

    const size_t nof_robot_joints = start_joint_states.size();
    const size_t total_joints = cc_trajectory[0].size() + (gripper_joint_trajectory.empty() ? 0 : 1);

    Eigen::VectorXd cc_start_joint_states = Eigen::VectorXd::Zero(total_joints);
    cc_start_joint_states.head(nof_robot_joints) = start_joint_states;

    // Lambda expression to handle both cases of transform with or without gripper trajectory
    auto transform_point = [&](const Eigen::VectorXd &point,
                               std::optional<double> gripper_value = std::nullopt) -> Eigen::VectorXd {
        if (gripper_value)
        {
            Eigen::VectorXd new_point(total_joints);
            new_point.head(nof_robot_joints) = point.head(nof_robot_joints);
            new_point[nof_robot_joints] = 0.0;
            new_point.tail(point.size() - nof_robot_joints) = point.tail(point.size() - nof_robot_joints);
            cc_start_joint_states[nof_robot_joints] = *gripper_value;
            return new_point + cc_start_joint_states;
        }
        else
        {
            return point + cc_start_joint_states;
        }
    };

    // If gripper trajectory is provided, we insert it in the final trajectory
    if (!gripper_joint_trajectory.empty())
    {
        std::transform(cc_trajectory.begin(), cc_trajectory.end(), gripper_joint_trajectory.begin() + 1,
                       cc_trajectory.begin(), transform_point);

        // We start with the second point in the gripper trajectory for std::transform cuz we'll insert the first
        // element below

        cc_start_joint_states[nof_robot_joints] = gripper_joint_trajectory[0];
    }

    // Insert the start state at the beginning of the trajectory
    cc_trajectory.insert(cc_trajectory.begin(), cc_start_joint_states);

    // If gripper trajectory is not provided, we transform the trajectory without gripper consideration
    if (gripper_joint_trajectory.empty())
    {
        std::transform(cc_trajectory.begin() + 1, cc_trajectory.end(), cc_trajectory.begin() + 1,
                       [&](Eigen::VectorXd &point) { return transform_point(point); });
    }
}

void CcAffordancePlannerInterface::validate_input_(const affordance_util::RobotDescription &robot_description,
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

    if ((!std::isnan(task_description.goal.gripper)) && (std::isnan(robot_description.gripper_state)))
    {
        throw std::invalid_argument("Robot description: 'gripper_state' must be supplied and cannot be NaN when "
                                    "goal.gripper is specified in task description.");
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

    if (std::isnan(task_description.goal.affordance))
    {

        throw std::invalid_argument("Task description: 'goal.affordance' must be specified and cannot be NaN.");
    }

    /* if ((task_description.motion_type == MotionType::APPROACH) && */
    /*     ((!task_description.goal.grasp_pose.block<3, 3>(0, 0).isUnitary(tolerance)) || */
    /*      (std::abs(task_description.goal.grasp_pose(3, 3) - 1.0) > tolerance) || */
    /*      (!task_description.goal.grasp_pose.row(3).head(3).isZero(tolerance)))) */
    /* { */
    /*     throw std::invalid_argument("Task description: 'grasp_pose' is not a valid transformation matrix. Valid grasp
     * " */
    /*                                 "pose is needed for approach motion."); */
    /* } */

    if (task_description.trajectory_density < 2)
    {

        throw std::invalid_argument("Task description: 'trajectory_density' must be >= 2.");
    }
}

} // namespace cc_affordance_planner
