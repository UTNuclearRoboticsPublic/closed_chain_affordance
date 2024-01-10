#include <Eigen/Core>
#include <Eigen/Geometry>
#include <actionlib/client/simple_action_client.h>
#include <affordance_util/affordance_util.hpp>
#include <affordance_util_ros/affordance_util_ros.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cmath>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_plan_and_viz/MoveItPlanAndViz.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
/*
   Author: Crasun Jans
*/

class CcAffordancePlannerRos
{
  public:
    CcAffordancePlannerRos(const std::string &robot_config_file_path, const Eigen::VectorXd &aff_screw,
                           const double &aff_goal)
        : nh_("~"),
          aff_screw(aff_screw),
          aff_goal(aff_goal),
          traj_execution_client_("/spot_arm/arm_controller/follow_joint_trajectory", true),
          moveit_plan_and_viz_client_("/moveit_plan_and_viz_server", true)
    {

        // Subscribers
        joint_states_sub_ = nh_.subscribe("/joint_states", 1000, &CcAffordancePlannerRos::joint_states_cb_, this);

        // Extract robot config info
        const AffordanceUtil::RobotConfig &robotConfig = AffordanceUtil::robot_builder(robot_config_file_path);
        robot_slist_ = robotConfig.Slist;
        M_ = robotConfig.M;
        tool_name_ = robotConfig.tool_name;
        joint_names_ = robotConfig.joint_names;
    }

    ~CcAffordancePlannerRos() {}

    void run_cc_affordance_planner()
    {

        // Capture initial configuration joint states
        std::string capture_joint_states_conf;
        std::cout << "Put the robot in the start configuration for affordance "
                     "execution. Done? y for yes."
                  << std::endl;
        std::cin >> capture_joint_states_conf;

        if (capture_joint_states_conf != "y" && capture_joint_states_conf != "Y")
        {
            std::cout << "You indicated you are not ready to capture joint states" << std::endl;
            return;
        }
        // Take a second to read callbacks to ensure proper capturing of joint
        // states data
        ros::Rate loop_rate(4);
        for (int i = 0; i < 4; ++i)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }

        // Manually set joint_states for planning time computation purposes only
        /* joint_states_.positions << */

        std::cout << "Here are the captured joint states: \n" << joint_states_.positions << std::endl;

        // Compose cc model and affordance goal
        Eigen::MatrixXd cc_slist =
            AffordanceUtilROS::compose_cc_model_slist(robot_slist_, joint_states_.positions, M_, aff_screw);
        std::cout << "Here is the full cc slist: \n" << cc_slist << std::endl;

        // Construct the CcAffordancePlanner object
        CcAffordancePlanner ccAffordancePlanner;

        // Set planner parameters
        auto sign_of = [](double x) {
            return (x > 0) ? 1.0 : (x < 0) ? -1.0 : 0.0;
        }; // Helper lambda to check the sign of affordance goal

        const double aff_step = 0.01;         // To be hard-coded as needed
        const double accuracy = 10.0 / 100.0; // To be hard-coded as needed

        ccAffordancePlanner.p_aff_step_deltatheta_a = sign_of(aff_goal) * aff_step;
        ccAffordancePlanner.p_task_err_threshold_eps_s = accuracy * aff_step;

        // Run the planner by passing the screw list, affordance goal, and task offset
        const int gripper_control_par_tau = 1;
        PlannerResult plannerResult =
            ccAffordancePlanner.affordance_stepper(cc_slist, aff_goal, gripper_control_par_tau);

        // Print the first point in the trajectory if planner succeeds
        std::vector<Eigen::VectorXd> solution = plannerResult.joint_traj;
        if (plannerResult.success)
        {
            std::cout << "Planner succeeded with " << plannerResult.traj_full_or_partial
                      << " solution, and planning took " << plannerResult.planning_time << " microseconds" << std::endl;
        }
        else
        {
            std::cout << "Planner did not find a solution" << std::endl;
        }

        // Convert the solution trajectory to ROS message type
        const double traj_time_step = 0.3;
        const control_msgs::FollowJointTrajectoryGoal goal = AffordanceUtilROS::follow_joint_trajectory_msg_builder(
            solution, joint_states_.positions, joint_names_,
            traj_time_step); // this function takes care of extracting the right
                             // number of joint_states although solution
                             // contains qs data too

        // Visualize plan
        moveit_plan_and_viz::MoveItPlanAndViz moveit_plan_and_viz_goal;
        moveit_plan_and_viz_goal.request.joint_traj = goal.trajectory;

        ros::Duration timeout(60.0); // 1 minute
        if (!moveit_plan_and_viz_client_.waitForServer(timeout))
        {
            std::cout << "Could not find service within the timeout" << std::endl;
            return;
        }

        if (moveit_plan_and_viz_client_.call(moveit_plan_and_viz_goal))
        {
            std::cout << "Calling " << plan_and_viz_service_name << " service was successful." << std::endl;

            // Execute trajectory on the real robot
            std::string execution_conf;
            std::cout << "Ready to execute the trajectory? y to confirm" << std::endl;
            std::cin >> execution_conf;

            if (execution_conf != "y" && execution_conf != "Y")
            {
                std::cout << "You indicated you are not ready to execute trajectory" << std::endl;
                return;
            }

            if (!traj_execution_client_.waitForServer(timeout))
            {
                std::cout << "Could not find follow joint trajectory action server within the timeout" << std::endl;
                return;
            }

            std::cout << "Sending goal to action server for execution" << std::endl;

            traj_execution_client_.sendGoal(goal);

            if (!traj_execution_client_.waitForResult(timeout))
            {
                std::cout << "Follow joint trajectory action server did not return result within timeout" << std::endl;
                return;
            }
        }

        else
        {
            std::cout << "Failed to call " << plan_and_viz_service_name << " service." << std::endl;
        }
    }

  private:
    // ROS variables
    ros::NodeHandle nh_;
    ros::Subscriber joint_states_sub_; // Joint states subscriber
    ros::ServiceClient<moveit_plan_and_viz::MoveItPlanAndViz>
        moveit_plan_and_viz_client_; // Service client to visualize joint trajectory
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
        traj_execution_client_; // Action client to execute joint trajectory on robot

    AffordanceUtilROS::JointTrajPoint joint_states_; // Processed and ordered joint states data

    // Robot data
    Eigen::MatrixXd robot_slist_;
    std::vector<std::string> joint_names_;
    Eigen::MatrixXd M_;
    std::string tool_name_;

    // Callback function for the joint_states subscriber
    void joint_states_cb_(const sensor_msgs::JointState::ConstPtr &msg)
    {
        joint_states_ = AffordanceUtilROS::get_ordered_joint_states(
            msg,
            joint_names_); // Takes care of filtering and ordering the joint_states
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cc_affordance_planner_node");

    // Buffer to lookup tf data for the tag
    tf2_ros::Buffer tf_buffer;

    // Find CC Affordance Description for the robot
    const std::string package_name = "cc_affordance_spot_description";
    const std::string rel_dir = "/config/";
    const std::string filename = package_name + ".yaml";
    std::string robot_cc_description_path;
    robot_cc_description_path = get_filepath_inside_pkg(package_name, rel_dir, filename);

    // Compose affordance screw
    const bool aff_from_tag = false; // To be hard-coded as needed
    Eigen::Matrix<double, 6, 1> aff_screw;

    if (aff_from_tag)
    {
        const std::string affordance_frame = "affordance_frame"; // Name of the tag frame
        const std::string reference_frame = "arm0_base_link";    // Maybe we can move the aff_from_tag portion inside
                                                                 // the class later since reference_frame info is
                                                                 // available there.
        // Set screw axis
        const Eigen::Vector3d w_aff(-1, 0, 0); // To be hard-coded as needed

        // Extract Affordance frame location from TF data
        const Eigen::Isometry3d tag_htm = AffordanceUtilROS::get_htm(reference_frame, affordance_frame, tf_buffer);
        const Eigen::Vector3d q_aff = tag_htm.translation();
        std::cout << "Here is the affordance frame location. Ensure it makes sense: \n" << q_aff << std::endl;

        // Compute the 6x1 screw vector
        aff_screw = get_screw(w_aff, q_aff); // return screw axis
    }
    else
    {
        const Eigen::Vector3d w_aff(-1, 0, 0); // To be hard-coded as needed
        const Eigen::Vector3d q_aff(0, 0, 0);  // To be hard-coded as needed

        // Compute the 6x1 screw vector
        aff_screw = get_screw(w_aff, q_aff);

        // Pure translation edit
        aff_screw = (Eigen::Matrix<double, 6, 1>() << 0, 0, 0, -1, 0, 0).finished();
    }

    // Set affordance goal
    /* const double aff_goal = -0.5 * M_PI; */
    const double aff_goal = -0.29; // To be hard-coded as needed

    // Construct the planner object and run the planner
    CcAffordancePlannerRos ccAffordancePlannerRos(robot_cc_description_path, aff_screw, aff_goal);
    ccAffordancePlannerRos.run_cc_affordance_planner();

    return 0;
}
