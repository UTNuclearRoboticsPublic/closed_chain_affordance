///////////////////////////////////////////////////////////////////////////////
//      Title     : cc_affordance_planner_ros.hpp
//      Project   : cc_affordance_planner_ros
//      Created   : Fall 2023
//      Author    : Janak Panthi (Crasun Jans)
//      Copyright : Copyright© The University of Texas at Austin, 2014-2026. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative license is explicitly included within the text of
//          each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation, including but not limited to those resulting from
//          defects in software and/or documentation, or loss or inaccuracy of
//          data of any kind.
//
///////////////////////////////////////////////////////////////////////////////
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

class CcAffordancePlannerRos
{
  public:
    CcAffordancePlannerRos(const std::string &robot_config_file_path)
        : nh_("~"),
          traj_execution_client_("/spot_arm/arm_controller/follow_joint_trajectory",
                                 true), // Simple action client does not have a default
                                        // constructor and needs to initialized this way
          joint_states_sub_(nh_.subscribe("/joint_states", 1000, &CcAffordancePlannerRos::joint_states_cb_, this)),
          moveit_plan_and_viz_client_(
              nh_.serviceClient<moveit_plan_and_viz::MoveItPlanAndViz>("/moveit_plan_and_viz_server"))
    {
        // Extract robot config info
        const AffordanceUtil::RobotConfig &robotConfig = AffordanceUtil::robot_builder(robot_config_file_path);
        robot_slist_ = robotConfig.Slist;
        M_ = robotConfig.M;
        tool_name_ = robotConfig.tool_name;
        joint_names_ = robotConfig.joint_names;
    }

    void run_cc_affordance_planner(const Eigen::VectorXd &aff_screw, const double &aff_goal)
    {

        // Get joint states at the start configuration of the affordance
        /* Eigen::VectorXd robot_thetalist = get_aff_start_joint_states_(); */
        Eigen::VectorXd robot_thetalist(joint_names_.size());
        /* robot_thetalist << -0.00076, -0.87982, 1.73271, 0.01271, -1.13217, -0.00273; // Pulling a drawer */
        /* robot_thetalist << 0.00795, -1.18220, 2.46393, 0.02025, -1.32321, */
        /*     -0.00053; // Pushing a drawer */
        robot_thetalist << 0.20841, -0.52536, 1.85988, 0.18575, -1.37188, -0.07426; // Moving a stool
        /* robot_thetalist << 0.08788, -1.33410, 2.14567, 0.19725, -0.79857, 0.46613; // Turning a valve2 */
        /* robot_thetalist << 0.03456, -1.40627, 2.10997, 0.13891, -0.66079, 0.76027; // Turning a valve4 */

        std::cout << "Here are the captured joint states: \n" << joint_states_.positions << std::endl;

        // Compose cc model and affordance goal
        Eigen::MatrixXd cc_slist = AffordanceUtil::compose_cc_model_slist(robot_slist_, robot_thetalist, M_, aff_screw);
        std::cout << "Here is the full cc slist: \n" << cc_slist << std::endl;

        // Construct the CcAffordancePlanner object
        CcAffordancePlanner ccAffordancePlanner;

        // Set planner parameters
        auto sign_of = [](double x) {
            return (x > 0) ? 1.0 : (x < 0) ? -1.0 : 0.0;
        }; // Helper lambda to check the sign of affordance goal

        /* const double aff_step = 0.05; // Pulling a drawer */
        /* const double aff_step = 0.05;         // Pushing a drawer */
        const double aff_step = 0.15; // Moving a stool
        /* const double aff_step = 0.2;          // Turning a valve2 */
        /* const double aff_step = 0.2;          // Turning a valve4 */
        const double accuracy = 10.0 / 100.0; //

        ccAffordancePlanner.p_aff_step_deltatheta_a = sign_of(aff_goal) * aff_step;
        ccAffordancePlanner.p_task_err_threshold_eps_s = accuracy * aff_step;

        // Run the planner by passing the screw list, affordance goal, and task
        // offset
        /* const int gripper_control_par_tau = 1; // Pulling a drawer */
        /* const int gripper_control_par_tau = 1; // Pushing a drawer */
        const int gripper_control_par_tau = 1; // Moving a stool
        /* const int gripper_control_par_tau = 3; // Turning a valve2 */
        /* const int gripper_control_par_tau = 3; // Turning a valve4 */
        PlannerResult plannerResult =
            ccAffordancePlanner.affordance_stepper(cc_slist, aff_goal, gripper_control_par_tau);

        // Print the first point in the trajectory if planner succeeds
        std::vector<Eigen::VectorXd> solution = plannerResult.joint_traj;
        if (plannerResult.success)
        {
            std::cout << "Planner succeeded with " << plannerResult.traj_full_or_partial
                      << " solution, and planning took " << plannerResult.planning_time.count() << " microseconds"
                      << std::endl;
            /* std::cout << "Here are the start and end affordances, " */
            /*           << "Start: " << solution[0](9) << ", End: " << solution[solution.size() - 1](9) << std::endl;
             */
            /* std::cout << "Here is the full solution:" << std::endl; */
            /* for (const auto &point : solution) */
            /* { */
            /*     std::cout << point << "\n\n"; */
            /* } */
        }
        else
        {
            std::cout << "Planner did not find a solution" << std::endl;
        }

        // Visualize and execute trajectory
        /* visualize_and_execute_trajectory_(solution); */
    }

  private:
    // Subscriber and server-related variables
    ros::NodeHandle nh_;
    ros::Subscriber joint_states_sub_;              // Joint states subscriber
    ros::ServiceClient moveit_plan_and_viz_client_; // Service client to visualize
                                                    // joint trajectory
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
        traj_execution_client_;                      // Action client to execute joint trajectory on
                                                     // robot
    AffordanceUtilROS::JointTrajPoint joint_states_; // Processed and ordered joint states data

    // Robot data
    Eigen::MatrixXd robot_slist_;
    std::vector<std::string> joint_names_;
    Eigen::MatrixXd M_;
    std::string tool_name_;

    // Methods
    // Callback function for the joint_states subscriber
    void joint_states_cb_(const sensor_msgs::JointState::ConstPtr &msg)
    {
        joint_states_ = AffordanceUtilROS::get_ordered_joint_states(
            msg,
            joint_names_); // Takes care of filtering and ordering the joint_states
    }

    // Function to read robot joint states at the start of the affordance
    Eigen::VectorXd get_aff_start_joint_states_()
    {
        // Manually set joint_states for planning time computation purposes only
        joint_states_.positions.conservativeResize(joint_names_.size());

        // Capture initial configuration joint states
        std::string capture_joint_states_conf;
        std::cout << "Put the robot in the start configuration for affordance "
                     "execution. Done? y for yes."
                  << std::endl;
        std::cin >> capture_joint_states_conf;

        if (capture_joint_states_conf != "y" && capture_joint_states_conf != "Y")
        {
            throw std::runtime_error("You indicated you are not ready to capture joint states");
        }
        // Take a second to read callbacks to ensure proper capturing of joint
        // states data
        ros::Rate loop_rate(4);
        for (int i = 0; i < 4; ++i)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
        return joint_states_.positions;
    }

    // function to visualize trajectory with moveit in Rviz and then, execute
    // afterwards on the robot
    void visualize_and_execute_trajectory_(std::vector<Eigen::VectorXd> trajectory)
    {

        const std::string traj_execution_server_name =
            "/spot_arm/arm_controller/follow_joint_trajectory"; // Action client
                                                                // doesn't have a
                                                                // way to use an AS
                                                                // name variable
                                                                // from within this
                                                                // class. So, this
                                                                // variable (as
                                                                // class variable)
                                                                // can't be used to
                                                                // initialize the
                                                                // client. Thus, we
                                                                // limit its scope
                                                                // in here as
                                                                // needed.
        const std::string moveit_plan_and_viz_server_name =
            "/moveit_plan_and_viz_server"; // Service client does not have the same
                                           // problem, but for consistency, we limit
                                           // this variable's scope as well.

        // Convert the solution trajectory to ROS message type
        const double traj_time_step = 0.3;
        const control_msgs::FollowJointTrajectoryGoal goal = AffordanceUtilROS::follow_joint_trajectory_msg_builder(
            trajectory, joint_states_.positions, joint_names_,
            traj_time_step); // this function takes care of extracting the right
                             // number of joint_states although solution
                             // contains qs data too

        //------------------------------------------------------------------------------------------------------
        // Print the contents of the goal using std::cout
        std::cout << "Goal:\n";

        std::cout << "\nPoints:\n";

        for (const auto &point : goal.trajectory.points)
        {
            std::cout << "  - Positions: ";
            for (const auto &position : point.positions)
            {
                std::cout << position << " ";
            }

            std::cout << "\n    Time From Start: " << point.time_from_start << "\n";
        }

        //------------------------------------------------------------------------------------------------------
        // Visualize plan
        moveit_plan_and_viz::MoveItPlanAndViz moveit_plan_and_viz_goal;
        moveit_plan_and_viz_goal.request.joint_traj = goal.trajectory;

        ros::Duration timeout(60.0); // 1 minute
        if (!ros::service::waitForService(moveit_plan_and_viz_server_name, timeout))
        {
            std::cout << "Could not find " << moveit_plan_and_viz_server_name << " service within the timeout"
                      << std::endl;
            return;
        }

        if (moveit_plan_and_viz_client_.call(moveit_plan_and_viz_goal))
        {
            std::cout << "Calling " << moveit_plan_and_viz_server_name << " service was successful." << std::endl;

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
                std::cout << "Could not find " << traj_execution_server_name << " action server within the timeout"
                          << std::endl;
                return;
            }

            std::cout << "Sending goal to " << traj_execution_server_name << " action server for execution"
                      << std::endl;

            traj_execution_client_.sendGoal(goal);

            if (!traj_execution_client_.waitForResult(timeout))
            {
                std::cout << traj_execution_server_name << " action server did not return result within timeout"
                          << std::endl;
                return;
            }
        }

        else
        {
            std::cout << "Failed to call " << moveit_plan_and_viz_server_name << " service." << std::endl;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cc_affordance_planner_ros_node");
    // Set the logging level to INFO
    /* ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); */
    /* ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug); */

    // Buffer to lookup tf data for the tag
    tf2_ros::Buffer tf_buffer;

    // Find CC Affordance Description for the robot
    const std::string package_name = "cc_affordance_spot_description";
    const std::string rel_dir = "/config/";
    const std::string filename = package_name + ".yaml";
    std::string robot_cc_description_path;
    robot_cc_description_path = AffordanceUtilROS::get_filepath_inside_pkg(package_name, rel_dir, filename);

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
        aff_screw = AffordanceUtil::get_screw(w_aff, q_aff); // return screw axis
    }
    else
    {
        const Eigen::Vector3d w_aff(0, 0, 1); // Moving a stool
        const Eigen::Vector3d q_aff(0, 0, 0); // Moving a stool
        /* const Eigen::Vector3d w_aff(-1, 0, 0);                       // Turning a valve2 */
        /* const Eigen::Vector3d q_aff(0.597133, -0.0887238, 0.170599); // Turning a valve2 */
        /* const Eigen::Vector3d w_aff(-1, 0, 0);                     // Turning a valve4 */
        /* const Eigen::Vector3d q_aff(0.602653, -0.119387, 0.16575); // Turning a valve4 */

        /* // Compute the 6x1 screw vector */
        aff_screw = AffordanceUtil::get_screw(w_aff, q_aff);

        // Pure translation edit
        /* aff_screw = (Eigen::Matrix<double, 6, 1>() << 0, 0, 0, -1, 0, 0).finished(); // Pulling a drawer */
        /* aff_screw = (Eigen::Matrix<double, 6, 1>() << 0, 0, 0, 1, 0, */
        /*              *0).finished(); // Pushing a drawer */
    }

    // Set affordance goal
    const double aff_goal = 0.5 * M_PI; // Moving a stool
    /* const double aff_goal = -1.5 * M_PI; // Turning a valve2 */
    /* const double aff_goal = -0.5 * M_PI; // Turning a valve4 */
    /* const double aff_goal = -0.29; // Pulling a drawer */
    /* const double aff_goal = 0.2; // Pushing a drawer*/

    // Construct the planner object and run the planner
    CcAffordancePlannerRos ccAffordancePlannerRos(robot_cc_description_path);
    ccAffordancePlannerRos.run_cc_affordance_planner(aff_screw, aff_goal);

    // Call the function again with new (or old) aff_screw and aff_goal to execute
    // another affordance in series
    /* ccAffordancePlannerRos.run_cc_affordance_planner(aff_screw, aff_goal); */

    return 0;
}
