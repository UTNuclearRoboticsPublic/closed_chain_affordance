#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <affordance_util_ros/affordance_util_ros.hpp>
#include <condition_variable>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <csignal>
#include <fstream>
#include <iomanip> // for std::precision
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <thread>
/*
Author: Crasun Jans
*/

/****************** Signal handling ********************************/
static bool g_exit_flag = false; // signal to shutdown ROS on ctrl+c

// Function to handle ctrl+c signal
static void signal_callback_handler(int signum) { g_exit_flag = true; }

/****************** EOF Signal handling ****************************/

/***** Joint Trajectory and EE TF Recorder class ******************/
class JointTrajAndTfRecorder
{
  public:
    JointTrajAndTfRecorder(const std::string &robot_config_file_path, const std::string &as_server_name) : nh_("~")
    {

        // Subscribers
        follow_joint_traj_sub_ =
            nh_.subscribe(as_server_name + "/goal", 1000, &JointTrajAndTfRecorder::follow_joint_traj_sub_cb_, this);
        joint_states_sub_ = nh_.subscribe("/joint_states", 1000, &JointTrajAndTfRecorder::joint_states_cb_, this);

        // Get abs path to the directory where we will save data
        const std::string rel_data_save_path = "/../data/";
        abs_data_save_path_ = AffordanceUtilROS::get_abs_path_to_rel_dir(__FILE__, rel_data_save_path);

        // Extract robot config info
        const AffordanceUtil::RobotConfig &robotConfig = AffordanceUtil::robot_builder(robot_config_file_path);
        slist_ = robotConfig.Slist;
        joint_names_ = robotConfig.joint_names;
        M_ = robotConfig.M;
        tool_name_ = robotConfig.tool_name;

        // Concurrently, while writing predicted data, we'll write actual data as
        // well, because while predicted data is being written, action server is
        // probably executing joint movement already
        act_data_writer_thread_ = std::thread(&JointTrajAndTfRecorder::write_act_data_, this);
        sentinel_cleanup_thread_ = std::thread(&JointTrajAndTfRecorder::cleanup_post_interruption_,
                                               this); // post-signal cleanup thread
    }

    ~JointTrajAndTfRecorder()
    {

        // Join the threads before exiting
        act_data_writer_thread_.join();
    }

  private:
    // ROS variables
    ros::NodeHandle nh_;
    ros::Subscriber follow_joint_traj_sub_;
    ros::Subscriber joint_states_sub_;
    AffordanceUtilROS::JointTrajPoint joint_states_;
    // Robot data
    Eigen::MatrixXd slist_;
    std::vector<std::string> joint_names_;
    Eigen::MatrixXd M_;
    std::string tool_name_;
    // Multithreading and data-sync tools
    std::mutex mutex_;
    std::thread act_data_writer_thread_;
    std::thread sentinel_cleanup_thread_; // thread to ensure cleanup after ctrl+c
                                          // interruption
    std::condition_variable joint_states_cv_;
    std::condition_variable follow_joint_traj_cv_;
    std::condition_variable cleanup_cv_; // CV to wake thread up after successful
                                         // cleanup in act_data_writer_thread_
    bool cb_called_ = false;
    bool joint_states_ready_ = false;
    // Other variables
    std::string abs_data_save_path_;

    // Function to handle cleanup on signal interruption
    void cleanup_post_interruption_()
    {
        // Wait until signal interruption is received
        while (!g_exit_flag)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Wake up act_data_writer_thread_
        joint_states_cv_.notify_all();
        follow_joint_traj_cv_.notify_all();
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cleanup_cv_.wait(lock); // Wait until act_data_writer_ thread is
                                    // successfully cleaned up
        }
        std::cout << "Successfully exited program because of ctrl+c interruption\n";
        ros::shutdown(); // Shutdown ROS
    }

    // Callback function for the follow_joint_trajectory goal subscriber
    void follow_joint_traj_sub_cb_(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr &msg)
    {
        // Lock the mutex and update cb_called_ flag
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cb_called_ = true;
        } // scope for mutex

        // Wake the other thread up
        follow_joint_traj_cv_.notify_one();

        // Extract trajectory, put it in the right order, and then, call the writing
        // function
        const auto &unordered_pred_traj_ = msg->goal.trajectory;
        const std::vector<AffordanceUtilROS::JointTrajPoint> pred_traj_ =
            AffordanceUtilROS::get_ordered_joint_traj(unordered_pred_traj_, joint_names_);
        std::cout << "Writing predicted data now" << std::endl;
        write_pred_data(pred_traj_);
        std::cout << "Finished writing predicted data" << std::endl;
    }

    // Callback function for the joint_states subscriber
    void joint_states_cb_(const sensor_msgs::JointState::ConstPtr &msg)
    {
        // Lock the mutex and update joint_states and data-ready flag
        {
            std::lock_guard<std::mutex> lock(mutex_);
            joint_states_ = AffordanceUtilROS::get_ordered_joint_states(msg, joint_names_);
            joint_states_ready_ = true;
        }

        // Wake the other thread up
        joint_states_cv_.notify_one();
    }

    // Function to write predicted data to file
    void write_pred_data(const std::vector<AffordanceUtilROS::JointTrajPoint> &pred_traj_)
    {

        const std::string filename = "pred_tf_and_joint_states_data.csv";
        const std::string filepath = abs_data_save_path_ + filename;

        // Open a CSV file for writing
        std::ofstream csvFile(filepath);

        // Set precision for writing to file
        csvFile << std::fixed << std::setprecision(5); // fixed-type notation and 5 decimal places

        // Check if the file was opened successfully
        if (!csvFile.is_open())
        {
            std::cerr << "Failed to open the predicted-data CSV file for writing. " << std::endl;
            return;
        }

        /* Headers */
        // Joint_names
        for (const std::string &joint_name : joint_names_)
        {
            csvFile << joint_name << ",";
        }
        // EE position and timestamp
        csvFile << "Pred EE x,Pred EE y,Pred EE z,"; // CSV header
        csvFile << "Timestamp"
                << "\n";

        for (const auto &pred_traj_point : pred_traj_)
        {

            // Joint positions
            for (size_t i = 0; i < pred_traj_point.positions.size(); ++i)
            {
                csvFile << pred_traj_point.positions[i] << ",";
            }

            // EE position
            Eigen::MatrixXd ee_htm = AffordanceUtil::FKinSpace(M_, slist_, pred_traj_point.positions);
            csvFile << ee_htm(0, 3) << "," << ee_htm(1, 3) << "," << ee_htm(2, 3) << ",";

            // Timestamp
            csvFile << pred_traj_point.timestamp << "\n";
        }

        // Close file before exiting
        csvFile.close();
    }

    // Function to write actual data to file
    void write_act_data_()
    {

        ros::Rate loop_rate(10); // Rate for the writing loop

        const std::string filename = "act_tf_and_joint_states_data.csv";
        const std::string filepath = abs_data_save_path_ + filename;

        // Open a CSV file for writing
        std::ofstream csvFile(filepath);

        // Set precision for writing to file
        csvFile << std::fixed << std::setprecision(5); // fixed-type notation and 5 decimal places

        // Check if the file was opened successfully
        if (!csvFile.is_open())
        {
            std::cerr << "Failed to open the actual-data CSV file for writing" << std::endl;
            return;
        }

        // Put the thread to sleep and check for cb_called_ to be true. Once true,
        // set it to false and move on.
        {
            std::unique_lock<std::mutex> lock(mutex_);
            follow_joint_traj_cv_.wait(lock, [this] { return (cb_called_ || g_exit_flag); });
            if (g_exit_flag)
            {
                // Close the file when done
                csvFile.close();
                std::cout << "Exited without writing actual data" << std::endl;
                cleanup_cv_.notify_all(); // Wake up cleanup thread
            }

            cb_called_ = false;
        }

        std::cout << "Writing actual data now" << std::endl;

        for (const std::string &joint_name : joint_names_)
        {
            csvFile << joint_name << ",";
        }
        csvFile << "Act EE x,Act EE y,Act EE z,"; // CSV header
        csvFile << "Timestamp"
                << "\n";

        // Write data to file until interrupted with ctrl-c
        while (true)
        {
            // Wait for joint states data to be ready routinely checking it while
            // putting the thread to sleep at other times and releasing the mutex.
            // When it is ready, copy it, set data-ready flag to false, release the
            // mutex, and move on
            AffordanceUtilROS::JointTrajPoint joint_states_copy;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                joint_states_cv_.wait(lock, [this] { return (joint_states_ready_ || g_exit_flag); });
                if (g_exit_flag)
                    break;
                joint_states_copy = joint_states_;
                joint_states_ready_ = false;
            }
            // Write joint_states data to file
            for (size_t i = 0; i < joint_states_copy.positions.size(); ++i)
            {
                csvFile << joint_states_copy.positions[i] << ",";
            }

            // Write EE position to file
            Eigen::MatrixXd ee_htm = AffordanceUtil::FKinSpace(M_, slist_, joint_states_copy.positions);

            csvFile << ee_htm(0, 3) << "," << ee_htm(1, 3) << "," << ee_htm(2, 3) << ",";

            // Write timestamp to file
            csvFile << joint_states_copy.timestamp << "\n";

            // Sleep
            loop_rate.sleep();
        }

        // Close the file when done
        csvFile.close();

        std::cout << "Finished writing actual data" << std::endl;

        cleanup_cv_.notify_all(); // Wake up cleanup thread
    }
};
/***** EOF Joint Trajectory and EE TF Recorder class *************/
int main(int argc, char **argv)
{
    ros::init(argc, argv,
              "joint_traj_and_tf_recorder"); // FYI, send this additional argument
                                             // to tell ROS signal handling will
                                             // be taken care of separately:
                                             // ros::init_options::NoSigintHandler

    // Furnish the filepath where the robot config yaml file is located and
    // supply the action server name whose goal to listen to
    const std::string robot_config_file_path = "/home/crasun/spot_ws/src/cc_affordance_spot_description/config/"
                                               "cc_affordance_spot_description.yaml"; // TODO: to retrieve the filepath
                                                                                      // from package name and relative
                                                                                      // path
    const std::string as_server_name = "/spot_arm/arm_controller/follow_joint_trajectory";
    JointTrajAndTfRecorder jointTrajAndTfRecorder(robot_config_file_path, as_server_name);

    // Ctrl+c signal handling
    signal(SIGINT,
           signal_callback_handler); // signal handler

    ROS_INFO("Joint_traj_and_tf_recorder is active");
    ros::spin();

    return 0;
}
