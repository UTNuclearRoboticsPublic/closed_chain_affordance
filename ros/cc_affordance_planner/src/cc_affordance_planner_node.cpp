#include <Eigen/Core>
#include <Eigen/Geometry>
#include <actionlib/client/simple_action_client.h>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/MoveItPlanAndViz.h>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
/*
   Author: Crasun Jans
*/
class CcAffordancePlannerNode {
public:
  const std::vector<std::string> joint_names = {
      "arm0_shoulder_yaw", "arm0_shoulder_pitch", "arm0_elbow_pitch",
      "arm0_elbow_roll",   "arm0_wrist_pitch",    "arm0_wrist_roll"};
  Eigen::VectorXd joint_states;
  double joint_states_timestamp;

  ros::Subscriber joint_states_subscriber;
  CcAffordancePlannerNode(ros::NodeHandle nh)
      : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_) {
    joint_states = Eigen::VectorXd::Zero(6);

    joint_states_subscriber = nh_.subscribe(
        "joint_states", 1, &CcAffordancePlannerNode::jointStatesCallback, this);
  }

  Eigen::Isometry3d get_htm(std::string targetFrame_,
                            std::string referenceFrame_) {

    Eigen::Isometry3d htm; // Output
    // Query the listener for the desired transformation at the latest available
    // time ros::Time::now()

    try {
      transformStamped_ = tfBuffer_.lookupTransform(
          targetFrame_, referenceFrame_, ros::Time(0), ros::Duration(0.3));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
    }
    ROS_INFO_STREAM("Here is transformStamped_: " << transformStamped_);

    // Convert the message to Eigen::Isometry3d type
    // Extract translation and rotation from the TransformStamped message
    Eigen::Vector3d translation(transformStamped_.transform.translation.x,
                                transformStamped_.transform.translation.y,
                                transformStamped_.transform.translation.z);
    Eigen::Quaterniond rotation(transformStamped_.transform.rotation.w,
                                transformStamped_.transform.rotation.x,
                                transformStamped_.transform.rotation.y,
                                transformStamped_.transform.rotation.z);

    // Set the translation and rotation components of the HTM
    htm.translation() = translation;
    htm.linear() = rotation.toRotationMatrix();

    return htm;
  }

  // Callback function to process joint states
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    // Collect joint state data for the specified joints in the specified order
    int j = 0; // index for selected_joint_positions_eigen
    for (const std::string &joint_name : joint_names) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == joint_name) {
          /* selected_joint_positions.push_back(msg->position[i]); */
          joint_states[j] = msg->position[i];
          j++;
          break; // Move to the next joint name
        }
      }
    }
  }

  control_msgs::FollowJointTrajectoryGoal follow_joint_trajectory_msg_builder(
      const std::vector<Eigen::VectorXd> &bare_trajectory,
      const std::vector<double> &config_offset,
      const double &time_step = 0.05) {
    control_msgs::FollowJointTrajectoryGoal fjtg_msg;

    // Set the joint names
    fjtg_msg.trajectory.joint_names = joint_names;

    fjtg_msg.trajectory.points.resize(
        bare_trajectory.size()); // resize points array before filling

    size_t j = 0;
    for (const Eigen::VectorXd &point : bare_trajectory) {
      trajectory_msgs::JointTrajectoryPoint &trajectory_point =
          fjtg_msg.trajectory.points[j];

      // Fill out point times
      trajectory_point.time_from_start = ros::Duration(j * time_step);

      // Fill out position values
      trajectory_point.positions.resize(
          joint_names.size()); // resize positions array before filling
      for (size_t i = 0; i < joint_names.size(); i++) {
        trajectory_point.positions[i] = point[i] + config_offset[i];
      }

      j++;
    }

    return fjtg_msg;
  }

  Eigen::MatrixXd robot_builder() {

    //** Creation of screw axes
    //* Screw axis locations from base frame in home position
    Eigen::MatrixXd q(3, 10);

    // Get q vectors from tf data
    /* std::string targetFrame_ = "arm0_base_link"; */
    /* for (size_t i = 0; i < joint_names.size(); i++) { */
    /*   std::string referenceFrame_ = joint_names.at(i); */
    /*   Eigen::Isometry3d htm = get_htm(targetFrame_, referenceFrame_); */
    /*   q.col(i) = htm.translation(); */
    /*   std::cout << "Reference frame: \n" << referenceFrame_ << std::endl; */
    /*   std::cout << "q: \n" << q.col(i) << std::endl; */
    /* } */

    // Hard-coded q-vector values
    q.col(0) << 0, 0, 0;
    q.col(1) << 0, 0, 0;
    q.col(2) << 0.3385, -0.0001, -0.0003;
    q.col(3) << 0.7428, -0.0003, 0.0693;
    q.col(4) << 0.7428, -0.0003, 0.0693;
    q.col(5) << 0.7428, -0.0003, 0.0693;

    // Type and alignment of screw axes
    Eigen::MatrixXd w(3, 10);
    w.col(0) << 0, 0, 1;
    w.col(1) << 0, 1, 0;
    w.col(2) << 0, 1, 0;
    w.col(3) << 1, 0, 0;
    w.col(4) << 0, 1, 0;
    w.col(5) << 1, 0, 0;
    w.col(6) << 1, 0, 0;  // Imaginary joint
    w.col(7) << 0, 1, 0;  // Imaginary joint
    w.col(8) << 0, 0, 1;  // Imaginary joint
    w.col(9) << -1, 0, 0; // Affordance

    // Get ee q-vector from tf data
    /* Eigen::Isometry3d eeHtm = get_htm(targetFrame_, "arm0_fingers"); */
    /* Eigen::Vector3d q_ee = eeHtm.translation(); */
    /* std::cout << "Here is q_ee: \n" << q_ee << std::endl; */

    // Hard-coded ee q-vector
    /* const Eigen::Vector3d q_ee(0.8605, -0.0002, */
    /*                            0.0818); // location of arm0_fingers */
    const Eigen::Vector3d q_ee(0.9383, 0.0005,
                               0.0664); // location of arm0_tool0

    // Affordance and virtual joint location
    const double affOffset = 0.1;
    const Eigen::Vector3d q_aff =
        q_ee + Eigen::Vector3d(0, affOffset, 0); // q-vector for affordance
    q.col(6) = q_ee;                             // imaginary joint
    q.col(7) = q_ee;                             // imaginary joint
    q.col(8) = q_ee;                             // imaginary joint
    q.col(9) = q_aff;                            // affordance

    // Compute screw axes
    Eigen::MatrixXd slist(6, 10);

    // Construct screw axes and frames
    for (int i = 0; i < w.cols(); i++) {
      Eigen::Vector3d wcurr =
          w.col(i); // required to convert to VectorXd type for use with cross
      Eigen::Vector3d qcurr =
          q.col(i); // required to convert to VectorXd type for use with cross
      slist.col(i) << wcurr, -wcurr.cross(qcurr);
    }

    // Pure translation edit for affordance
    slist.col(9) = (Eigen::VectorXd(6) << 0, 0, 0, -1, 0, 0).finished();

    // Pure rotation edit for affordance
    /* slist.col(9) = (Eigen::VectorXd(6) << 0, 0, -1, 0, 0, 0).finished(); */
    return slist;
  }

private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tfBuffer_;              // buffer to hold tf data
  tf2_ros::TransformListener tfListener_; // listener object for tf data
  geometry_msgs::TransformStamped
      transformStamped_; // ros message to hold transform info
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "cc_affordance_planner_node");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(
      1); // to get robot's state via MoveGroupInterface ros::spin needs to be
          // running. One way to do this is to have an asyncspinner going
          // beforehand
  spinner.start();

  CcAffordancePlannerNode capN(node_handle);

  /* // MoveIt Planning */
  /* // Before we can */
  /* // load the planning_pipeline planner, we need two objects, a RobotModel
   * and a */
  /* // PlanningScene. */
  /* // */
  /* // We will start by instantiating a `RobotModelLoader`_ object, which will
   */
  /* // look up the robot description on the ROS parameter server and construct
   * a */
  /* // :moveit_core:`RobotModel` for us to use. */
  /* // */
  /* // .. _RobotModelLoader: */
  /* //
   * http://docs.ros.org/noetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
   */
  /* robot_model_loader::RobotModelLoaderPtr robot_model_loader( */
  /*     new robot_model_loader::RobotModelLoader("robot_description")); */

  /* // Using the RobotModelLoader, we can construct a planing scene monitor
   * that */
  /* // will create a planning scene, monitors planning scene diffs, and apply
   * the */
  /* // diffs to it's internal planning scene. We then call startSceneMonitor,
   */
  /* // startWorldGeometryMonitor and startStateMonitor to fully initialize the
   */
  /* // planning scene monitor */
  /* planning_scene_monitor::PlanningSceneMonitorPtr psm( */
  /*     new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
   */

  /* /1* listen for planning scene messages on topic xxx and apply them to */
  /*                      the internal planning scene accordingly *1/ */
  /* psm->startSceneMonitor(); */
  /* /1* listens to changes of world geometry, collision objects, and
   * (optionally) */
  /*  * octomaps *1/ */
  /* psm->startWorldGeometryMonitor(); */
  /* /1* listen to joint state updates as well as changes in attached collision
   */
  /*    objects and update the internal planning scene accordingly*/
  /* psm->startStateMonitor(); */

  /* /1* We can also use the RobotModelLoader to get a robot model which
     contains */
  /*  * the robot's kinematic information *1/ */
  /* moveit::core::RobotModelPtr robot_model =
     robot_model_loader->getModel(); */

  /* /1* We can get the most up to date robot state from the
     PlanningSceneMonitor by */
  /*    locking the internal planning scene for reading. This lock ensures
     that the */
  /*    underlying scene isn't updated while we are reading it's state. */
  /*    RobotState's are useful for computing the forward and inverse
     kinematics of */
  /*    the robot among many other uses *1/ */
  /* moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
   */
  /*     planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
   */

  /* /1* Create a JointModelGroup to keep track of the current robot pose
     and */
  /*    planning group. The Joint Model */
  /*    group is useful for dealing with one set of joints at a time such as
     a left */
  /*    arm or a end effector *1/ */
  /* const moveit::core::JointModelGroup *joint_model_group = */
  /*     robot_state->getJointModelGroup("arm"); */

  /* // We can now setup the PlanningPipeline object, which will use the ROS
   */
  /* // parameter server to determine the set of request adapters and the
     planning */
  /* // plugin to use */
  /* planning_pipeline::PlanningPipelinePtr planning_pipeline( */
  /*     new planning_pipeline::PlanningPipeline( */
  /*         robot_model, node_handle, "planning_plugin",
     "request_adapters")); */

  /* // Visualization */
  /* // ^^^^^^^^^^^^^ */
  /* // The package MoveItVisualTools provides many capabilities for
     visualizing */
  /* // objects, robots, and trajectories in RViz as well as debugging tools
     such */
  /* // as step-by-step introspection of a script. */
  /* namespace rvt = rviz_visual_tools; */
  /* moveit_visual_tools::MoveItVisualTools visual_tools("arm0_base_link");
   */
  /* visual_tools.deleteAllMarkers(); */

  /* /1* Remote control is an introspection tool that allows users to step
     through a */
  /*    high level script via buttons and keyboard shortcuts in RViz *1/ */
  /* visual_tools.loadRemoteControl(); */

  /* /1* RViz provides many types of markers, in this demo we will use text,
   */
  /*  * cylinders, and spheres*/
  /* Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity(); */
  /* text_pose.translation().z() = 1.75; */
  /* visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", */
  /*                          rvt::WHITE, rvt::XLARGE); */

  /* // We will now create a motion plan request for the arm */
  /* planning_interface::MotionPlanRequest req; */
  /* planning_interface::MotionPlanResponse res; */
  /* req.group_name = "arm"; */

  /* // Visualization variables */
  /* ros::Publisher display_publisher = */
  /*     node_handle.advertise<moveit_msgs::DisplayTrajectory>( */
  /*         "/move_group/display_planned_path", 1, true); */
  /* moveit_msgs::DisplayTrajectory display_trajectory; */

  /* moveit_msgs::MotionPlanResponse response; */

  // Computing the rotational screw axis for affordance
  std::string affCapConfigConfirm;
  std::cout << "Put robot in the aff capture configuration " << std::endl;
  std::cin >> affCapConfigConfirm;

  if (affCapConfigConfirm != "y")
    return 1;
  //-------------------------------------------------------------------------
  // Get ee q-vector from tf data
  /* Eigen::Isometry3d eeHtm = capN.get_htm("arm0_base_link", "arm0_tool0"); */
  /* Eigen::Isometry3d tagHtm = capN.get_htm("arm0_base_link", "tag_7"); */
  /* Eigen::Isometry3d tagHtm = capN.get_htm("arm0_base_link",
   * "affordance_frame"); */
  Eigen::Isometry3d tagHtm = capN.get_htm("arm0_base_link", "arm0_tool0");
  /* Eigen::Isometry3d tagHtm = capN.get_htm("arm0_base_link", "arm0_tool0"); */
  /* Eigen::Vector3d q_ee = eeHtm.translation(); */
  Eigen::Vector3d q_tag = tagHtm.translation();
  std::cout << "Here is the tag location: " << q_tag << std::endl;
  /* const double affOffset = -0.2; */
  /* const Eigen::Vector3d q_aff(0, -0.2, 0); */
  Eigen::Vector3d q_aff;
  /* q_aff = q_tag + Eigen::Vector3d(0, 0, 0.410); */
  /* q_aff = Eigen::Vector3d(0, -0.2, 0); */
  q_aff = q_tag;
  /* const Eigen::Vector3d q_aff = */
  /*     q_ee + Eigen::Vector3d(0, affOffset, 0); // q-vector for affordance */
  Eigen::Vector3d wcurr(
      -1, 0, 0); // required to convert to VectorXd type for use with cross
  Eigen::Vector3d qcurr =
      q_aff; // required to convert to VectorXd type for use with cross
  Eigen::VectorXd s_aff(6);
  s_aff << wcurr, -wcurr.cross(qcurr);
  //-------------------------------------------------------------------------
  // Go to desired configuration
  // Define planning group and move_group_interface
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(
      PLANNING_GROUP);

  // Build robot
  Eigen::MatrixXd slist = capN.robot_builder();

  std::cout << "Here is the screwAxes matrix before mod: \n"
            << slist << std::endl;

  AffordanceUtil::RobotConfig robotConfig;
  robotConfig = AffordanceUtil::robot_builder(
      "/home/crasun/spot_ws/src/cc_affordance_planner/config/"
      "robot_setup.yaml");
  std::cout << "Here is the screwAxes matrix from the AffordanceUtil library: "
               "\n"
            << robotConfig.Slist << std::endl;

  std::string preStartConfigConfirm;
  std::cout << "Put robot in the prestart configuration " << std::endl;
  std::cin >> preStartConfigConfirm;

  if (preStartConfigConfirm != "y")
    return 1;

  // Go to hard-coded start config for moving a stool
  //----------------------------------------------------------------------------
  /* std::vector<double> stool_start_config = { */
  /*     -0.13461518287658691, 0.03472280502319336, 1.1548473834991455, */
  /*     -0.27599477767944336, -1.3527731895446777, 0.08957767486572266}; */
  /* std::vector<double> valve_start_config = { */
  /*     -0.002530813217163086, -0.9516636729240417, 1.6752504110336304, */
  /*     0.0010259151458740234, -0.680487871170044,  -0.009893417358398438}; */
  /* std::vector<double> start_config = valve_start_config; */
  /* move_group_interface.setJointValueTarget(start_config); */
  /* /1* // Lower the max acceleration and velocity to 5%. Default is 10% *1/ */
  /* move_group_interface.setMaxVelocityScalingFactor(0.05); */
  /* move_group_interface.setMaxAccelerationScalingFactor(0.05); */
  /* move_group_interface.move(); */
  //----------------------------------------------------------------------------
  // Put robot in the affordance start configuration to read joint states

  std::string startConfigConfirm;
  std::cout
      << "Put robot in the affordance start configuration to read joint states"
      << std::endl;
  std::cin >> startConfigConfirm;

  if (startConfigConfirm != "y")
    return 1;
  ros::spinOnce(); // read joint states

  // Record current state
  const moveit::core::JointModelGroup *joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP);
  moveit::core::RobotStatePtr current_state =
      move_group_interface.getCurrentState();
  std::vector<double> joint_group_positions_cur;
  current_state->copyJointGroupPositions(joint_model_group,
                                         joint_group_positions_cur);

  // Compute the robot Jacobian based on the joint states. This will be the
  // screw list we send to the robot
  const size_t total_nof_joints = 9;
  Eigen::MatrixXd robotSlist = slist.leftCols(total_nof_joints);
  Eigen::VectorXd joint_states_w_vjoints(total_nof_joints);
  joint_states_w_vjoints.head(6) = capN.joint_states;
  joint_states_w_vjoints.tail(3) << 0, 0, 0;
  std::cout << "Here is the joint_states_w_vjoints: \n"
            << joint_states_w_vjoints << std::endl;
  std::cout << "Here is the robotSlist: \n" << robotSlist << std::endl;
  Eigen::MatrixXd result =
      AffordanceUtil::JacobianSpace(robotSlist, joint_states_w_vjoints);
  slist.leftCols(joint_states_w_vjoints.size()) = result;
  std::cout << "Here is the screwAxes matrix after mod: \n"
            << slist << std::endl;

  // Compute the virtual gripper screw axes
  Eigen::Isometry3d newEEHtm = capN.get_htm("arm0_base_link", "arm0_tool0");
  Eigen::Vector3d new_qee = tagHtm.translation();
  Eigen::MatrixXd new_wee(3, 3);
  new_wee.col(0) << 1, 0, 0; // Imaginary joint
  new_wee.col(1) << 0, 1, 0; // Imaginary joint
  new_wee.col(2) << 0, 0, 1; // Imaginary joint
  Eigen::MatrixXd vir_slist(6, 4);
  // Construct screw axes and frames
  for (int i = 0; i < new_wee.cols(); i++) {
    Eigen::Vector3d new_wcurr = new_wee.col(
        i); // required to convert to VectorXd type for use with cross
    vir_slist.col(i) << new_wcurr, -new_wcurr.cross(new_qee);
  }
  /* vir_slist.rightCols(1) << 0, 0, 0, -1, 0, 0; // pure translation edit */
  // pure rotation edit
  Eigen::Vector3d rot_w(1, 0, 0);
  Eigen::Vector3d rot_q(0, -0.2, 0);
  Eigen::VectorXd rot_S(6);
  rot_S << rot_w, -rot_w.cross(rot_q);
  vir_slist.rightCols(1) = rot_S;
  slist.rightCols(4) = vir_slist;
  // Computing the rotational screw axis for affordance
  //-------------------------------------------------------------------------
  // Get ee q-vector from tf data
  /* Eigen::Isometry3d eeHtm = capN.get_htm("arm0_base_link", "arm0_tool0"); */
  /* Eigen::Isometry3d tagHtm = capN.get_htm("arm0_base_link", "tag_7"); */
  /* Eigen::Isometry3d tagHtm = capN.get_htm("arm0_base_link",
   * "affordance_frame"); */
  /* Eigen::Isometry3d tagHtm = capN.get_htm("arm0_base_link", "arm0_tool0"); */
  /* Eigen::Vector3d q_ee = eeHtm.translation(); */
  /* Eigen::Vector3d q_tag = tagHtm.translation(); */
  /* std::cout << "Here is the tag location: " << q_tag << std::endl; */
  /* const double affOffset = -0.2; */
  /* const Eigen::Vector3d q_aff(0, -0.2, 0); */
  /* Eigen::Vector3d q_aff; */
  /* q_aff = q_tag + Eigen::Vector3d(0, 0, 0.410); */
  /* q_aff = Eigen::Vector3d(0, -0.2, 0); */
  /* q_aff = q_tag; */
  /* const Eigen::Vector3d q_aff = */
  /*     q_ee + Eigen::Vector3d(0, affOffset, 0); // q-vector for affordance */
  /* Eigen::Vector3d wcurr( */
  /* -1, 0, 0); // required to convert to VectorXd type for use with cross */
  /* Eigen::Vector3d qcurr = */
  /* q_aff; // required to convert to VectorXd type for use with cross */
  /* Eigen::VectorXd s_aff(6); */
  /* s_aff << wcurr, -wcurr.cross(qcurr); */
  /* slist.col(9) = s_aff; */
  /* slist.col(9) << 0, 0, 0, 1, 0, 0; // pure translation edit */
  std::cout << "Here is the screwAxes matrix after vir mod: \n"
            << slist << std::endl;
  //-------------------------------------------------------------------------

  // Define affordance goal and create planner object
  /* const double affGoal = 1.0 * M_PI; */
  const double affGoal = 1.57;
  /* const double affGoal = 0.1; */
  Eigen::VectorXd thetalist0 = Eigen::VectorXd::Zero(slist.cols());
  std::cout << "Before creating the object" << std::endl;
  CcAffordancePlanner ccAffordancePlanner(slist, thetalist0, affGoal);
  ccAffordancePlanner.affStep = 0.01;
  ccAffordancePlanner.taskOffset = 1;
  /* ccAffordancePlanner.affStep = 0.01; */

  // Run the planner
  PlannerResult plannerResult = ccAffordancePlanner.affordance_stepper();

  // Print the first point in the trajectory if planner succeeds
  std::vector<Eigen::VectorXd> solution = plannerResult.jointTraj;
  if (plannerResult.success) {
    std::cout
        << "Planner succeeded with "
        << plannerResult.trajFullorPartial
        /* << " solution. and here is the first point in the trajectory \n" */
        /* << solution.at(0) << std::endl; */
        << " solution\n"
        << std::endl;
    /* std::cout << "Here is the trajectory \n"; */
    /* for (size_t i = 0; i < solution.size(); i++) { */
    /*   std::cout << solution.at(i) << std::endl; */
    /* } */
  } else {
    std::cout << "No solution found" << std::endl;
  }

  /* visual_tools.trigger(); */
  /* /1* Wait for user input *1/ */
  /* visual_tools.prompt("Press next to plan the trajectory"); */

  /* // Set the state in the planning scene to the current state of the robot */
  /* /1* static const std::string PLANNING_GROUP = "arm"; *1/ */
  /* /1* moveit::planning_interface::MoveGroupInterface move_group_interface(
   * *1/ */
  /* /1* PLANNING_GROUP); *1/ */
  /* moveit::core::RobotStatePtr current_state = */
  /*     move_group_interface.getCurrentState(); */
  /* std::vector<double> joint_group_positions_cur; */
  /* current_state->copyJointGroupPositions(joint_model_group, */
  /*                                        joint_group_positions_cur); */
  /* std::ostringstream joint_positions_stream; */
  /* for (const auto &position : joint_group_positions_cur) { */
  /*   joint_positions_stream << position << " "; */
  /* } */
  /* ROS_INFO_STREAM( */
  /*     "Here are the current joint values: " << joint_positions_stream.str());
   */
  /* std::cout << "Joint group positions cur: " << std::endl; */

  /* // Lock the planning scene and set the current state to the current robot
   */
  /* // state just to be sure */
  /* *robot_state = */
  /*     planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState();
   */

  /* **** Planning loop */
  /* // Create the planner object */

  /* for (size_t j = 0; j < solution.size(); j++) { */
  /*   /1* First, set the state in the planning scene to the final state of the
   * last */
  /*    * plan *1/ */

  /*   if (j == 0) { */
  /*     robot_state->setJointGroupPositions(joint_model_group, */
  /*                                         joint_group_positions_cur); */
  /*     moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
   */
  /*   } else { */

  /*     robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm) */
  /*                       ->getCurrentStateUpdated(response.trajectory_start);
   */
  /*     robot_state->setJointGroupPositions( */
  /*         joint_model_group, */
  /*         response.trajectory.joint_trajectory.points.back().positions); */
  /*     moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
   */
  /*     robot_state->setJointGroupPositions( */
  /*         joint_model_group, */
  /*         response.trajectory.joint_trajectory.points.back().positions); */
  /*   } */

  /*   /1*   if (j == 0) { *1/ */
  /*   /1*     std::vector<double> start_config = { *1/ */
  /*   /1*         -0.13461518287658691,
   * 0.03472280502319336, 1.1548473834991455, *1/ */
  /*   /1*         -0.27599477767944336, -1.3527731895446777,
   * 0.08957767486572266}; */
  /*    *1/ */
  /*   /1*     move_group_interface.setJointValueTarget(start_config); *1/ */
  /*   /1*   } else { *1/ */
  /*   std::vector<double> joint_group_positions; */
  /*   const size_t armDoF = 6; */

  /*   for (size_t i = 0; i < armDoF; i++) { */
  /*     joint_group_positions.push_back(joint_group_positions_cur[i] + */
  /*                                     solution.at(j)[i]); */
  /*     /1* joint_group_positions.push_back(solution.at(j)[i]); *1/ */
  /*   } */
  /*   /1*     move_group_interface.setJointValueTarget(joint_group_positions);
   * *1/ */
  /*   /1*   } *1/ */

  /*   // Now, setup a joint space goal */
  /*   moveit::core::RobotState goal_state(*robot_state); */
  /*   /1* std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0,
   */
  /*    * 0.0}; */
  /*    *1/ */
  /*   std::cout << "Joint group positions: " << std::endl; */
  /*   for (const double &value : joint_group_positions) { */
  /*     std::cout << value << " "; */
  /*   } */
  /*   std::cout << std::endl; */

  /*   goal_state.setJointGroupPositions(joint_model_group,
   * joint_group_positions); */
  /*   std::cout << "Debug flag" << std::endl; */
  /*   moveit_msgs::Constraints joint_goal = */
  /*       kinematic_constraints::constructGoalConstraints(goal_state, */
  /*                                                       joint_model_group);
   */

  /*   req.goal_constraints.clear(); */
  /*   req.goal_constraints.push_back(joint_goal); */

  /*   // Before planning, we will need a Read Only lock on the planning scene
   * so */
  /*   // that it does not modify the world representation while planning */
  /*   { */
  /*     planning_scene_monitor::LockedPlanningSceneRO lscene(psm); */
  /*     /1* Now, call the pipeline and check whether planning was successful.
   * *1/ */
  /*     planning_pipeline->generatePlan(lscene, req, res); */
  /*   } */
  /*   /1* Check that the planning was successful *1/ */
  /*   if (res.error_code_.val != res.error_code_.SUCCESS) { */
  /*     ROS_ERROR("Could not compute plan successfully"); */
  /*     return 0; */
  /*   } */
  /*   /1* Visualize the trajectory *1/ */
  /*   ROS_INFO("Visualizing the trajectory"); */
  /*   res.getMessage(response); */
  /*   display_trajectory.trajectory_start = response.trajectory_start; */
  /*   display_trajectory.trajectory.push_back(response.trajectory); */
  /*   // Now you should see two planned trajectories in series */
  /*   display_publisher.publish(display_trajectory); */
  /*   visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(),
   */
  /*                                      joint_model_group); */
  /* } */

  // Send the goal directly to the follow_joint_trajectory action server
  const double traj_time_step = 0.3;
  const control_msgs::FollowJointTrajectoryGoal goal =
      capN.follow_joint_trajectory_msg_builder(
          solution, joint_group_positions_cur, traj_time_step);
  ROS_INFO_STREAM("Here is the coveted ROS msg: " << goal);

  // Write the solution onto a file
  // Open a CSV file for writing
  std::ofstream csvFile("pred_tf_and_joint_states_data.csv");
  // Check if the file was opened successfully
  if (!csvFile.is_open()) {
    std::cerr << "Failed to open the CSV file for writing. " << std::endl;
    return 1;
  }

  for (const std::string &joint_name : capN.joint_names) {
    csvFile << joint_name << ",";
  }
  csvFile << "Pred EE x, Pred EE y, Pred EE z,"; // CSV header
  csvFile << "timestamp" << std::endl;
  std::cout << "Start recording predicted data? ";
  std::string writePredConf;
  std::cin >> writePredConf;

  if (writePredConf != "y") {
    std::cout << "You indicated you are not ready to record data. Exiting."
              << std::endl;
    return 1;
  }
  // Build robot
  Eigen::MatrixXd fkinSlist(6, 6);
  Eigen::MatrixXd orgSlist = capN.robot_builder();
  fkinSlist = orgSlist.leftCols(6).eval();
  Eigen::MatrixXd M = Eigen::Matrix4d::Identity();
  // Set the translation part
  /* M(0, 3) = 0.9383; */
  M(0, 3) = 0.94;
  /* M(1, 3) = 0.0005; */
  M(1, 3) = -0.0004;
  /* M(2, 3) = 0.0664; */
  M(2, 3) = 0.0656;
  double timestamp = 0.0;

  for (const auto &solPoint : goal.trajectory.points) {
    const auto &writeThetaList = solPoint.positions;

    // Write joint_states data to file
    for (int i = 0; i < writeThetaList.size(); ++i) {
      csvFile << writeThetaList[i] << ",";
    }

    // Write computed predicted ee htm to file
    Eigen::VectorXd writeThetaListVec(6);
    writeThetaListVec << writeThetaList[0], writeThetaList[1],
        writeThetaList[2], writeThetaList[3], writeThetaList[4],
        writeThetaList[5];
    Eigen::MatrixXd pred_ee_htm =
        AffordanceUtil::FKinSpace(M, fkinSlist, writeThetaListVec);
    csvFile << pred_ee_htm(0, 3) << "," << pred_ee_htm(1, 3) << ","
            << pred_ee_htm(2, 3) << ",";

    // Write timestamp to file
    csvFile << timestamp << std::endl;
    timestamp += traj_time_step;
  }

  // Send to move_plan_and_viz_server to visualize planning
  std::string plan_and_viz_service_name = "/moveit_plan_and_viz_server";
  ros::ServiceClient moveit_plan_and_viz_client =
      node_handle.serviceClient<cc_affordance_planner::MoveItPlanAndViz>(
          plan_and_viz_service_name);

  cc_affordance_planner::MoveItPlanAndViz moveit_plan_and_viz_goal;
  moveit_plan_and_viz_goal.request.joint_traj = goal.trajectory;

  ROS_INFO_STREAM("Waiting for " << plan_and_viz_service_name
                                 << " service to be available");
  ros::Duration timeout(60.0);
  if (!ros::service::waitForService(plan_and_viz_service_name)) {
    ROS_INFO_STREAM(plan_and_viz_service_name
                    << " service isn't available. Timed out waiting");
  } else {
    ROS_INFO_STREAM(plan_and_viz_service_name
                    << " service found. Now sending goal.");
  }

  if (moveit_plan_and_viz_client.call(moveit_plan_and_viz_goal)) {
    ROS_INFO_STREAM("Calling " << plan_and_viz_service_name
                               << " service was successful.");
    std::string executionConfirm;
    std::cout << "Ready to execute the trajectory? y to confirm" << std::endl;
    std::cin >> executionConfirm;

    if (executionConfirm != "y")
      return 1;

    // Execute directly
    // Create the connection to the action server
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
        client("/spot_arm/arm_controller/follow_joint_trajectory", true);
    client.waitForServer(); // Waits until the action server is up and running

    client.sendGoal(goal);
    client.waitForResult();
  }

  else {
    ROS_INFO_STREAM("Failed to call " << plan_and_viz_service_name
                                      << " service.");
  }

  return 0;
}
