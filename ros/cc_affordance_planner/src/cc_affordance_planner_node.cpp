#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

/*
   Author: Crasun Jans
*/
class CcAffordancePlannerNode {
public:
  CcAffordancePlannerNode(ros::NodeHandle nh)
      : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_) {}

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

  Eigen::VectorXd htmToScrew(Eigen::Isometry3d htm) {

    /* std::cout << "Here is the rotation of the HTM: \n" */
    /*           << htm.linear() << std::endl; */
    /* std::cout << "Here is the translation of the HTM: \n" */
    /*           << htm.translation() << std::endl; */
    Eigen::VectorXd s(6);
    Eigen::Vector3d w = htm.linear().block<3, 1>(0, 2);
    s.head(3) = w;
    s.tail(3) = -w.cross(htm.translation());
    return s;
  }

private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tfBuffer_;              // buffer to hold tf data
  tf2_ros::TransformListener tfListener_; // listener object for tf data
  geometry_msgs::TransformStamped
      transformStamped_; // ros message to hold transform info
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "urdf_parser_example");
  ros::NodeHandle nh;

  CcAffordancePlannerNode ccAffordancePlannerNode(nh);
  //** Creation of screw axes
  //* Screw axis locations from base frame in home position
  // First get HTMs from TF data
  std::vector<std::string> joint_names;
  joint_names.push_back("arm0_shoulder_yaw");
  joint_names.push_back("arm0_shoulder_pitch");
  joint_names.push_back("arm0_elbow_pitch");
  joint_names.push_back("arm0_elbow_roll");
  joint_names.push_back("arm0_wrist_pitch");
  joint_names.push_back("arm0_wrist_roll");

  std::string targetFrame_ = "arm0_base_link";

  Eigen::MatrixXd q(3, 10);

  for (size_t i = 0; i < joint_names.size(); i++) {
    std::string referenceFrame_ = joint_names.at(i);
    Eigen::Isometry3d htm =
        ccAffordancePlannerNode.get_htm(targetFrame_, referenceFrame_);
    q.col(i) = htm.translation();
    std::cout << "Reference frame: \n" << referenceFrame_ << std::endl;
    std::cout << "q: \n" << q.col(i) << std::endl;
  }

  // Type and alignment of screw axes
  Eigen::MatrixXd w(3, 10);
  w.col(0) << 0, 0, 1;
  w.col(1) << 0, 1, 0;
  w.col(2) << 0, 1, 0;
  w.col(3) << 1, 0, 0;
  w.col(4) << 0, 1, 0;
  w.col(5) << 1, 0, 0;
  w.col(6) << 1, 0, 0; // Imaginary joint
  w.col(7) << 0, 1, 0; // Imaginary joint
  w.col(8) << 0, 0, 1; // Imaginary joint
  w.col(9) << 1, 0, 0; // Affordance

  Eigen::Isometry3d eeHtm =
      ccAffordancePlannerNode.get_htm(targetFrame_, "arm0_fingers");
  Eigen::Vector3d q_ee = eeHtm.translation();
  Eigen::Vector3d q_aff = q_ee + Eigen::Vector3d(0.1, 0, 0);
  q.col(6) = q_ee;  // imaginary joint
  q.col(7) = q_ee;  // imaginary joint
  q.col(8) = q_ee;  // imaginary joint
  q.col(9) = q_aff; // affordance
  std::cout << "Debug flag" << std::endl;
  Eigen::MatrixXd slist(6, 10);

  // Construct screw axes and frames
  for (int i = 0; i < w.cols(); i++) {
    Eigen::Vector3d wcurr =
        w.col(i); // required to convert to VectorXd type for use with cross
    Eigen::Vector3d qcurr =
        q.col(i); // required to convert to VectorXd type for use with cross
    slist.col(i) << wcurr, -wcurr.cross(qcurr);
  }
  std::cout << "w: \n" << w << std::endl;
  std::cout << "q: \n" << q << std::endl;

  std::cout << "Here is the screwAxes matrix: \n" << slist << std::endl;

  Eigen::Matrix4d Tsd = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d mErr = Eigen::Matrix4d::Identity();
  Eigen::VectorXd thetalist0 = Eigen::VectorXd::Zero(slist.cols());
  std::cout << "Before creating the object" << std::endl;
  CcAffordancePlanner ccAffordancePlanner(slist, mErr, thetalist0, Tsd);

  std::vector<Eigen::VectorXd> solution =
      ccAffordancePlanner.affordance_stepper();

  if (solution.empty())
    std::cout << "No solution found" << std::endl;
  else
    std::cout << "Here is the first point in the solution \n"
              << solution.at(0) << std::endl;

  // Send it to MoveIt for planning

  return 0;
}
