#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

/*
   Author: Crasun Jans
*/
class CcAffordancePlanner {
public:
  CcAffordancePlanner(ros::NodeHandle nh)
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
/* geometry_msgs::TransformStamped */
/*     transformStamped_; // ros message to hold transform info */
/* Eigen::Isometry3d get_htm(tf2_ros::Buffer tfBuffer_, std::string
 * targetFrame_, */
/*                           std::string referenceFrame_) { */

/*   Eigen::Isometry3d htm; // Output */
/*   // Query the listener for the desired transformation at the latest
 * available */
/*   // time ros::Time::now() */
/*   try { */
/*     transformStamped_ = tfBuffer_.lookupTransform( */
/*         targetFrame_, referenceFrame_, ros::Time(0), ros::Duration(0.2)); */
/*   } catch (tf2::TransformException &ex) { */
/*     ROS_WARN("%s", ex.what()); */
/*   } */

/*   // Convert the message to Eigen::Isometry3d type */
/*   // Extract translation and rotation from the TransformStamped message */
/*   Eigen::Vector3d translation(transformStamped_.transform.translation.x, */
/*                               transformStamped_.transform.translation.y, */
/*                               transformStamped_.transform.translation.z); */
/*   Eigen::Quaterniond rotation(transformStamped_.transform.rotation.w, */
/*                               transformStamped_.transform.rotation.x, */
/*                               transformStamped_.transform.rotation.y, */
/*                               transformStamped_.transform.rotation.z); */

/*   // Set the translation and rotation components of the HTM */
/*   htm.translation() = translation; */
/*   htm.linear() = rotation.toRotationMatrix(); */

/*   return htm; */
/* } */

/* Eigen::VectorXd htmToScrew(Eigen::Isometry3d htm) { */

/*   /1* std::cout << "Here is the rotation of the HTM: \n" *1/ */
/*   /1*           << htm.linear() << std::endl; *1/ */
/*   /1* std::cout << "Here is the translation of the HTM: \n" *1/ */
/*   /1*           << htm.translation() << std::endl; *1/ */
/*   Eigen::VectorXd s(6); */
/*   Eigen::Vector3d w = htm.linear().block<3, 1>(0, 2); */
/*   s.head(3) = w; */
/*   s.tail(3) = -w.cross(htm.translation()); */
/*   return s; */
/* } */
int main(int argc, char **argv) {
  ros::init(argc, argv, "urdf_parser_example");
  ros::NodeHandle nh;

  CcAffordancePlanner ccAffordancePlanner(nh);
  std::vector<std::string> joint_names;
  joint_names.push_back("arm0_shoulder_yaw");
  joint_names.push_back("arm0_shoulder_pitch");
  joint_names.push_back("arm0_elbow_pitch");
  joint_names.push_back("arm0_elbow_roll");
  joint_names.push_back("arm0_wrist_pitch");
  joint_names.push_back("arm0_wrist_roll");

  std::string referenceFrame_ = "arm0_base_link";

  std::cout << "Screw axis of the wrist roll with respect to the base_link"
            << std::endl;

  Eigen::MatrixXd screwAxes(6, joint_names.size());
  for (int i = 0; i < joint_names.size(); i++) {
    std::string targetFrame_ = joint_names.at(i);
    screwAxes.col(i) << ccAffordancePlanner.htmToScrew(
        ccAffordancePlanner.get_htm(targetFrame_, referenceFrame_));
  }

  std::cout << "Here is the screwAxes matrix: \n" << screwAxes << std::endl;

  return 0;
}
