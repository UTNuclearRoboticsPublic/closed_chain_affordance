#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <urdf/model.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "urdf_parser_example");
  ros::NodeHandle nh;

  std::string urdf_string;
  // Fetch the URDF XML from the robot_description parameter.
  if (!nh.getParam("robot_description", urdf_string)) {
    ROS_ERROR("Failed to retrieve URDF from the robot_description parameter.");
    return 1;
  }
  urdf::Model model;
  /* if (!model.initFile("path/to/urdf")) { */ // If wanted to parse directly
                                               // from a urdf file, use this
  if (!model.initString(urdf_string)) {
    ROS_ERROR("Failed to parse URDF file");
    return 1;
  }

  urdf::LinkConstSharedPtr base_link = model.getLink("arm0_base_link");
  urdf::LinkConstSharedPtr end_effector = model.getLink("arm0_fingers");

  if (!base_link || !end_effector) {
    ROS_ERROR("Failed to get base_link or end_effector from URDF");
    return 1;
  }

  std::cout
      << "Transformation matrices for joints with respect to the base joint:\n";

  urdf::LinkConstSharedPtr current_link = end_effector;
  while (current_link != base_link) {
    urdf::JointSharedPtr joint = current_link->parent_joint;
    if (joint) {
      if (joint->type != urdf::Joint::FIXED) { // Skip fixed joints
        const urdf::Pose &origin = joint->parent_to_joint_origin_transform;
        std::cout << "Joint: " << joint->name << std::endl;
        std::cout << "Translation (XYZ): " << origin.position.x << " "
                  << origin.position.y << " " << origin.position.z << std::endl;
        std::cout << "Rotation (XYZW): " << origin.rotation.x << " "
                  << origin.rotation.y << " " << origin.rotation.z << " "
                  << origin.rotation.w << std::endl;
        Eigen::Quaterniond q(origin.rotation.w, origin.rotation.x,
                             origin.rotation.y, origin.rotation.z);
        Eigen::Matrix3d rotation_matrix = q.normalized().toRotationMatrix();
        std::cout << "Rotation Matrix: \n" << rotation_matrix << std::endl;

        Eigen::Vector3d w = rotation_matrix.block<3, 1>(0, 0);
        Eigen::Vector3d q_vector(origin.position.x, origin.position.y,
                                 origin.position.z);
        Eigen::VectorXd s(6);
        s.head(3) = w;
        s.tail(3) = -w.cross(q_vector);

        std::cout << "Screw axis: \n" << s << std::endl;
      }
    } else {
      ROS_WARN("No joint found between links. Terminating.");
      break;
    }
    current_link = current_link->getParent();
  }

  return 0;
}
