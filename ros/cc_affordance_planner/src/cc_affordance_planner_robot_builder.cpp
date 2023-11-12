/* Author : Crasun Jans */
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>

struct JointData {
  std::string name;
  std::vector<double> w;
  std::vector<double> q;
};

int main() {
  try {
    // Load the YAML file
    YAML::Node config = YAML::LoadFile("/home/crasun/spot_ws/src/cc_affordance_planner/config/robot_setup.yaml");

    // Access the 'joints' array
    const YAML::Node &jointsNode = config["joints"];

    // Parse each joint
    std::vector<JointData> jointsData;
    for (const YAML::Node &jointNode : jointsNode) {
      JointData joint;
      joint.name = jointNode["name"].as<std::string>();
      joint.w = jointNode["w"].as<std::vector<double>>();
      joint.q = jointNode["q"].as<std::vector<double>>();
      jointsData.push_back(joint);
    }

    // Display the parsed data (replace with your logic)
    for (const auto &joint : jointsData) {
      std::cout << "Joint Name: " << joint.name << std::endl;
      std::cout << "w: [" << joint.w[0] << ", " << joint.w[1] << ", "
                << joint.w[2] << "]" << std::endl;
      std::cout << "q: [" << joint.q[0] << ", " << joint.q[1] << ", "
                << joint.q[2] << "]" << std::endl;
      std::cout << std::endl;
    }

  } catch (const YAML::Exception &e) {
    std::cerr << "Error reading YAML file: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
