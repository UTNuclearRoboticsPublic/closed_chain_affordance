# Affordance Planning Framework

This repository encompasses two C++ packages, namely `affordance_util` and `cc_affordance_planner`, collectively forming a standalone library framework for affordance planning based on the closed-chain affordance model described in the paper referenced at `<paper_reference>`.

## Installation Instructions
Follow these steps to install the `affordance_util` and `cc_affordance_planner` libraries:

1. Create a temporary directory and clone this repository in there:
   ```bash
   mkdir ~/temp_cca_ws && cd ~/temp_cca_ws
   ```
   ```
   git clone git@github.com:UTNuclearRoboticsPublic/closed_chain_affordance.git
   ```

2. Navigate to the `affordance_util` directory and create a `build` folder at the same level as the `src` folder and navigate to it:
   ```bash
   cd closed_chain_affordance/affordance_util/ && mkdir build && cd build
   ```

3. Run CMake to configure the build process. This will generate the build system files in the build directory. Also turn the PIC flag on so the ROS packages that use this can make shared objects:
   ```bash
   cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON
   ```

4. Execute the following command to build the project:
   ```bash
   cmake --build .
   ```

5. Install the `affordance_util` library and header files:
   ```bash
   sudo cmake --install .
   ```

6. Repeat the above process for the `cc_affordance_planner` package.
   ```bash
   cd ~/temp_cca_ws/closed_chain_affordance/cc_affordance_planner
   ```
   ```bash
   mkdir build && cd build
   ```
   ```bash
   cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON && cmake --build . && sudo cmake --install .
   ```

7. Remove the temporary directory along with this repo clone since we don't need them anymore:
   ```bash
   cd && rm -rf ~/temp_cca_ws
   ```

## ROS2 Implementation

The framework is designed to be directly used in your Cpp project (see Usage section below). However, a ROS2 interface is available to facilitate easy implementation on physical robots. To utilize the ROS2 interface, which is basically a wrapper around this Cpp library, go to the following repository and follow instructions there:</br>
   [ROS2 implementation instructions](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_ros.git)

## Usage Information
This section provides detailed instructions on configuring your project's CMakeLists.txt file (see Housekeeping) and writing code (see Code) to utilize the planner.
### Housekeeping
If you need to use these libraries in other projects, you can do so by adding the following to your CMakeLists file:
To find the libraries:
```cmake
FIND_LIBRARY(affordance_util_LIBRARIES affordance_util /usr/local/lib)
FIND_LIBRARY(cc_affordance_planner_LIBRARIES cc_affordance_planner /usr/local/lib)
```
<small>Note: Verify that the libraries were indeed installed in the /usr/local/lib folder and that the header files were placed in the /usr/local/include folder. This can be done by ensuring each of the following commands returns some output. If installed elsewhere, replace the path in the above calls.</small>
```bash
ls /usr/local/lib/ | grep affordance_util
```
```bash
ls /usr/local/include/ | grep affordance_util
```
```bash
ls /usr/local/lib/ | grep cc_affordance_planner
```
```bash
ls /usr/local/include/ | grep cc_affordance_planner
```
To link the libraries against your target:
```cmake
 target_link_libraries(${PROJECT_NAME} affordance_util cc_affordance_planner)
```
where `${PROJECT_NAME}` is your desired target.

### Code
This section describes the required and optional code setup for this planner.
#### Required Setup
Using the planner is straightforward and requires the following 5 steps.
1. Include these headers:
```cpp
#include <affordance_util_ros/affordance_util_ros.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
```
2. Instantiate the planner object with planner configuration:
```cpp
cc_affordance_planner::PlannerConfig plannerConfig;
plannerConfig.trajectory_density = 4; // desired number of points in the joint trajectory, 4 for example

cc_affordance_planner::CcAffordancePlannerInterface ccAffordancePlannerInterface(plannerConfig);
```
3. Furnish robot description
```cpp
affordance_util::RobotDescription robot_description;
robot_description.slist = ; // Eigen::MatrixXd with robot joint screws as its columns in order
robot_description.M = ; // Eigen::Matrix4d representing the homogenous transformation matrix for the EE (palm) at home position
robot_description.joint_states = ; // Eigen::VectorXd representing the joint states of the robot in order at the start config of the affordance
```

4. Furnish task description. Here is an example for affordance-type motion. More examples in Task Examples section.
```cpp
cc_affordance_planner::TaskDescription task_description;

// Affordance
affordance_util::ScrewInfo aff;
aff.type = affordance_util::ScrewType::ROTATION; // Possible values are ROTATION, TRANSLATION, SCREW
aff.axis = Eigen::Vector3d(1, 0, 0); // Eigen::Vector3d representing the affordance screw axis, [1,0,0] for example
aff.location = Eigen::Vector3d(0, 0, 0); // Eigen::Vector3d representing the location of the affordance screw axis, [0,0,0] for example

task_description.affordance_info = aff;

// Goals
task_description.goal.affordance = 0.4; // Goal for the affordance, 0.4 for instance
```

4. Here is a task-description example for approach-type motion. More examples in Task Examples section.
```cpp
cc_affordance_planner::TaskDescription task_description;

// Affordance
affordance_util::ScrewInfo aff;
aff.type = affordance_util::ScrewType::ROTATION; // Possible values are ROTATION, TRANSLATION, SCREW. SCREW requires the pitch parameter.
aff.axis = Eigen::Vector3d(1, 0, 0); // Eigen::Vector3d representing the affordance screw axis, [1,0,0] for example
aff.location = Eigen::Vector3d(0, 0, 0); // Eigen::Vector3d representing the location of the affordance screw axis, [0,0,0] for example

task_description.affordance_info = aff;

// Goals
task_description.goal.affordance = 0.4; // Goal for the affordance, 0.4 for instance
task_description.goal.grasp_pose = ; // Eigen::Matrix4d representing grasp pose. Only relevant for approach motion
```
5. Generating the joint trajectory to accomplish the specified task:
```cpp
try
{
plannerResult = ccAffordancePlannerInterface.generate_joint_trajectory(robot_description, task_description);
}
catch (const std::invalid_argument &e)
{
std::cerr << "Planner returned exception: " << e.what() << std::endl;
}
```

Reading the result:
```cpp
if (plannerResult.success)
{
std::vector<Eigen::VectorXd> solution = plannerResult.joint_trajectory;// Contains the entire closed-chain trajectory. For robot trajectory that you can send to a j-joint robot, extract the first j joint positions in each point in the trajectory.
// Additional planning result info
switch (plannerResult.trajectory_description)
        {

        case cc_affordance_planner::TrajectoryDescription::FULL:
            std::cout<<"Trajectory description: FULL.\n";
            break;
        case cc_affordance_planner::TrajectoryDescription::PARTIAL:
            std::cout<<"Trajectory description: PARTIAL. Execute with caution.\n";
            break;
        default:
            std::cout<<"Trajectory description: UNSET.\n";
            break;
        }
std::cout << "The entire planning took " << plannerResult.planning_time.count() << " microseconds\n";
}
else
{
std::cerr << "Planner did not find a solution." << std::endl;
}
```

#### Optional Features
Below are optional and advanced features of this framework that one may choose to use as needed.

##### Gripper Trajectory Consideration
The framework can compute a joint trajectory that considers the gripper. To use this feature simply, provide the current state of the gripper in robot description and specify the desired goal state in task description. Optionally, provide gripper goal type.
```cpp
robot_description.gripper_state = 0.0; // Joint value of the gripper
task_description.goal.gripper = 0.4;
task_description.gripper_goal_type = affordance_util::GripperGoalType::CONSTANT; // Possible values are CONSTANT and CONTINUOUS. CONSTANT is default and means the gripper joint will have the desired value, for instance, 0.4 for all points in the trajectory. CONTINUOUS means it will take trajectory_density number of points to go from the current gripper state to the goal gripper state.
```

##### Optional and Advanced Settings
Optional planner config parameters with default values:
```cpp
plannerConfig.accuracy = 10.0/100; // accuracy of the planner, 10% for example
plannerConfig.update_method = cc_affordance_planner::UpdateMethod::INVERSE; // method used to solve closed-chain IK. Choices are INVERSE, TRANSPOSE, and BEST. BEST runs INVERSE and TRANSPOSE in parallel and returns the best result.
plannerConfig.motion_type = cc_affordance_planner::MotionType::AFFORDANCE; // Choices are AFFORDANCE and APPROACH.
```
Optional advanced planner config parameters with default values:
```cpp
plannerConfig.closure_err_threshold_ang = 1e-4; // Threshold for the closed-chain closure angular error
plannerConfig.closure_err_threshold_lin = 1e-5; // Threshold for the closed-chain closure linear error
plannerConfig.ik_max_itr = 200; // Limit for the number of iterations for the closed-chain IK solver
```

Optional task description parameters:
```cpp
task_description.goal.ee_orientation = Eigen::Vector3d(0.1, 0.0, 0.1); // EE orientation to maintain along the affordance path, rpy = [0.1,0.0,0.1] for instance. Can specify one or more aspections of the orientation in the order specified by VirScrewOrder below.
task_description.vir_screw_order = affordance_util::VirtualScrewOrder::XYZ; // Order of the axes in the closed-chain model virtual gripper joint. Possible values are XYZ, YZX, ZXY, and NONE.
task_description.motion_type = cc_affordance_planner::MotionType::AFFORDANCE; // Possible values are AFFORDANCE and APPROACH. Default is AFFORDANCE.

// For affordance, one can directly specify the 6x1 screw vector for affordance instead of axis and location
aff.type = affordance_util::ScrewType::TRANSLATION;
aff.screw = Eigen::Vector6d(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
```
#### Task Examples
##### Affordance - Rotation
```cpp
cc_affordance_planner::TaskDescription task_description;

// Affordance
aff.type = affordance_util::ScrewType::ROTATION;
aff.axis = Eigen::Vector3d(0, 0, 1);
aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);
aff_goal = (Eigen::VectorXd(1) << (1.0 / 2.0) * M_PI).finished();
task_description.affordance_info = aff;

// Goal
task_description.goal.affordance = (1.0 / 2.0) * M_PI;
```

##### Affordance - Translation
```cpp
cc_affordance_planner::TaskDescription task_description;

// Affordance
aff.type = affordance_util::ScrewType::TRANSLATION;
aff.axis = Eigen::Vector3d(1.0, 0.0, 0.0);
aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);
task_description.affordance_info = aff;

// Goal
task_description.goal.affordance = 0.5;
```

##### Affordance - Screw
```cpp
cc_affordance_planner::TaskDescription task_description;

// Affordance
aff.type = affordance_util::ScrewType::SCREW;
aff.axis = Eigen::Vector3d(0, 0, 1);
aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);
aff.pitch = 0.5;
task_description.affordance_info = aff;

// Goal
task_description.goal.affordance = (1.0 / 2.0) * M_PI;
```

##### Affordance - Rotation with EE Orientation Control
```cpp
cc_affordance_planner::TaskDescription task_description;

// Affordance
aff.type = affordance_util::ScrewType::ROTATION;
aff.axis = Eigen::Vector3d(0, 0, 1);
aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);
aff_goal = (Eigen::VectorXd(1) << (1.0 / 2.0) * M_PI).finished();
task_description.affordance_info = aff;

// Goal
task_description.goal.affordance = (1.0 / 2.0) * M_PI;
task_description.goal.ee_orientation = Eigen::Vector3d(0.1, 0.0, 0.1); # Gripper-frame orientation per vir_screw_order. Default as roll-pitch-yaw.
```

##### Approach Motion
```cpp
cc_affordance_planner::TaskDescription task_description;
task_description.motion_type = cc_affordance_planner::MotionType::APPROACH;

// Affordance
aff.type = affordance_util::ScrewType::ROTATION;
aff.axis = Eigen::Vector3d(0, 0, 1);
aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);
task_description.affordance_info = aff;

// Goal
task_description.goal.affordance = (1.0 / 4.0) * M_PI; // This is where the joint trajectory will take you. Specify where along the affordance path to "approach".
task_description.goal.grasp_pose = Eigen::Matrix4d::Identity(); // Specify a valid grasp pose HTM. This is the reference pose where affordance magnitude is 0.
```

### Author
Janak Panthi aka Crasun Jans
