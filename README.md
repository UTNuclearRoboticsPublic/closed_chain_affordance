# Affordance Planning Framework

This repository encompasses two C++ packages, namely `affordance_util` and `cc_affordance_planner`, collectively forming a standalone library framework for affordance planning based on the closed-chain affordance model described in the paper referenced at `<paper_reference>`.

## Installation Instructions

1. Create a temporary directory and clone this repository in there:
   ```
   mkdir ~/temp_cca_ws
   ```
   ```
   cd ~/temp_cca_ws
   ```
   ```
   git clone -b cpp git@github.com:UTNuclearRobotics/closed_chain_affordance.git
   ```

2. Navigate to the `affordance_util` directory and create a `build` folder at the same level as the `src` folder and navigate to it:
   ```
   cd ~/temp_cca_ws/closed_chain_affordance
   ```
   ```
   mkdir build
   ```
   ```
   cd build
   ```

3. Run CMake to configure the build process. This will generate the build system files in the build directory. Also turn the PIC flag on so the ROS packages that use this can make shared objects:
   ```
   cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON
   ```

4. Execute the following command to build the project:
   ```
   cmake --build .
   ```

5. Install the `affordance_util` library and header files:
   ```
   cmake --install .
   ```

6. Repeat the above process for the `cc_affordance_planner` package.

7. Remove the temporary directory along with this repo clone since we don't need them anymore:
   ```
   cd ~/temp_cca_ws
   ```
   ```
   rm -rf ~/temp_cca_ws
   ```

At this point, if you need to use these libraries in other projects, you can do so by adding the following to your CMakeLists file and linking the libraries, for instance, `affordance_util` against your desired targets:
```cmake
FIND_LIBRARY(affordance_util_LIBRARIES affordance_util /usr/local/lib)
```
Note: Verify that the libraries were indeed installed in the /usr/local/lib folder and that the header files were placed in the /usr/local/include folder. If installed elsewhere, replace the path in the above call.

## ROS2 Implementation

If using ROS2 to interface the closed-chain affordance planner with a physical robot, go to the main branch of this repository and follow instructions there:</br>
   [ROS2 implementation instructions](https://github.com/UTNuclearRobotics/closed_chain_affordance/tree/main)
