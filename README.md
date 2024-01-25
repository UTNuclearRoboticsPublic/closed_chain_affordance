# Affordance Planning Framework

This repository encompasses two C++ packages, namely `affordance_util` and `cc_affordance_planner`, collectively forming an independent framework for affordance planning based on the closed-chain affordance model described in the paper referenced at `<paper_reference>`.

## Install the `affordance_util` Library

### 1. Create a Build Folder

Execute the following commands to create a build folder at the same level as the `src` folder in the CMake package:

```bash
mkdir build
cd build
```

### 2. Configure the Build
Run CMake to configure the build process. This will generate the build system files in the build directory. Also turn the PIC flag on so the ROS packages that use this can make shared objects.
```bash
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON
```

### 3. Build the project
Execute the following command to build the project:
```bash
cmake --build .
```

### 4. Install the Project
To install the project and make the library and header files available for other projects, run:
```bash
cmake --install .
```
### 5. Repeat the above process for the cc_affordance_planner package

At this point, if you need to use these libraries in other projects you can do so by adding the following to your CMakeLists file and linking the libraries, for instance, `affordance_util` against your desired targets:
```cmake
FIND_LIBRARY(affordance_util_LIBRARIES affordance_util /usr/local/lib)
```
Note: verify that the libraries were indeed installed in the /usr/local/lib folder and that the header files were placed in the /usr/local/include folder. If installed elsewhere, replace the path in the above call.
