# Install the affordance_util library by running the following commands
## Create a build folder at the same level as the src folder in the CMake package
`mkdir build` </br>
`cd build`
## Configure the Build
Run CMake to configure the build process. This will generate the build system files in the build directory. Also turn the PIC flag on so the ROS packages that use this can make shared objects.</br>
`cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON`
## Build the project
`cmake --build .`
## Install the Project
To install the project and make the library and header files available for other projects, run:</br>
`cmake --install .` </br>
Then, you can use the library in other projects by adding the following to your CMakeLists file and linking the library `affordance_util` against your desired targets:</br>
`FIND_LIBRARY(affordance_util_LIBRARIES affordance_util /usr/local/lib)` </br>
Note: verify that the affordance_util library was indeed installed in the /usr/local/lib folder and that the header was placed in the /usr/local/include folder. If it was installed somewhere else, replace the path in the above call. When uninstalling and reinstalling, manually delete the affordance_util-related files in those folders.
