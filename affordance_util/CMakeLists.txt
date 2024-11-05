cmake_minimum_required(VERSION 3.12)

project(affordance_util)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set library output path
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include Eigen library
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
include_directories(${EIGEN3_INCLUDE_DIR})

# Find necessary cmake packages
find_package(yaml-cpp REQUIRED)
find_package(urdfdom REQUIRED)

# Define the affordance_util library
add_library(${PROJECT_NAME} STATIC src/${PROJECT_NAME}/${PROJECT_NAME}.cpp)

# Link libraries against the affordance_util library and make them available to downstream packages
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp urdfdom_model)

# Add include directories with different paths for build and install contexts
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install the library
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
)

# Generate and install CMake package configuration files. The following steps enable finding this library with the find_package call in downstream packages
include(CMakePackageConfigHelpers)

# Define the version of the package
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.0)

# Generate a configuration file for find_package()
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE affordance_util::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Print the top-level source directory
message("Top-level source directory: ${CMAKE_SOURCE_DIR}")

# Print the source directory of the current CMakeLists.txt file
message("Current CMake file's source directory: ${CMAKE_CURRENT_SOURCE_DIR}")

# Create the main package configuration file
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Add the affordance_util library tester and link it against the affordance_util library
add_executable(${PROJECT_NAME}_tester src/${PROJECT_NAME}_tester.cpp)
target_link_libraries(${PROJECT_NAME}_tester ${PROJECT_NAME})
