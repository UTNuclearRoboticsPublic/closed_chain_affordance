cmake_minimum_required(VERSION 3.12)

project(cc_affordance_planner)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set library output path
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include Eigen library
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
include_directories(include ${EIGEN3_INCLUDE_DIR})

# Find the affordance_util package
find_package(affordance_util REQUIRED)

# Create the library with source files
add_library(${PROJECT_NAME} STATIC
    src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
    src/${PROJECT_NAME}/${PROJECT_NAME}_interface.cpp
)

# Link this package's library against affordance_util and pthread
target_link_libraries(${PROJECT_NAME} PUBLIC affordance_util::affordance_util pthread)

# Add include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Set the library installation path
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install the header files
install(FILES include/${PROJECT_NAME}/${PROJECT_NAME}.hpp
              include/${PROJECT_NAME}/${PROJECT_NAME}_interface.hpp
        DESTINATION include/${PROJECT_NAME}
)

# Install jthread headers
install(DIRECTORY include/jthread
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp"
)

# Generate and install CMake package configuration files. The following steps enable finding this library with the find_package call in downstream packages
include(CMakePackageConfigHelpers)

# Define the version of the package
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.0)

# Generate the Targets file for find_package()
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE cc_affordance_planner::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Create the main package configuration file
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

# Install the generated Config.cmake file
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        DESTINATION lib/cmake/${PROJECT_NAME}
)

# Add the cc_affordance_planner library tester and link it against the cc_affordance_planner library
add_executable(${PROJECT_NAME}_tester src/${PROJECT_NAME}_tester.cpp)
target_link_libraries(${PROJECT_NAME}_tester ${PROJECT_NAME})
