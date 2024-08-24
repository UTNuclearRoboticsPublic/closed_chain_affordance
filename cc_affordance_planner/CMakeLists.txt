
cmake_minimum_required(VERSION 3.12)

project(cc_affordance_planner)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set library output path
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include eigen library
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
include_directories(include ${EIGEN3_INCLUDE_DIR})

# Find the affordance_util library
FIND_LIBRARY(affordance_util_LIBRARIES affordance_util /usr/local/lib)

# Find the yaml-cpp library (although not directly used, it is needed during linking since affordance_util uses it)
find_package(yaml-cpp REQUIRED )

# Create the library with source files
add_library(${PROJECT_NAME} STATIC
    src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
    src/${PROJECT_NAME}/${PROJECT_NAME}_interface.cpp
)

# Link this package's library against affordance_util
target_link_libraries(${PROJECT_NAME} affordance_util yaml-cpp pthread)

# Add include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Set the library installation path
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
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

# Library Tester
add_executable(${PROJECT_NAME}_tester src/${PROJECT_NAME}_tester.cpp)
target_link_libraries(${PROJECT_NAME}_tester ${PROJECT_NAME})
