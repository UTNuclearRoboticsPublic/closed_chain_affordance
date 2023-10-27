cmake_minimum_required(VERSION 3.12)

project(affordance_util)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set library output path
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include eigen library
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
include_directories(${EIGEN3_INCLUDE_DIR})

# Create the library with source files
add_library(${PROJECT_NAME} STATIC
    src/${PROJECT_NAME}.cpp
)

# Add include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Set the library installation path
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
)

# Install header files
install(FILES include/${PROJECT_NAME}/${PROJECT_NAME}.hpp
    DESTINATION include/${PROJECT_NAME}
)

# Library Tester
add_executable(${PROJECT_NAME}_tester src/${PROJECT_NAME}_tester.cpp)
target_link_libraries(${PROJECT_NAME}_tester ${PROJECT_NAME})
