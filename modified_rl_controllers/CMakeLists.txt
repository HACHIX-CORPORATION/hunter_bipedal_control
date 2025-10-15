cmake_minimum_required(VERSION 3.10)
project(modified_rl_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_common
        legged_estimation
        legged_interface
        dynamic_reconfigure
        controller_interface
        angles
        )

# Manually configure onnxruntime for catkin compatibility
set(onnxruntime_INCLUDE_DIRS "/opt/ros/noetic/include/onnxruntime")
set(onnxruntime_LIBRARIES "/opt/ros/noetic/lib/libonnxruntime.so")
set(onnxruntime_FOUND TRUE)

# Create imported target for onnxruntime
add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES
    IMPORTED_LOCATION "/opt/ros/noetic/lib/libonnxruntime.so"
    INTERFACE_INCLUDE_DIRECTORIES "/opt/ros/noetic/include/onnxruntime"
)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

generate_dynamic_reconfigure_options(  
        cfg/Tutorials.cfg
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        legged_common
        legged_estimation
        legged_interface
        dynamic_reconfigure
        controller_interface
        angles
        DEPENDS
        onnxruntime
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${onnxruntime_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(${pinocchio_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
        src/CustomAcController.cpp
        src/RLControllerBase.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        onnxruntime
        )

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} legged_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
  
# Mark other files for installation
install(
        FILES custom_ac_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)