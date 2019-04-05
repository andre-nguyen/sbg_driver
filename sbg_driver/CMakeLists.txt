cmake_minimum_required(VERSION 2.8.3)
project(sbg_driver)

find_package(catkin REQUIRED COMPONENTS
             message_filters
             roscpp
             sara_msgs
             sbg_msgs
             std_srvs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_filters roscpp sara_msgs sbg_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########
add_subdirectory(external/sbgECom)

SET(COMMON_WARNINGS
        -Wall
        -Wpedantic
        -Wextra
        -Werror)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM
        ${PROJECT_SOURCE_DIR}/external/sbgECom/common
        ${PROJECT_SOURCE_DIR}/external/sbgECom/src)

## Declare a C++ library
add_library(${PROJECT_NAME} STATIC
            src/sbg_driver.cpp
            src/sbg_ecom_ros_conversions.cpp
            src/external_timestamping.cpp
            src/sequential_stamping.cpp src/stamp_matcher.cpp include/sbg_driver/stamp_matcher.h)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}
                 sbgECom
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
                      sbgECom)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)

target_compile_options(${PROJECT_NAME} PRIVATE
        ${COMMON_WARNINGS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
        src/sbg_driver_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

target_compile_options(${PROJECT_NAME}_node PRIVATE
        ${COMMON_WARNINGS})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
                      ${PROJECT_NAME}
                      ${catkin_LIBRARIES})

set_property(TARGET ${PROJECT_NAME}_node PROPERTY CXX_STANDARD 14)

add_library(stamp_matcher src/stamp_matcher.cpp)
target_link_libraries(stamp_matcher ${catkin_LIBRARIES})
target_compile_options(stamp_matcher PRIVATE ${COMMON_WARNINGS})

add_executable(stamp_matcher_node src/stamp_matcher_node.cpp)
target_link_libraries(stamp_matcher_node stamp_matcher)
target_compile_options(stamp_matcher_node PRIVATE ${COMMON_WARNINGS})




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
