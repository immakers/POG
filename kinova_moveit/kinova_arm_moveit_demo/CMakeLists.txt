cmake_minimum_required(VERSION 2.8.3)
project(kinova_arm_moveit_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS
             moveit_core
             moveit_ros_planning
             moveit_ros_planning_interface
             pluginlib
             cmake_modules
             geometric_shapes
             ## add
             roscpp
             rospy
             std_msgs
             message_generation
)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)
find_package(kinova_driver REQUIRED)

#######################################
## msg add change by qcrong 20180416 ##
#######################################
add_message_files(
	FILES
	targetState.msg
	targetsVector.msg
        toolposeChange.msg
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  CATKIN_DEPENDS message_runtime  #add by qcrong 20180416
)


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(${kinova_driver_INCLUDE_DIRS})

add_executable(motion_plan src/motion_plan.cpp)
target_link_libraries(motion_plan ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS motion_plan DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## added by petori in 2018/4/2
add_executable(simple_move src/simple_move.cpp)
target_link_libraries(simple_move ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS simple_move DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
## end

## added by petori in 2018/4/3
add_executable(talker1 src/talker1.cpp)
target_link_libraries(talker1 ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS talker1 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
## end

## added by qcrong in 2018/4/8
add_executable(eye2hand_calib_control src/eye2hand_calib_control.cpp)
target_link_libraries(eye2hand_calib_control ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS eye2hand_calib_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
## end

## added by qcrong in 2018/4/17
add_executable(our_pick_place src/our_pick_place.cpp)
target_link_libraries(our_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS our_pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
## end
## added by qcrong in 2018/5/25
add_executable(visual_detect_simulate src/visual_detect_simulate.cpp)
target_link_libraries(visual_detect_simulate ${catkin_LIBRARIES} )
install(TARGETS visual_detect_simulate DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
## end

add_executable(work_scene src/work_scene.cpp)
target_link_libraries(work_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS work_scene DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pick_place src/pick_place.cpp include/pick_place.h)
target_link_libraries(pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_accuracy src/test_accuracy.cpp include/test_accuracy.h)
target_link_libraries(test_accuracy ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS test_accuracy DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(finger_control src/finger_control.cpp)
target_link_libraries(finger_control ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS finger_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})   #added by yang 20180417


#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
