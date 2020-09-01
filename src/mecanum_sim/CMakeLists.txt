cmake_minimum_required(VERSION 2.8)
project(mecanum_sim CXX C)
set(CMAKE_CXX_FLAG “-std=c++11” ${CMAKE_CXX_FLAGS})
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_msgs
  moveit_ros_planning_interface
  message_generation
  tf
  pcl_ros
  )

set(ACADO_DIR "/home/weili/ACADOtoolkit/build")
set(ACADO_ROOT "/home/weili/ACADOtoolkit")

include_directories(${ACADO_ROOT})
include_directories(${ACADO_ROOT}/acado)
include_directories(${ACADO_ROOT}/acado/utils)

link_directories(
  /home/weili/ACADOtoolkit/build/lib
  )

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ACADO)
find_package(OpenCV REQUIRED)##use opencv 2.4.8

add_service_files(
  FILES
  DRLStep.srv
  NMPCStep.srv
)

##very important!##
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_msgs
   CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
#  DEPENDS system_lib
)


include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_library(mecanum SHARED src/mecanum_controller.cc)
target_link_libraries(mecanum ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY worlds
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_node src/test_cylinder.cpp)
add_dependencies(test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_node ${catkin_LIBRARIES})

include_directories(/home/weili/catkin_ws/src/mecanum_sim/include)
add_executable(sample_based_move_node src/sample_based_move.cpp src/FoliTreeNode.cpp src/FoliTree.cpp src/JacobianPoint.cpp)
add_dependencies(sample_based_move_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sample_based_move_node 
${catkin_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libopencv_core.so)

add_executable(test_arm_node src/test_cylinder2.cpp)
add_dependencies(test_arm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_arm_node ${catkin_LIBRARIES})

include_directories(/home/weili/catkin_ws/src/mecanum_sim/include)
add_executable(step_server_node src/step_server.cpp src/PositionAll.cpp)
add_dependencies(step_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(step_server_node ${catkin_LIBRARIES})


add_executable(nmpc_server_node src/nmpc_server.cpp)
add_dependencies(nmpc_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nmpc_server_node
    ${catkin_LIBRARIES}
    /home/weili/ACADOtoolkit/build/lib/libacado_toolkit_s.so
    /home/weili/ACADOtoolkit/build/lib/libacado_casadi.a
    /home/weili/ACADOtoolkit/build/lib/libacado_csparse.a
    /home/weili/ACADOtoolkit/build/lib/libacado_qpoases.a
)

include_directories(/home/weili/catkin_ws/src/mecanum_sim/include)
add_executable(nmpc_client_node src/nmpc_client.cpp src/JacobianPoint.cpp)
add_dependencies(nmpc_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nmpc_client_node
    ${catkin_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libopencv_core.so
)

include_directories(/home/weili/catkin_ws/src/mecanum_sim/include)
include_directories(/home/weili/V-REP_PRO_EDU_V3_4_0_Linux/programming/include)
include_directories(/home/weili/V-REP_PRO_EDU_V3_4_0_Linux/programming/remoteApi)
add_executable(nmpc_move_node src/nmpc_move.cpp src/JacobianPoint.cpp)
add_dependencies(nmpc_move_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nmpc_move_node
    ${catkin_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libopencv_core.so
    /home/weili/ACADOtoolkit/build/lib/libacado_toolkit_s.so
    /home/weili/ACADOtoolkit/build/lib/libacado_casadi.a
    /home/weili/ACADOtoolkit/build/lib/libacado_csparse.a
    /home/weili/ACADOtoolkit/build/lib/libacado_qpoases.a
)

include_directories(/home/weili/catkin_ws/src/mecanum_sim/include)
add_executable(move_arm_node src/move_arm.cpp src/JacobianPoint.cpp)
add_dependencies(move_arm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_arm_node
    ${catkin_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libopencv_core.so
)

add_executable(test_pointcloud_node src/test_pointcloud.cpp)
add_dependencies(test_pointcloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pointcloud_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES})
