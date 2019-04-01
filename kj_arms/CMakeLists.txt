cmake_minimum_required(VERSION 2.8.3)
project(kj_arms)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs trajectory_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ssc32_driver_kj_arms ssc32_kj_arms
   CATKIN_DEPENDS roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(ssc32_kj_arms src/ssc32_kj_arms.cpp)
target_link_libraries(ssc32_kj_arms ${catkin_LIBRARIES})

add_library(ssc32_driver_kj_arms src/ssc32_driver_kj_arms.cpp)
target_link_libraries(ssc32_driver_kj_arms ssc32_kj_arms ${catkin_LIBRARIES})

add_executable(ssc32_node_kj_arms src/ssc32_node_kj_arms.cpp)
target_link_libraries(ssc32_node_kj_arms ssc32_driver_kj_arms ${catkin_LIBRARIES})

add_executable(test_ssc32_kj_arms src/ssc32_test_kj_arms.cpp)
target_link_libraries(test_ssc32_kj_arms ssc32_driver_kj_arms ${catkin_LIBRARIES})

#add_executable(pub_joints src/pub_joints.cpp)
#target_link_libraries(pub_joints ${catkin_LIBRARIES})

#add_executable(pub_two_joints_one_control src/pub_two_joints_one_control.cpp)
#target_link_libraries(pub_two_joints_one_control ${catkin_LIBRARIES})

#add_executable(pub_moveit src/pub_moveit.cpp)
#target_link_libraries(pub_moveit ${catkin_LIBRARIES})

#add_executable(sub_pub_moveit src/sub_pub_moveit.cpp)
#target_link_libraries(sub_pub_moveit ${catkin_LIBRARIES})

install(TARGETS ssc32_kj_arms ssc32_driver_kj_arms ssc32_node_kj_arms test_ssc32_kj_arms
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
