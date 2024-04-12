cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED roscpp std_msgs rotate_recovery gazebo_msgs genmsg geometry_msgs nav_msgs tf2_ros global_planner move_base base_local_planner clear_costmap_recovery nav_core tf2_geometry_msgs)
catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp nodelet
#   DEPENDS elektron
)

include_directories(include ${catkin_INCLUDE_DIRS})
install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
install (DIRECTORY param/ DESTINATION share/${CMAKE_PROJECT_NAME}/param 
         FILES_MATCHING PATTERN "*.yaml")

        add_executable(lab1 src/lab1.cpp)
        target_link_libraries(lab1 ${catkin_LIBRARIES})
        add_executable(lab1_er src/lab1_er.cpp)
        target_link_libraries(lab1_er ${catkin_LIBRARIES})
        add_executable(proj2 src/proj2.cpp)
        target_link_libraries(proj2 ${catkin_LIBRARIES})
         