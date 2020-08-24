## 经典生成器
FIND_PACKAGE(tf REQUIRED)

SET(ROS_LIB_PATH /opt/ros/kinetic/lib)


FIND_PACKAGE(REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_perception
  moveit
  moveit_msgs
)

LINK_DIRECTORIES(${ROS_LIB_PATH})

INCLUDE_DIRECTORIES(${tf_INCLUDE_DIRS})

ADD_LIBRARY(Actuator adjuster.cpp classic_actuator.cpp )

TARGET_LINK_LIBRARIES(Actuator
    tf
    moveit_planning_interface
    moveit_planning_scene_interface
    moveit_move_group_interface
    moveit_collision_detection
    moveit_kinematic_constraints
    HPlugin
    yaml-cpp
    moveit_trajectory_processing
    )



INSTALL(TARGETS Actuator DESTINATION lib)

############################ Start Simaple Generator ############################

#ADD_LIBRARY(ClassicGenerator SHARED simaple_generator.cpp)
#INSTALL(TARGETS ClassicGenerator DESTINATION lib)

############################ End Simaple Generator ############################
