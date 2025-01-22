find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        message_generation
        sensor_msgs
        nav_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)