cmake_minimum_required(VERSION 2.8.3)
project(curio_firmware)

find_package(catkin REQUIRED
    COMPONENTS
    curio_msgs
)

catkin_package(
    CATKIN_DEPENDS
    curio_msgs
)


