cmake_minimum_required(VERSION 2.8.3)
project(iiwa_li)

find_package(catkin REQUIRED COMPONENTS
  iiwa_msgs
  tf
  cmake_modules
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


