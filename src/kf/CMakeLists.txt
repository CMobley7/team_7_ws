cmake_minimum_required(VERSION 2.8.3)
project(kf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)

# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
find_package(PkgConfig REQUIRED)
# the documentation is wrong
pkg_check_modules(BFL REQUIRED orocos-bfl)
#pkg_check_modules(BFL REQUIRED bfl)
include_directories(${BFL_INCLUDE_DIRS}/bfl)
message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pf
#  CATKIN_DEPENDS bfl geometry_msgs nav_msgs ros_cpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${BFL_INCLUDE_DIRS}
)

# TODO: Uncomment the following 4 lines after pf_py branch merges in to the pf
# add_executable(particle_filter src/particle_filter.cpp)
# add_dependencies(particle_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(particle_filter
#	${catkin_LIBRARIES}
#	${BFL_LIBRARY_DIRS}
#)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   std_msgs
# )
