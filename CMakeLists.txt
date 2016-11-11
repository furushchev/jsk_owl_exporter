cmake_minimum_required(VERSION 2.8.3)
project(jsk_owl_exporter)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

# add_service_files(
#   FILES
#   Export.srv
# )

# generate_messages(
#   DEPENDENCIES std_msgs
# )

catkin_package()

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
