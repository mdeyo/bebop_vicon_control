cmake_minimum_required(VERSION 2.8.3)
project(bebop_vicon_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation)

  add_service_files(
    FILES
    FlyTo.srv
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
  )

catkin_package()

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   teleop_twist_keyboard.py
   bebop_controller.py
   bebop_geofence.py
   update_goal_keyboard.py
   bebop_api_server.py
   bebop_api_client.py
   autonomous_landing.py

   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
