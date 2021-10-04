cmake_minimum_required(VERSION 3.0.2)
project(tut)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  turtlesim
)

add_message_files(
  FILES
  Number.msg
)

add_service_files(
  FILES
  Adder.srv
)

add_action_files(
  FILES
  Count.action
  Fibonacci.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
	actionlib_msgs 
	# ...
)

catkin_package(
 CATKIN_DEPENDS actionlb_msgs actionlib geometry_msgs message_runtime roscpp rospy std_msgs #turtlesim 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


foreach(node_name IN ITEMS spiral helloworld)
  add_executable(${node_name} src/${node_name}.cpp)
endforeach()

foreach(node_name IN ITEMS talker listener)
  add_executable(${node_name} src/topic/${node_name}.cpp)
endforeach()

foreach(node_name IN ITEMS adder_srv adder_client)
  add_executable(${node_name} src/service/${node_name}.cpp)
endforeach()

foreach(node_name IN ITEMS count_actsrv count_actclient)
  add_executable(${node_name} src/act/${node_name}.cpp)
endforeach()

add_executable(param_test src/param/param_test.cpp)

foreach(node_name IN ITEMS spiral helloworld talker listener param_test adder_srv adder_client count_actsrv count_actclient)
  add_dependencies(${node_name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${node_name} ${catkin_LIBRARIES})
endforeach()

