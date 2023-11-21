cmake_minimum_required(VERSION 2.8.3)
project(rly_08)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg robotnik_msgs)
find_package(Boost REQUIRED)

add_message_files(FILES rly08_io_msg.msg)
add_service_files(FILES GetRelayStatus.srv  GetSwVersion.srv  SetRelayStatus.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp std_msgs self_test diagnostic_updater robotnik_msgs
   DEPENDS messages_runtime roscpp std_msgs)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})



# in order to include the messages headers of the same package messages
add_executable(rly_08_node src/Component.cc src/SerialDevice.cc src/rly08.cc src/rly08_node.cpp)

add_dependencies(rly_08_node rly_08_gencpp)
add_dependencies(rly_08_node robotnik_msgs_gencpp)

target_link_libraries(rly_08_node ${catkin_LIBRARIES})


