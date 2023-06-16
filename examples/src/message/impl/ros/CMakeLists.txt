find_package(Eigen3 REQUIRED )
message(EIGEN3_INCLUDE_DIRS : ${EIGEN3_INCLUDE_DIRS} )

add_library(ros_message_manager SHARED RosMessageManager.cpp)
add_ros(ros_message_manager roscpp)
target_link_libraries(ros_message_manager PUBLIC tf )
target_include_directories(ros_message_manager PUBLIC
        ${EIGEN3_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../include
        ${CMAKE_CURRENT_LIST_DIR}

        ${EIGEN3_INCLUDE_DIR})

#set_asan(ros_message_manager )

install_target(ros_message_manager)
