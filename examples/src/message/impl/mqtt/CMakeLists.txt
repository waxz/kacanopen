find_package(Eigen3 REQUIRED )
message(EIGEN3_INCLUDE_DIRS : ${EIGEN3_INCLUDE_DIRS} )

add_library(mqtt_message_manager SHARED MqttMessageManager.cpp MqttClient.cpp)
target_link_libraries(mqtt_message_manager PUBLIC
        ${mosquitto_LIBRARY}
        )
target_include_directories(mqtt_message_manager PUBLIC
        ${EIGEN3_INCLUDE_DIRS}
        ${mosquitto_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../include
        ${EIGEN3_INCLUDE_DIR})

#set_target_rpath(mqtt_message_manager)
set_target_properties(mqtt_message_manager PROPERTIES
        BUILD_WITH_INSTALL_RPATH FALSE
        LINK_FLAGS "-Wl,-rpath,$ORIGIN/../lib")
#set_asan(mqtt_message_manager )

install_target(mqtt_message_manager)