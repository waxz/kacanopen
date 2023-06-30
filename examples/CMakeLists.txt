cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # use -std=c++... instead of -std=gnu++...
set(CMAKE_CXX_STANDARD  14)
include(../cmake/common.cmake)

include(../cmake/set_ros.cmake)
include(../cmake/super_build.cmake)

#abseil-cpp-20220623.1.tar.gz
build_external_project(absl lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/abseil-cpp-20220623.1.tar.gz -DABSL_BUILD_TESTING=OFF -DABSL_USE_GOOGLETEST_HEAD=ON -DCMAKE_CXX_STANDARD=14  -DABSL_ENABLE_INSTALL=ON)
find_package(absl REQUIRED)

build_external_project(mosquitto lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mosquitto-2.0.15.tar.gz -DCMAKE_BUILD_TYPE=Release  -DDOCUMENTATION=OFF) #-DcJSON_DIR=${cJSON_DIR}
set(mosquitto_LIBRARY ${mosquitto_ROOT}/lib/libmosquitto.so ${mosquitto_ROOT}/lib/libmosquittopp.so)
set(mosquitto_INCLUDE_DIR ${mosquitto_ROOT}/include)


# https://github.com/openresty/luajit2
build_external_project_command(luajit lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/luajit2-2.1-20230410.tar.gz "mkdir -p ${CMAKE_BINARY_DIR}/force_luajit/install/bin" "make PREFIX=${CMAKE_BINARY_DIR}/force_luajit/install" "make install PREFIX=${CMAKE_BINARY_DIR}/force_luajit/install")
set(LUA_LIBRARIES ${luajit_ROOT}/lib/libluajit-5.1.a)
set(LUA_INCLUDE_DIR ${luajit_ROOT}/include/luajit-2.1)

add_library(liblua STATIC IMPORTED)
set_target_properties(liblua PROPERTIES  IMPORTED_LOCATION "${LUA_LIBRARIES}")
target_include_directories(liblua INTERFACE ${LUA_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/third_party/sol)
target_link_libraries(liblua INTERFACE  dl )

build_external_project(Eigen3 lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.4.0.zip  -DCMAKE_BUILD_TYPE=Release )
message(Eigen3_ROOT : ${Eigen3_ROOT} )

set(Eigen3_ROOT ${Eigen3_ROOT}/share/eigen3/cmake)
find_package(Eigen3 3.4.0 REQUIRED HINTS ${Eigen3_ROOT})
message(new eigen: EIGEN3_INCLUDE_DIRS : ${EIGEN3_INCLUDE_DIRS} )


build_external_project(Ceres lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ceres-solver-2.1.0.tar.gz  -DEigen3_ROOT=${Eigen3_ROOT} -DCMAKE_BUILD_TYPE=Release -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCUMENTATION=OFF)
find_package(Ceres  REQUIRED)
message(Ceres_ROOT : ${Ceres_ROOT} ,CERES_LIBRARIES : ${CERES_LIBRARIES} )



add_subdirectory(src/message/impl/ros)
add_subdirectory(src/message/impl/mqtt)


set(shared_dirs ${mosquitto_ROOT}/lib ${Ceres_ROOT}/lib)
foreach(dir ${shared_dirs})
	if(EXISTS "${dir}" AND IS_DIRECTORY "${dir}")
		message("EXISTS：${dir}")
		#Installing roslaunch Files or Other Resources
		install(DIRECTORY ${dir}/
				DESTINATION lib
				PATTERN ".svn" EXCLUDE)
	endif()
endforeach()




message(STATUS "Configuring examples")

file(GLOB TARGETS_SRC
	"*.cpp"
#	"test/*.cpp"
)

if (NOT(${NO_ROS}))
	#list(REMOVE_ITEM TARGETS_SRC "${CMAKE_CURRENT_LIST_DIR}/ros_*")
	file(GLOB TARGETS_SRC_ROS
		"ros/*.cpp"
	)
	list(APPEND TARGETS_SRC ${TARGETS_SRC_ROS})
endif()

find_package(Eigen3 REQUIRED )
message(new eigen: EIGEN3_INCLUDE_DIRS : ${EIGEN3_INCLUDE_DIRS} )
#https://stackoverflow.com/questions/43330165/how-to-link-a-shared-library-with-cmake-with-relative-path/69707790#69707790

add_library(controller SHARED  src/control/MobileRobotController.cpp src/control/MotionPlanner.cpp)
set_property(TARGET controller PROPERTY CXX_STANDARD 14)
target_link_libraries(controller PUBLIC  absl::strings  absl::str_format ${mosquitto_LIBRARY} ros_message_manager mqtt_message_manager ${CERES_LIBRARIES})
target_include_directories(controller PUBLIC include src  ${EIGEN3_INCLUDE_DIRS}   )
set_target_properties(controller PROPERTIES
		BUILD_WITH_INSTALL_RPATH FALSE
		LINK_FLAGS "-Wl,-rpath,$ORIGIN/../lib")

#set_asan(controller )

kacanopen_install(controller FALSE)

foreach(TARGET_SRC ${TARGETS_SRC})

	get_filename_component(TARGET_NAME ${TARGET_SRC} NAME_WE)
	set(TARGET_NAME "kacanopen_example_${TARGET_NAME}")
	add_executable(${TARGET_NAME} ${TARGET_SRC}   )
	set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 14)
	target_link_libraries(${TARGET_NAME} PUBLIC kacanopen_core absl::strings   ${mosquitto_LIBRARY} ros_message_manager mqtt_message_manager controller ${LUA_LIBRARIES})
	target_include_directories(${TARGET_NAME} PUBLIC include src  ${EIGEN3_INCLUDE_DIRS} third_party/sol ${LUA_INCLUDE_DIR} )

	add_ros(${TARGET_NAME} roscpp)
	target_link_libraries(${TARGET_NAME} PUBLIC tf )
	set_target_properties(${TARGET_NAME} PROPERTIES
			BUILD_WITH_INSTALL_RPATH FALSE
			LINK_FLAGS "-Wl,-rpath,$ORIGIN/../lib")

#	set_asan(${TARGET_NAME} )
#	target_link_libraries(${TARGET_NAME} kacanopen_master)
#	set_target_rpath(${TARGET_NAME})

	if (NOT(${NO_ROS}))
		target_link_libraries(${TARGET_NAME} kacanopen_ros_bridge)
	endif()
	
	list(APPEND KACANOPEN_EXAMPLES_TARGET_NAMES ${TARGET_NAME})

endforeach(TARGET_SRC ${TARGETS_SRC})

# Install
set(INSTALL_EXAMPLES TRUE)
if (${INSTALL_EXAMPLES})
	kacanopen_install("${KACANOPEN_EXAMPLES_TARGET_NAMES}" FALSE)
endif()

