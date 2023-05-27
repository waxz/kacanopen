cmake_minimum_required(VERSION 2.8)

message(STATUS "Configuring core")

find_package(Threads)

file(GLOB TARGET_SRC
	"src/*.cpp"
)

# Create library
set(TARGET_NAME kacanopen_core)
add_library(${TARGET_NAME} SHARED ${TARGET_SRC})
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 14)
target_include_directories(${TARGET_NAME} PUBLIC include  )
#target_link_libraries(${TARGET_NAME} ${CAN_DRIVER})
target_link_libraries(${TARGET_NAME} PUBLIC ${DRIVER_LINK_NAME})
target_link_libraries(${TARGET_NAME} PUBLIC ${CMAKE_DL_LIBS})
target_link_libraries(${TARGET_NAME} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
set_target_properties(${TARGET_NAME} PROPERTIES
		BUILD_WITH_INSTALL_RPATH FALSE
		LINK_FLAGS "-Wl,-rpath,$ORIGIN/../lib")
# Install
kacanopen_install(${TARGET_NAME} TRUE)
