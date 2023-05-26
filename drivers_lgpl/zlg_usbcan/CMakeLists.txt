cmake_minimum_required(VERSION 2.8)

message(STATUS "Configuring zlg_usbcan driver")

set(LIB_NAME can_zlg_usbcan)

file(GLOB LIB_SRC
	"*.c"
)

add_library(${LIB_NAME} SHARED ${LIB_SRC})

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED TRUE)

set_property(TARGET ${LIB_NAME} PROPERTY C_STANDARD 11)

target_link_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}   )
# usbcan : zlg driver
target_link_libraries(${LIB_NAME} PUBLIC usbcan )
#set_asan(${LIB_NAME} )
set_target_properties(${LIB_NAME} PROPERTIES
		BUILD_WITH_INSTALL_RPATH FALSE
		LINK_FLAGS "-Wl,-rpath,$ORIGIN/../lib")
# Install
kacanopen_install(${LIB_NAME} FALSE)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/libusbcan.so
		DESTINATION lib
		)
