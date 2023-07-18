cmake_minimum_required(VERSION 3.2)
project(autopilot_control_system)

find_library(UAVCAN_LIB uavcan REQUIRED)

#set(CMAKE_CXX_FLAGS "-Wall -Wextra -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11 -O3")  # Enable compiler optimizations with -O3 flag


set(UAVCAN_DSDL_DEFINITIONS "/usr/local/share/uavcan/dsdl/uavcan")      # For Linux, if the library is installed

add_custom_target(dsdlc libuavcan_dsdlc                                      # If the library is installed
                  "./dsdl_source" -I${UAVCAN_DSDL_DEFINITIONS}
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

include_directories("dsdlc_generated") # Adding the default output directory to the include paths

# Add libfort library
include_directories("/usr/local/include")  # Path to the libfort header files

# Add libfort library path
link_directories("/usr/local/lib/libfort.a")  # Path to the libfort library binary

add_executable(publisher_sensors publisher_sensors.cpp ${CMAKE_SOURCE_DIR}/../../2._Node_initialization_and_startup/platform_linux.cpp)
target_link_libraries(publisher_sensors ${UAVCAN_LIB} rt fort)
add_dependencies(publisher_sensors dsdlc)

add_executable(subscriber_sensors subscriber_sensors.cpp ${CMAKE_SOURCE_DIR}/../../2._Node_initialization_and_startup/platform_linux.cpp)
target_link_libraries(subscriber_sensors ${UAVCAN_LIB} rt fort)
add_dependencies(subscriber_sensors dsdlc)

