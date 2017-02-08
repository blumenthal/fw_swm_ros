cmake_minimum_required(VERSION 2.8.3)
project(swm_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# BINARIES  #
############
add_definitions(-std=c++11)

cs_add_executable(${PROJECT_NAME}_rosbag src/node_rosbag.cc src/swm_ros.cc)
cs_add_executable(${PROJECT_NAME}_rosstream src/node_rosstream.cc src/swm_ros.cc )

add_dependencies(${PROJECT_NAME}_rosbag ${${PROJECT_NAME}_EXPORTED_TARGETS}})
add_dependencies(${PROJECT_NAME}_rosstream ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_rosbag /usr/local/lib/libswmzyre.so ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_rosstream /usr/local/lib/libswmzyre.so ${catkin_LIBRARIES} )

add_dependencies(${PROJECT_NAME}_rosstream ${PROJECT_NAME}_rosstream_gencpp)

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

##########
# EXPORT #
##########
cs_install()
cs_export()
