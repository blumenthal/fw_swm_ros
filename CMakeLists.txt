cmake_minimum_required(VERSION 2.8.3)
project(swm_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# BINARIES  #
############
add_definitions(-std=c++11)

cs_add_library(${PROJECT_NAME}_lib src/main.cc )
cs_add_executable(${PROJECT_NAME} src/main.cc src/swm_ros.cc)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME} /usr/local/lib/libswmzyre.so ${catkin_LIBRARIES} )

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
