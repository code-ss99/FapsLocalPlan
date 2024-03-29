cmake_minimum_required(VERSION 2.8.3)
project(test_plugin)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            costmap_2d
            nav_core
            nav_msgs
            pluginlib
            pcl_conversions
            roscpp
            tf
            xform_utils
        )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
add_definitions(${Eigen3_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES test_plugin
    CATKIN_DEPENDS
        pluginlib
        roscpp
)

add_library(test_planner 
	    src/test_planner.cpp)
##target_link_libraries(test_planner xform_utils ${catkin_LIBRARIES})
target_link_libraries(test_planner xform_utils  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_dependencies(test_planner nav_msgs_gencpp)

install(TARGETS test_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )
       
install(FILES test_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
 



#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#catkin_package()

#Library
#Just make the one library, the plugin manager will handle the rest.
#add_library(test_planner src/test_planner.cpp)
#target_link_libraries(test_planner xform_utils  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#include_directories(include)



