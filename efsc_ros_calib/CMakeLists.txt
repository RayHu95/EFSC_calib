cmake_minimum_required(VERSION 3.14)
project(efsc_ros_calib)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

find_package(
  catkin REQUIRED COMPONENTS 
    message_generation
    tf
    image_transport
    roscpp
    std_msgs
    sensor_msgs
    visualization_msgs
    dv_ros_msgs
    dv_ros_messaging
    dynamic_reconfigure
    pcl_ros
)

generate_dynamic_reconfigure_options(
	cfg/calib.cfg
)


find_package(dv-processing 1.4.0 REQUIRED)
include_directories("/usr/include/eigen3")
set(nanoflann_DIR /usr/local/include/cmake/nanoflann) #change this address if "nanoflannConfig.cmake" is not here.
find_package(nanoflann REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED) # not strictly neccesary
find_package(OpenCV 4.0 REQUIRED)
find_package(FMT REQUIRED)
find_package(kindr REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)

LIST(APPEND CMAKE_MODULE_PATH /home/ray/Third_party/g2o/cmake_modules )
SET(G2O /usr/local/include/g2o)
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})


include_directories(include ${catkin_INCLUDE_DIRS})
# add_subdirectory(ef_slambase)

catkin_package(
  CATKIN_DEPENDS
	message_runtime
	roscpp
	std_msgs
	sensor_msgs
	dv_ros_msgs
	dv_ros_messaging
	pcl_ros
)

add_executable(EFSC_calib
		src/calib_node.cpp 
		src/subtask/task.cpp
		src/subtask/TwoViewRecover.cpp
		src/subtask/EventImageWithMotion.cpp
		
		src/ef_calib/dbscan/kdtree.cpp
		src/ef_calib/picker/EventPicker.cpp
		src/ef_calib/picker/EventSlice.cpp
		src/ef_calib/picker/ImageSlice.cpp
		src/ef_calib/calib/EventCalib.cpp
		src/ef_calib/calib/PoseCalib_g2o.cpp

		src/tools/tools.cpp
		)


add_dependencies(EFSC_calib ${catkin_EXPORTED_TARGETS}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg
				 ${tf_broadcaster_EXPORTED_TARGETS})

target_link_libraries(EFSC_calib 
			${catkin_LIBRARIES} 
			dv::processing 
			nanoflann::nanoflann
			Boost::container
			${CERES_LIBRARIES}
			${OpenCV_LIBS}
			${Sophus_INCLUDE_DIRS}
			# ${G2O_CORE_LIBRARIES}  ${G2O_STUFF_LIBRARIES} 
			g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
			)
