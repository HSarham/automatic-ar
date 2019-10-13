add_library(filesystem filesystem.cpp)

add_library(cam_config cam_config.cpp)
target_link_libraries(cam_config filesystem)

add_library(multicam_mapper multicam_mapper.cpp)
target_link_libraries(multicam_mapper aruco_serdes)
target_link_libraries(multicam_mapper initializer)
target_link_libraries(multicam_mapper pointcloud_drawer)
target_link_libraries(multicam_mapper sgl)

add_library(dataset dataset.cpp)

add_library(image_array_detector image_array_detector.cpp)

add_library(initializer initializer.cpp)
target_link_libraries(initializer aruco_serdes)

add_library(aruco_serdes aruco_serdes.cpp)

add_library(pointcloud_drawer pointcloud_drawer.cpp)
