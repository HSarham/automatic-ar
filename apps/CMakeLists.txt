add_executable(detect_markers detect_markers.cpp)
target_link_libraries(detect_markers multicam_mapper dataset cam_config aruco ${pcl_libs} ${OpenCV_LIBS})

add_executable(find_solution find_solution.cpp)
target_link_libraries(find_solution cam_config initializer multicam_mapper aruco ${pcl_libs} ${OpenCV_LIBS})

add_executable(overlay overlay.cpp)
target_link_libraries(overlay multicam_mapper dataset cam_config aruco ${pcl_libs} ${OpenCV_LIBS})

add_executable(track track.cpp)
target_link_libraries(track multicam_mapper dataset cam_config image_array_detector aruco ${pcl_libs} ${OpenCV_LIBS})
