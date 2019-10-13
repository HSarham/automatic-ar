INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
add_executable(pcd_viewer pcd_viewer.cpp)
add_executable(dynamic_scene dynamic_scene.cpp)
target_link_libraries(pcd_viewer sgl  )
target_link_libraries(dynamic_scene sgl  )

#aruco_create_board
INSTALL(TARGETS pcd_viewer RUNTIME DESTINATION bin)
