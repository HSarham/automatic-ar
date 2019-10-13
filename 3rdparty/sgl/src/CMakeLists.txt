SET(LIBNAME sgl)
set(sources  sgldisplay.cpp )
set(headers  sgldisplay.h  sgl.h  sgl_exports.h   )

IF(Qt5Widgets_FOUND)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(sources_QT sgldisplay_qt.cpp sgldisplay_qt_internal.cpp)
set(headers_QT sgldisplay_qt.h sgldisplay_qt_internal.h)
add_definitions(-DSGL_QT)
ENDIF()


IF(OpenCV_FOUND)
    add_definitions(-DSGL_OPENCV)
    set(headers_cv  sgldisplay_cv.h)
    SET(REQUIRED_LIBS ${REQUIRED_LIBS} opencv_core opencv_highgui)
ENDIF()

SET(REQUIRED_LIBS ${REQUIRED_LIBS} -pthread)




add_library(${LIBNAME}  ${sources} ${headers} ${sources_QT} ${headers_QT} ${headers_cv})

IF(Qt5Widgets_FOUND)
qt5_use_modules(sgl  Widgets  )
ENDIF()

set_target_properties(${LIBNAME} PROPERTIES          # create *nix style library versions + symbolic links
    DEFINE_SYMBOL ${LIBNAME}_DSO_EXPORTS
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_SOVERSION}
    CLEAN_DIRECT_OUTPUT 1                       # allow creating static and shared libs without conflicts
    OUTPUT_NAME "${LIBNAME}${PROJECT_DLLVERSION}"    # avoid conflicts between library and binary target names
)

include(GenerateExportHeader)
generate_export_header(${LIBNAME})
target_link_libraries(${LIBNAME} ${REQUIRED_LIBS})
#target_link_libraries(${LIBNAME} PRIVATE ${QT_LIBRARIES} opencv_imgproc opencv_calib3d opencv_features2d opencv_ml)

target_include_directories(${LIBNAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${LIBNAME} EXPORT ${LIBNAME}Config
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    ARCHIVE DESTINATION lib
)

install(FILES ${headers}
    DESTINATION include/${LIBNAME}
)

install(EXPORT ${LIBNAME}Config DESTINATION "share/${LIBNAME}/cmake")
