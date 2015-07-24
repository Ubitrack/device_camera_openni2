set(the_description "The UbiTrack OpenNI2FrameGrabber Component")
ut_add_component(OpenNI2FrameGrabber DEPS utcore utdataflow utvision )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# should be extended to recognize V1/V2 and include files accordingly ...
FIND_PACKAGE(Flycapture)
IF(FLYCAPTURE_FOUND)
	ut_component_include_directories("src/OpenNI2FrameGrabber" ${UBITRACK_CORE_DEPS_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR} ${TBB_INCLUDE_DIR} ${FLYCAPTURE_INCLUDE_DIRS})
	ut_glob_component_sources(SOURCES "src/OpenNI2FrameGrabber/OpenNI2FrameGrabber.cpp")
	ut_create_single_component(${FLYCAPTURE_LIBRARIES} ${TBB_ALL_LIBRARIES})
	ut_install_utql_patterns()
ENDIF(FLYCAPTURE_FOUND)
