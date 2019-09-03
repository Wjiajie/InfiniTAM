#####################################
# CMakeLists.txt for Tests
#####################################

option(BUILD_TESTS "Build automated tests with the project (Boost-test required)." OFF)

macro (ITM_ADD_TEST)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs SOURCES)
    cmake_parse_arguments(ITM_ADD_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    set(targetname test_${ITM_ADD_TEST_NAME})
    set(sources ${ITM_ADD_TEST_SOURCES} ${COMMON_TEST_SOURCES})

    include(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseFFmpeg.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseGLUT.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseLibRoyale.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseOpenGL.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UsePNG.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseUVC.cmake)

    ##########################################
    # Specify the target and where to put it #
    ##########################################
    include(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)

    target_link_libraries(${targetname}
                          InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib
                          ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
                          )
    include(${PROJECT_SOURCE_DIR}/cmake/LinkFFmpeg.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkGLUT.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkLibRoyale.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkPNG.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake)
endmacro()

if(BUILD_TESTS)

    ###########################
    # Find additional packages #
    ###########################

    find_package(Boost COMPONENTS unit_test_framework REQUIRED)

    itm_add_test(NAME VoxelVolumeConstruction SOURCES TestVoxelVolumeConstruction.cpp TestUtils.cpp TestUtils.h TestUtils.tpp)
    itm_add_test(NAME IntArrayMap3D SOURCES TestITMIntArrayMap3D.cpp)
endif()