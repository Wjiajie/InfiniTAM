#####################################
# CMakeLists.txt for Tests
#####################################

option(BUILD_TESTS "Build automated tests with the project (Boost-test required)." OFF)

set(COMMON_TEST_SOURCES
    TestUtils.cpp
    TestUtils.h
    TestUtils.tpp
    )

macro(itm_add_test)
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

if (BUILD_TESTS)

    ###########################
    # Find additional packages #
    ###########################

    find_package(Boost COMPONENTS unit_test_framework REQUIRED)

    itm_add_test(NAME VoxelVolume_SetCopyCompare_CPU SOURCES TestVoxelVolume_SetCopyCompare_CPU.cpp)
    itm_add_test(NAME VoxelVolume_ConstructFromImage_CPU SOURCES TestVoxelVolume_ConstructFromImage_CPU.cpp)
    itm_add_test(NAME VoxelVolume_SaveLoadCompact_CPU SOURCES TestVoxelVolume_SaveLoadCompact_CPU.cpp)
    itm_add_test(NAME IntArrayMap3D SOURCES TestIntArrayMap3D.cpp)
    itm_add_test(NAME ImageMaskReader SOURCES TestImageMaskReader.cpp)

    ###########################################
    # Transfer test data to working directory #
    ###########################################

    file(GLOB TEST_DATA_FILES
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.png"
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.pnm"
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.txt")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/TestData")
    foreach (TEST_DATA_FILE ${TEST_DATA_FILES})
        get_filename_component(FILENAME ${TEST_DATA_FILE} NAME)
        configure_file(
                ${TEST_DATA_FILE}
                "${CMAKE_CURRENT_BINARY_DIR}/TestData/${FILENAME}"
                COPYONLY
        )
    endforeach (TEST_DATA_FILE)
endif ()