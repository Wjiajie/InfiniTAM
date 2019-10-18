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

    ##########################################
    # Add the target as a CMake CTest        #
    ##########################################
    add_test(NAME ${ITM_ADD_TEST_NAME} COMMAND test_${ITM_ADD_TEST_NAME})
endmacro()

if (BUILD_TESTS)
    enable_testing()

    ###########################
    # Find additional packages #
    ###########################

    find_package(Boost COMPONENTS unit_test_framework REQUIRED)

    itm_add_test(NAME SetCopyCompare_CPU SOURCES Test_SetCopyCompare_CPU.cpp)
    itm_add_test(NAME SetCopyCompare_CUDA SOURCES Test_SetCopyCompare_CUDA.cpp)
    itm_add_test(NAME ConstructVolumeFromImage_CPU SOURCES Test_ConstructVolumeFromImage_CPU.cpp)
    itm_add_test(NAME ConstructVolumeFromImage_CUDA SOURCES Test_ConstructVolumeFromImage_CUDA.cpp)
    itm_add_test(NAME VolumeSaveLoadCompact_CPU SOURCES Test_VolumeSaveLoadCompact_CPU.cpp)
    itm_add_test(NAME VolumeSaveLoadCompact_CUDA SOURCES Test_VolumeSaveLoadCompact_CUDA.cpp)
    itm_add_test(NAME IntArrayMap3D SOURCES TestIntArrayMap3D.cpp)
    itm_add_test(NAME ImageMaskReader SOURCES TestImageMaskReader.cpp)
    itm_add_test(NAME WarpGradient_CPU SOURCES Test_WarpGradient_CPU.cpp)
    itm_add_test(NAME WarpGradient_CUDA SOURCES Test_WarpGradient_CUDA.cpp)
    itm_add_test(NAME SceneSlicingPVA_CPU SOURCES Test_SceneSlicingPVA_CPU.cpp)



    #####################################################
    # Recursively transfer test data to build directory #
    #####################################################

    file(GLOB_RECURSE TEST_DATA_FILES
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.png"
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.pnm"
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.txt"
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.dat"
         )
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/TestData")
    foreach (TEST_DATA_FILE ${TEST_DATA_FILES})
        string(REPLACE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} NEW_FILE ${TEST_DATA_FILE})
        get_filename_component(TARGET_DIR ${NEW_FILE} DIRECTORY)
        file(MAKE_DIRECTORY ${TARGET_DIR})
        configure_file(
                ${TEST_DATA_FILE}
                ${NEW_FILE}
                COPYONLY
        )
    endforeach (TEST_DATA_FILE)
endif ()