#####################################
# CMakeLists.txt for Tests
#####################################

option(BUILD_TESTS "Build automated tests with the project (Boost-test required)." OFF)

if(BUILD_TESTS)
    ###########################
    # Find additional packages #
    ###########################

    find_package(Boost COMPONENTS unit_test_framework REQUIRED)

    ###########################
    # Specify the target name #
    ###########################
    SET(targetname tests)

    #############################
    # Specify the project files #
    #############################
    SET(sources
            AllTests.cpp
            )

    ################################
    # Specify the libraries to use #
    ################################

    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseFFmpeg.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseGLUT.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseLibRoyale.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenGL.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UsePNG.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseUVC.cmake)

    ##########################################
    # Specify the target and where to put it #
    ##########################################
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)

    target_link_libraries(${targetname}
            InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib
            ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
            )
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkFFmpeg.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkGLUT.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkLibRoyale.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkPNG.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake)
endif()