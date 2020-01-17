###################
# UseOpenCV.cmake #
###################

option(WITH_OPENCV "Enable OpenCV support? (required for some visualization recording functions)" OFF)

if(WITH_OPENCV)
    find_package(OpenCV REQUIRED core highgui imgproc imgcodecs)
    add_definitions(-DWITH_OPENCV)
endif()
