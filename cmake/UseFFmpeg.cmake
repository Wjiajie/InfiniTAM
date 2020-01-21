###################
# UseFFmpeg.cmake #
###################

option(WITH_FFMPEG "Build with FFmpeg support?" OFF)

if(WITH_FFMPEG)
  find_package(FFMPEG COMPONENTS avcodec avformat avfilter avutil QUIET)
  if(${FFMPEG_FOUND})
    include_directories(${FFMPEG_INCLUDE_DIRS})
      #TODO: replace add_definitions with an ITMLibConfig.h.in --> ITMLibConfig.h configurable header, containing the cmake-dependent preprocessor defines -Greg (GitHub: Algomorph)
    add_definitions(-DCOMPILE_WITH_FFMPEG)
  else()
    MESSAGE(FATAL_ERROR "FFMPEG not found!")
  endif()
endif()
