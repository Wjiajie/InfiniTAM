####################
# LinkFFmpeg.cmake #
####################

if(WITH_FFMPEG)
  target_link_libraries(${targetname} ${FFMPEG_LIBRARIES})

  #if(MSVC_IDE)
  #  add_custom_command(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${FFmpeg_SHARED_ROOT}/bin/avcodec-57.dll" "$<TARGET_FILE_DIR:${targetname}>")
  #  add_custom_command(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${FFmpeg_SHARED_ROOT}/bin/avdevice-57.dll" "$<TARGET_FILE_DIR:${targetname}>")
  #  add_custom_command(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${FFmpeg_SHARED_ROOT}/bin/avfilter-6.dll" "$<TARGET_FILE_DIR:${targetname}>")
  #  add_custom_command(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${FFmpeg_SHARED_ROOT}/bin/avformat-57.dll" "$<TARGET_FILE_DIR:${targetname}>")
  #  add_custom_command(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${FFmpeg_SHARED_ROOT}/bin/avutil-55.dll" "$<TARGET_FILE_DIR:${targetname}>")
  #endif()
endif()
