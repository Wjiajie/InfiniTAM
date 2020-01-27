##################
# LinkGLUT.cmake #
##################

target_link_libraries(${targetname} ${GLUT_LIBRARIES})
target_include_directories(${targetname} PUBLIC ${GLUT_INCLUDE_DIR})
#if(MSVC_IDE)
#   message(STATUS "GLUT ROOT: $ENV{GLUT_ROOT}")
#  add_custom_command(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${GLUT_ROOT}/bin/x64/freeglut.dll" "$<TARGET_FILE_DIR:${targetname}>")
#endif()
