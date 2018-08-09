##########################
# SetCUDAAppTarget.cmake #
##########################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/Flags.cmake)

#IF(WITH_CUDA)
#  CUDA_ADD_EXECUTABLE(${targetname} ${sources} ${headers} ${templates})
#ELSE()
#  ADD_EXECUTABLE(${targetname} ${sources} ${headers} ${templates})
#ENDIF()

if(WITH_CUDA)
	enable_language(CUDA)
endif()

add_executable(${targetname} ${sources} ${headers} ${templates})

IF(MSVC_IDE)
  SET_TARGET_PROPERTIES(${targetname} PROPERTIES LINK_FLAGS_DEBUG "/DEBUG")
ENDIF()
