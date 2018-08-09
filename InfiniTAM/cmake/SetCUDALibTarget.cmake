##########################
# SetCUDALibTarget.cmake #
##########################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/Flags.cmake)

#IF(WITH_CUDA)
#  CUDA_ADD_LIBRARY(${targetname} STATIC ${sources} ${headers} ${templates})
#ELSE()
#  ADD_LIBRARY(${targetname} STATIC ${sources} ${headers} ${templates})
#ENDIF()
if(WITH_CUDA)
	enable_language(CUDA)
endif()

add_library(${targetname} STATIC ${sources} ${headers} ${templates})
