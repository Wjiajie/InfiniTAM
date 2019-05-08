#####################################
# CMakeLists.txt for Apps/InfiniTAM #
#####################################

###########################
# Specify the target name #
###########################

SET(targetname InfiniTAM_bpo_cli)

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


#################################
# Extra libraries (application-specific)
#################################
find_package(VTK 8.1 COMPONENTS
			 vtkIOImage
			 vtkChartsCore
			 vtkCommonColor
			 vtkCommonCore
			 vtkCommonDataModel
			 vtkFiltersSources
			 vtkInteractionStyle
			 vtkInteractionWidgets
			 vtkRenderingContext2D
			 vtkRenderingContextOpenGL2
			 vtkRenderingCore
			 vtkRenderingFreeType
			 vtkRenderingOpenGL2
			 vtkRenderingGL2PSOpenGL2
			 vtkViewsContext2D
			 REQUIRED)
include(${VTK_USE_FILE})

SET (Boost_FIND_REQUIRED TRUE)
SET (Boost_FIND_QUIETLY TRUE)
set (Boost_USE_MULTITHREADED TRUE)
if(CMAKE_CXX_COMPILER_ID MATCHES MSVC)
	set (Boost_USE_STATIC_LIBS TRUE)
	set(Boost_USE_STATIC_RUNTIME OFF)
endif()
find_package(Boost REQUIRED COMPONENTS program_options)

#############################
# Specify the project files #
#############################

SET(sources
    InfiniTAM_bpo_cli.cpp
    CLIEngine_BPO.cpp
    CreateDefaultImageSource.cpp
)

SET(headers
    CLIEngine_BPO.h
    prettyprint.hpp
    CreateDefaultImageSource.h
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${sources} ${headers})

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)

#################################
# Specify the libraries to link #
#################################

TARGET_LINK_LIBRARIES(${targetname} InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkFFmpeg.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkGLUT.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkLibRoyale.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkPNG.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake)


target_link_libraries(${targetname} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${VTK_LIBRARIES})
if(CMAKE_CXX_COMPILER_ID MATCHES MSVC)
	target_compile_definitions(${targetname} PUBLIC -DBOOST_ALL_NO_LIB  -DBOOST_SYSTEM_NO_DEPRECATED)
endif()