###################
# UseVTK.cmake #
###################

# TODO allow proper support for build w/o VTK, then set the default to off
option(WITH_VTK "Enable VTK support ?(required for real-time visualization and some recording functions)" OFF)

if(WITH_VTK)
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
    add_definitions(-DWITH_VTK)
    #workaround for https://codereview.qt-project.org/#/c/195531/
    if(TARGET Qt5::Core)
        get_property(core_options TARGET Qt5::Core PROPERTY INTERFACE_COMPILE_OPTIONS)
        string(REPLACE "-fPIC" "" new_core_options ${core_options})
        set_property(TARGET Qt5::Core PROPERTY INTERFACE_COMPILE_OPTIONS ${new_core_options})
        set_property(TARGET Qt5::Core PROPERTY INTERFACE_POSITION_INDEPENDENT_CODE "ON")
    endif()
endif()
