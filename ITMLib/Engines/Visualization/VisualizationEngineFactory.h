// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CPU/VisualizationEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/VisualizationEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Metal/VisualizationEngine_Metal.h"
#endif

namespace ITMLib
{

/**
 * \brief This struct provides functions that can be used to construct Visualization engines.
 */
struct VisualizationEngineFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a Visualization engine.
   *
   * \param deviceType  The device on which the Visualization engine should operate.
   */
  template <typename TVoxel, typename TIndex>
  static VisualizationEngine<TVoxel,TIndex> *MakeVisualizationEngine(MemoryDeviceType deviceType)
  {
    VisualizationEngine<TVoxel,TIndex> *visualization_engine = NULL;

    switch(deviceType)
    {
      case MEMORYDEVICE_CPU:
	      visualization_engine = new VisualizationEngine_CPU<TVoxel,TIndex>;
        break;
      case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
        visualization_engine = new VisualizationEngine_CUDA<TVoxel,TIndex>;
#endif
        break;
      case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
        visualizationEngine = new VisualizationEngine_Metal<TVoxelCanonical,TIndex>;
#endif
        break;
    }

    return visualization_engine;
  }
};

}
