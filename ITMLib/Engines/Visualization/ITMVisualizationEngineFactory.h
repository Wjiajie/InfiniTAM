// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CPU/ITMVisualizationEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMVisualizationEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Metal/ITMVisualisationEngine_Metal.h"
#endif

namespace ITMLib
{

/**
 * \brief This struct provides functions that can be used to construct visualisation engines.
 */
struct ITMVisualizationEngineFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a visualisation engine.
   *
   * \param deviceType  The device on which the visualisation engine should operate.
   */
  template <typename TVoxel, typename TIndex>
  static ITMVisualisationEngine<TVoxel,TIndex> *MakeVisualisationEngine(MemoryDeviceType deviceType)
  {
    ITMVisualisationEngine<TVoxel,TIndex> *visualisationEngine = NULL;

    switch(deviceType)
    {
      case MEMORYDEVICE_CPU:
        visualisationEngine = new ITMVisualizationEngine_CPU<TVoxel,TIndex>;
        break;
      case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
        visualisationEngine = new ITMVisualizationEngine_CUDA<TVoxel,TIndex>;
#endif
        break;
      case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
        visualisationEngine = new ITMVisualisationEngine_Metal<TVoxelCanonical,TIndex>;
#endif
        break;
    }

    return visualisationEngine;
  }
};

}
