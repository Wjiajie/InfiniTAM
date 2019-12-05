// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#include "ITMSurfelVisualizationEngineFactory.h"

#include "CPU/ITMSurfelVisualizationEngine_CPU.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMSurfelVisualizationEngine_CUDA.h"
#endif

namespace ITMLib
{

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
ITMSurfelVisualisationEngine<TSurfel> *
ITMSurfelVisualizationEngineFactory<TSurfel>::make_surfel_visualisation_engine(MemoryDeviceType deviceType)
{
  ITMSurfelVisualisationEngine<TSurfel> *visualisationEngine = NULL;

  if(deviceType == MEMORYDEVICE_CUDA)
  {
#ifndef COMPILE_WITHOUT_CUDA
    visualisationEngine = new ITMSurfelVisualizationEngine_CUDA<TSurfel>;
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    visualisationEngine = new ITMSurfelVisualizationEngine_CPU<TSurfel>;
  }

  return visualisationEngine;
}

}
