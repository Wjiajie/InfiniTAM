// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/Configuration.h"
#include "CPU/ITMSceneReconstructionEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMSceneReconstructionEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Metal/ITMSceneReconstructionEngine_Metal.h"
#endif

namespace ITMLib
{

/**
 * \brief This struct provides functions that can be used to construct scene reconstruction engines.
 */
struct ITMSceneReconstructionEngineFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a scene reconstruction engine.
   *
   * \param deviceType  The device on which the scene reconstruction engine should operate.
   */
  template <typename TVoxel, typename TIndex>
  static ITMSceneReconstructionEngine<TVoxel,TIndex> *MakeSceneReconstructionEngine(MemoryDeviceType deviceType)
  {
    ITMSceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine = NULL;

    switch(deviceType)
    {
      case MEMORYDEVICE_CPU:
        sceneRecoEngine = new ITMSceneReconstructionEngine_CPU<TVoxel,TIndex>;
        break;
      case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
        sceneRecoEngine = new ITMSceneReconstructionEngine_CUDA<TVoxel,TIndex>;
#endif
        break;
      case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
        sceneRecoEngine = new ITMSceneReconstructionEngine_Metal<TVoxelCanonical,TIndex>;
#endif
        break;
    }

    return sceneRecoEngine;
  }
};

}
