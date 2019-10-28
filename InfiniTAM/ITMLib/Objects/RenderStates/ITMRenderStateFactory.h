// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMRenderState_VH.h"
#include "../../Utils/ITMSceneParams.h"

namespace ITMLib
{
  template <typename TIndex>
  struct ITMRenderStateFactory
  {
    /** Creates a render state, containing rendering info for the scene. */
    static ITMRenderState *CreateRenderState(const Vector2i& imgSize, const ITMSceneParams *sceneParams, MemoryDeviceType memoryType, const TIndex& index)
    {
      return new ITMRenderState(imgSize, sceneParams->viewFrustum_min, sceneParams->viewFrustum_max, memoryType);
    }
  };

  template <>
  struct ITMRenderStateFactory<ITMVoxelBlockHash>
  {
    /** Creates a render state, containing rendering info for the scene. */
    static ITMRenderState *CreateRenderState(const Vector2i& imgSize, const ITMSceneParams *sceneParams, MemoryDeviceType memoryType, const ITMVoxelBlockHash& index)
    {
      return new ITMRenderState_VH(index.hashEntryCount, index.voxelBlockCount, imgSize, sceneParams->viewFrustum_min, sceneParams->viewFrustum_max, memoryType);
    }
  };
}
