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
    static ITMRenderState *CreateRenderState(const Vector2i& imgSize, const ITMSceneParams *sceneParams, MemoryDeviceType memoryType, int hashEntryCount = 0)
    {
      return new ITMRenderState(imgSize, sceneParams->viewFrustum_min, sceneParams->viewFrustum_max, memoryType);
    }
  };

  template <>
  struct ITMRenderStateFactory<ITMVoxelBlockHash>
  {
    /** Creates a render state, containing rendering info for the scene. */
    static ITMRenderState *CreateRenderState(const Vector2i& imgSize, const ITMSceneParams *sceneParams, MemoryDeviceType memoryType, int hashEntryCount = ORDERED_LIST_SIZE + DEFAULT_EXCESS_LIST_SIZE)
    {
      return new ITMRenderState_VH(hashEntryCount, imgSize, sceneParams->viewFrustum_min, sceneParams->viewFrustum_max, memoryType);
    }
  };
}
