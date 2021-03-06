// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMDenseMapper.h"

#include "../Reconstruction/ITMSceneReconstructionEngineFactory.h"
#include "../Swapping/ITMSwappingEngineFactory.h"

using namespace ITMLib;

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel, TIndex>::ITMDenseMapper(const TIndex& index)
{
	auto& settings = configuration::get();
	sceneRecoEngine = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxel,TIndex>(settings.device_type);
	swappingEngine = settings.swapping_mode != configuration::SWAPPINGMODE_DISABLED ? ITMSwappingEngineFactory::MakeSwappingEngine<TVoxel,TIndex>(settings.device_type, index) : nullptr;

	swappingMode = settings.swapping_mode;
}

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel,TIndex>::~ITMDenseMapper()
{
	delete sceneRecoEngine;
	delete swappingEngine;
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ResetScene(ITMVoxelVolume<TVoxel,TIndex> *scene) const
{
	sceneRecoEngine->ResetScene(scene);
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMVoxelVolume<TVoxel,TIndex> *scene, ITMRenderState *renderState)
{
	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState);

	// integration
	sceneRecoEngine->IntegrateIntoScene(scene, view, trackingState, renderState);

	if (swappingEngine != NULL) {
		// swapping: CPU -> CUDA
		if (swappingMode == configuration::SWAPPINGMODE_ENABLED) swappingEngine->IntegrateGlobalIntoLocal(scene, renderState);

		// swapping: CUDA -> CPU
		switch (swappingMode)
		{
		case configuration::SWAPPINGMODE_ENABLED:
			swappingEngine->SaveToGlobalMemory(scene, renderState);
			break;
		case configuration::SWAPPINGMODE_DELETE:
			swappingEngine->CleanLocalMemory(scene, renderState);
			break;
		case configuration::SWAPPINGMODE_DISABLED:
			break;
		} 
	}
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, ITMVoxelVolume<TVoxel,TIndex> *scene, ITMRenderState *renderState, bool resetVisibleList)
{
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, true, resetVisibleList);
}
