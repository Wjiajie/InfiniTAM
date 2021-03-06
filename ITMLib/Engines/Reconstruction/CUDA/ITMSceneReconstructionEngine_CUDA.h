// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMSceneReconstructionEngine.h"
#include "../../../Objects/Scene/PlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMSceneReconstructionEngine_CUDA : public ITMSceneReconstructionEngine < TVoxel, TIndex >
	{};

	template<class TVoxel>
	class ITMSceneReconstructionEngine_CUDA<TVoxel, VoxelBlockHash> : public ITMSceneReconstructionEngine < TVoxel, VoxelBlockHash >
	{
	private:
		void *allocationTempData_device;
		void *allocationTempData_host;

	public:
		void ResetScene(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene);

		void AllocateSceneFromDepth(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                            const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                        const ITMRenderState *renderState);

		ITMSceneReconstructionEngine_CUDA(void);
		~ITMSceneReconstructionEngine_CUDA(void);
	};

	template<class TVoxel>
	class ITMSceneReconstructionEngine_CUDA<TVoxel, PlainVoxelArray> : public ITMSceneReconstructionEngine < TVoxel, PlainVoxelArray >
	{
	public:
		void ResetScene(ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene);

		void AllocateSceneFromDepth(ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                            const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                        const ITMRenderState *renderState);
	};
}
