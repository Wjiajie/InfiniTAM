// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMSceneReconstructionEngine.h"
#include "../../../Objects/Scene/PlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMSceneReconstructionEngine_CPU : public ITMSceneReconstructionEngine < TVoxel, TIndex >
	{};

	template<class TVoxel>
	class ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMSceneReconstructionEngine < TVoxel, ITMVoxelBlockHash >
	{

	public:
		void ResetScene(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash> *scene);

		void AllocateSceneFromDepth(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                            const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false) override;

		void IntegrateIntoScene(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                        const ITMRenderState *renderState);

		ITMSceneReconstructionEngine_CPU() = default;
		~ITMSceneReconstructionEngine_CPU() = default;
	};

	template<class TVoxel>
	class ITMSceneReconstructionEngine_CPU<TVoxel, PlainVoxelArray> : public ITMSceneReconstructionEngine < TVoxel, PlainVoxelArray >
	{
	public:
		void ResetScene(ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene);

		void AllocateSceneFromDepth(ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                            const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                        const ITMRenderState *renderState);

		ITMSceneReconstructionEngine_CPU(void);
		~ITMSceneReconstructionEngine_CPU(void);
	};
}
