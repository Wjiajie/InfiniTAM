// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMSwappingEngine.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMSwappingEngine_CUDA : public ITMSwappingEngine < TVoxel, TIndex >
	{
	public:
		explicit ITMSwappingEngine_CUDA(const TIndex& index){};
		void IntegrateGlobalIntoLocal(ITMVoxelVolume<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
		void SaveToGlobalMemory(ITMVoxelVolume<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
		void CleanLocalMemory(ITMVoxelVolume<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
	};

	template<class TVoxel>
	class ITMSwappingEngine_CUDA<TVoxel, VoxelBlockHash> : public ITMSwappingEngine < TVoxel, VoxelBlockHash >
	{
	private:
		int *noNeededEntries_device, *noAllocatedVoxelEntries_device;
		int *entriesToClean_device;

		int LoadFromGlobalMemory(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene);

	public:
		void IntegrateGlobalIntoLocal(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene, ITMRenderState *renderState);
		void SaveToGlobalMemory(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene, ITMRenderState *renderState);
		void CleanLocalMemory(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene, ITMRenderState *renderState);

		explicit ITMSwappingEngine_CUDA(const VoxelBlockHash& index);
		~ITMSwappingEngine_CUDA();
	};
}
