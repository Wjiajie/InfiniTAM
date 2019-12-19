//  ================================================================
//  Created by Gregory Kramida on 11/1/19.
//  Copyright (c) 2019 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================

#include "ITMIndexingEngine_CPU_VoxelBlockHash.tpp"
#include "../ITMIndexingEngine_VoxelBlockHash.tpp"
#include "../../../../ITMLibDefines.h"

namespace ITMLib {
template
class ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>;
template
class ITMIndexingEngine<ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>;

template void ITMIndexingEngine<ITMVoxel,ITMVoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMLib::ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* targetVolume,
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* sourceVolume);
template void ITMIndexingEngine<ITMVoxel,ITMVoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* targetVolume,
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* sourceVolume);
template void ITMIndexingEngine<ITMVoxel,ITMVoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeExpanded(
		ITMLib::ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* targetVolume,
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* sourceVolume);
template void ITMIndexingEngine<ITMVoxel,ITMVoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeExpanded(
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* targetVolume,
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* sourceVolume);
template void ITMIndexingEngine<ITMVoxel,ITMVoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		ITMLib::ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* targetVolume,
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix);
template void ITMIndexingEngine<ITMVoxel,ITMVoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* targetVolume,
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix);

template void ITMIndexingEngine_VoxelBlockHash<ITMVoxel, MEMORYDEVICE_CPU,
		ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>>::
AllocateFromWarpedVolume<WarpType::WARP_CUMULATIVE>(
		ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warpField,
		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* targetTSDF
);
template void ITMIndexingEngine_VoxelBlockHash<ITMVoxel, MEMORYDEVICE_CPU,
		ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>>::
AllocateFromWarpedVolume<WarpType::WARP_FLOW>(
		ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warpField,
		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* targetTSDF
);
template void ITMIndexingEngine_VoxelBlockHash<ITMVoxel, MEMORYDEVICE_CPU,
		ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>>::
AllocateFromWarpedVolume<WarpType::WARP_UPDATE>(
		ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warpField,
		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* targetTSDF
);

}//namespace ITMLib