//  ================================================================
//  Created by Gregory Kramida on 11/15/19.
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

#include "../../Interface/ITMIndexingEngine.tpp"
#include "../../../../ITMLibDefines.h"

namespace ITMLib {
template
class ITMIndexingEngine<ITMVoxel, ITMPlainVoxelArray, MEMORYDEVICE_CPU>;
template
class ITMIndexingEngine<ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>;


template void ITMIndexingEngine<ITMVoxel,ITMPlainVoxelArray,MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMLib::ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* targetVolume,
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* sourceVolume);
template void ITMIndexingEngine<ITMVoxel,ITMPlainVoxelArray,MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* targetVolume,
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* sourceVolume);
template void ITMIndexingEngine<ITMVoxel,ITMPlainVoxelArray,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeExpanded(
		ITMLib::ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* targetVolume,
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* sourceVolume);
template void ITMIndexingEngine<ITMVoxel,ITMPlainVoxelArray,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeExpanded(
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* targetVolume,
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* sourceVolume);
template void ITMIndexingEngine<ITMVoxel,ITMPlainVoxelArray,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		ITMLib::ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* targetVolume,
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix);
template void ITMIndexingEngine<ITMVoxel,ITMPlainVoxelArray,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* targetVolume,
		ITMLib::ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix);

template void ITMIndexingEngine<ITMVoxel, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
AllocateFromWarpedVolume<WarpType::WARP_CUMULATIVE>(
		ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warpField,
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* sourceTSDF,
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* targetTSDF
);
template void ITMIndexingEngine<ITMVoxel, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
AllocateFromWarpedVolume<WarpType::WARP_FLOW>(
		ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warpField,
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* sourceTSDF,
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* targetTSDF
);
template void ITMIndexingEngine<ITMVoxel, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
AllocateFromWarpedVolume<WarpType::WARP_UPDATE>(
		ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warpField,
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* sourceTSDF,
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* targetTSDF
);
} //namespace ITMLib