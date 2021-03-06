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
#pragma once

#include "../../Interface/ITMIndexingEngine.h"
#include "../ITMIndexingEngine_VoxelBlockHash.h"
#include "../../../Common/ITMWarpEnums.h"

namespace ITMLib {

template<typename TVoxel>
class ITMIndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> :
		public ITMIndexingEngine_VoxelBlockHash<TVoxel, MEMORYDEVICE_CUDA,
		ITMIndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> > {
private:
	ITMIndexingEngine() = default;

	void SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(HashBlockVisibility* hashBlockVisibilityTypes,
	                                                        const int* visibleBlockHashCodes,
	                                                        int visibleHashBlockCount);
public:
	static ITMIndexingEngine& Instance() {
		static ITMIndexingEngine instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	ITMIndexingEngine(ITMIndexingEngine const&) = delete;
	void operator=(ITMIndexingEngine const&) = delete;

	void AllocateFromDepth(ITMVoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view,
			const ITMTrackingState* trackingState, bool onlyUpdateVisibleList, bool resetVisibleList) override;

	void AllocateFromDepth(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                       const Matrix4f& depth_camera_matrix = Matrix4f::Identity(),
	                       bool onlyUpdateVisibleList = false, bool resetVisibleList = false) override;

	void AllocateHashEntriesUsingLists(ITMVoxelVolume <TVoxel, VoxelBlockHash>* volume) override;

	void AllocateHashEntriesUsingLists_SetVisibility(ITMVoxelVolume <TVoxel, VoxelBlockHash>* volume) override;

	void BuildVisibilityList(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                         const Matrix4f& depth_camera_matrix = Matrix4f::Identity());

	ITMHashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates) override;
	ITMHashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates, int& hashCode);
	bool AllocateHashBlockAt(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hashCode) override;

	template<typename TVoxelATarget, typename TVoxelASource>
	void AllocateUsingOtherVolume(ITMVoxelVolume <TVoxelATarget, VoxelBlockHash>* targetVolume,
	                              ITMVoxelVolume <TVoxelASource, VoxelBlockHash>* sourceVolume);

	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolumeExpanded(ITMVoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
	                                      ITMVoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume);

	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolumeAndSetVisibilityExpanded(ITMVoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
	                                                      ITMVoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume,
	                                                      ITMView* view, const Matrix4f& depth_camera_matrix = Matrix4f::Identity());
};
} //namespace ITMLib

