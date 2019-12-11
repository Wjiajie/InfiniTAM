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

#include "../ITMIndexingEngine_VoxelBlockHash.h"


namespace ITMLib {

template<typename TVoxel>
class ITMIndexingEngine<TVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU> :
		public ITMIndexingEngine_VoxelBlockHash<TVoxel, MEMORYDEVICE_CPU,
				ITMIndexingEngine<TVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>> {
private:
	ITMIndexingEngine() = default;
public:
	static ITMIndexingEngine& Instance() {
		static ITMIndexingEngine instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	ITMIndexingEngine(ITMIndexingEngine const&) = delete;
	void operator=(ITMIndexingEngine const&) = delete;

	void AllocateFromDepth(
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view,
			const ITMTrackingState* trackingState, const ITMRenderState* renderState,
			bool onlyUpdateVisibleList, bool resetVisibleList) override;


	void AllocateHashEntriesUsingLists(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene,
	                                   const HashEntryState* hashEntryStates_device,
	                                   Vector3s* blockCoordinates_device) override;


	void AllocateHashEntriesUsingLists_SetVisibility(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene,
	                                                 const HashEntryState* hashEntryStates_device,
	                                                 Vector3s* blockCoordinates_device,
	                                                 uchar* hashBlockVisibilityTypes_device) override;

	ITMHashEntry FindHashEntry(const ITMVoxelBlockHash& index, const Vector3s& coordinates) override;
	ITMHashEntry FindHashEntry(const ITMVoxelBlockHash& index, const Vector3s& coordinates, int& hashCode);


	bool AllocateHashBlockAt(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* volume, Vector3s at, int& hashCode) override;

	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolume(ITMVoxelVolume<TVoxelTarget, ITMVoxelBlockHash>* targetVolume,
	                              ITMVoxelVolume<TVoxelSource, ITMVoxelBlockHash>* sourceVolume);
	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolumeExpanded(ITMVoxelVolume<TVoxelTarget, ITMVoxelBlockHash>* targetVolume,
	                                      ITMVoxelVolume<TVoxelSource, ITMVoxelBlockHash>* sourceVolume);

};
} //namespace ITMLib
