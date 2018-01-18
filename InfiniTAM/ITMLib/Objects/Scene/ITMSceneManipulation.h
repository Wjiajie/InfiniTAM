//  ================================================================
//  Created by Gregory Kramida on 11/5/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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

#include "ITMScene.h"
#include "../../ITMLibDefines.h"
namespace ITMLib {
	bool AllocateHashEntry_CPU(const Vector3s& hashEntryPosition,
	                           ITMHashEntry* hashTable,
	                           ITMHashEntry*& resultEntry,
	                           int& lastFreeVoxelBlockId,
	                           int& lastFreeExcessListId,
	                           const int* voxelAllocationList,
	                           const int* excessAllocationList);

	template<class TVoxel, class TIndex>
	void CopySceneWithOffset_CPU(ITMScene <TVoxel, TIndex>& destination,
	                             ITMScene <TVoxel, TIndex>& source,
	                             Vector3i offset);

	//TODO -make this suitable for source/dest scenes with different voxel types somehow -Greg (Github: Algomorph)
	void CopySceneWithOffset_CPU(ITMScene <ITMVoxelLive, ITMVoxelIndex>& destination,
	                             ITMScene <ITMVoxelCanonical, ITMVoxelIndex>& source,
	                             Vector3i offset);

	template<class TVoxel, class TIndex>
	TVoxel ReadVoxel(ITMScene <TVoxel, TIndex>& scene, Vector3i at);



	template<class TVoxel, class TIndex>
	bool SetVoxel_CPU(ITMScene <TVoxel, TIndex>& scene, Vector3i at, TVoxel voxel);


	template<class TVoxel, class TIndex>
	void CopySceneWithOffset_CPU(ITMScene <TVoxel, TIndex>& destination, ITMScene <TVoxel, TIndex>& source, Vector3i offset);

	void
	CopySceneWithOffset_CPU(ITMScene <ITMVoxelLive, ITMVoxelIndex>& destination, ITMScene <ITMVoxel, ITMVoxelIndex>& source,
	                        Vector3i offset);

	template<class TVoxel, class TIndex>
	TVoxel ReadVoxel(ITMScene <TVoxel, TIndex>& scene, Vector3i at);


	template<class TVoxel, class TIndex>
	bool SetVoxel_CPU(ITMScene <TVoxel, TIndex>& scene, Vector3i at, TVoxel voxel);
}//namespace ITMLib