//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
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
#include <cmath>
#include "ITMSceneMotionTracker_CPU.h"
#include "../Shared/ITMSceneMotionTracker_Shared.h"

using namespace ITMLib;



template<class TVoxel, class TWarpField, class TIndex>
float
ITMSceneMotionTracker_CPU<TVoxel, TWarpField, TIndex>::UpdateWarpField(ITMScene<TVoxel, TIndex>* canonicalScene,
                                                                       ITMScene<TVoxel, TIndex>* liveScene,
                                                                       ITMScene<TWarpField, TIndex>* warpField,
                                                                       ITMScene <TWarpField, TIndex>* warpFieldDelta) {

	//TODO

	DIEWITHEXCEPTION("Scene tracking iteration not yet implemented");
	return 0;
}

template<class TVoxel, class TWarpField, class TIndex>
void ITMSceneMotionTracker_CPU<TVoxel, TWarpField, TIndex>::DeformScene(ITMScene<TVoxel, TIndex>* sceneOld,
                                                                        ITMScene<TVoxel, TIndex>* sceneNew,
                                                                        ITMScene<TWarpField, TIndex>* warpField) {
	const TVoxel* oldVoxelBlocks = sceneOld->localVBA.GetVoxelBlocks();
	const TWarpField* warpVoxelBlocks = warpField->localVBA.GetVoxelBlocks();

	TVoxel* newVoxelBlocks = sceneNew->localVBA.GetVoxelBlocks();

	//should match //TODO: maybe combine warp field and the live scene into a single voxel grid to accelerate lookups(?)
	const ITMHashEntry* oldHashTable = sceneOld->index.GetEntries();
	const ITMHashEntry* warpHashTable = warpField->index.GetEntries();

	ITMHashEntry* newHashTable = sceneNew->index.GetEntries();
	int noTotalLiveEntries = sceneOld->index.noTotalEntries;


	for (int entryId = 0; entryId < noTotalLiveEntries; entryId++) {
		Vector3i oldHashEntryPosition;
		const ITMHashEntry& currentLiveHashEntry = oldHashTable[entryId];

		if (currentLiveHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		oldHashEntryPosition = currentLiveHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i oldVoxelPosition = oldHashEntryPosition + Vector3i(x, y, z);
					bool foundVoxel;
					TVoxel oldVoxel = readVoxel(oldVoxelBlocks, oldHashTable, oldVoxelPosition, foundVoxel);
					if(!foundVoxel){
						continue;
					}
					TWarpField warpVoxel = readVoxel(warpVoxelBlocks, warpHashTable, oldVoxelPosition, foundVoxel);
					Vector3f projectedPosition = oldVoxelPosition.toFloat() + warpVoxel.warp_t;

					distributeTrilinearly(oldVoxel, projectedPosition, newVoxelBlocks, newHashTable, sceneNew);
				}
			}
		}
	}
	DIEWITHEXCEPTION("Scene tracking iteration not yet implemented");
}





