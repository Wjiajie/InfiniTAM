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
#include "ITMSceneMotionTracker_CPU.h"
#include "../Shared/ITMSceneMotionTracker_Shared.h"

using namespace ITMLib;



template<class TVoxel, class TWarpField, class TIndex>
float
ITMSceneMotionTracker_CPU<TVoxel, TWarpField, TIndex>::UpdateWarpField(ITMScene<TVoxel, TIndex>* canonicalScene,
                                                                       ITMScene<TVoxel, TIndex>* liveScene,
                                                                       ITMScene<TWarpField, TIndex>* warpField) {

	//TODO

	DIEWITHEXCEPTION("Scene tracking iteration not yet implemented");
	return 0;
}

template<class TVoxel, class TWarpField, class TIndex>
void ITMSceneMotionTracker_CPU<TVoxel, TWarpField, TIndex>::DeformScene(ITMScene<TVoxel, TIndex>* sceneOld,
                                                                        ITMScene<TVoxel, TIndex>* sceneNew,
                                                                        ITMScene<TWarpField, TIndex>* warpField) {
	const TVoxel* oldLocalVBA = sceneOld->localVBA.GetVoxelBlocks();
	const TWarpField* warpLocalVBA = warpField->localVBA.GetVoxelBlocks();
	const TVoxel* newLocalVBA = sceneNew->localVBA.GetVoxelBlocks();

	//should match //TODO: maybe combine warp field and the live scene into a single voxel grid to accelerate lookups(?)
	const ITMHashEntry* oldHashTable = sceneOld->index.GetEntries();
	const ITMHashEntry* warpHashTable = warpField->index.GetEntries();
	int noTotalLiveEntries = sceneOld->index.noTotalEntries;


	for (int entryId = 0; entryId < noTotalLiveEntries; entryId++) {
		Vector3i liveHashEntryPosition;
		const ITMHashEntry& currentLiveHashEntry = oldHashTable[entryId];
		const ITMHashEntry& currentWarpHashEntry = warpHashTable[entryId];

		if (currentLiveHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		liveHashEntryPosition = currentLiveHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					const int neighborCount = 8;
					Vector3f points[neighborCount];
					float sdfVals[neighborCount];
					Vector3u colorVals[neighborCount];

					Vector3i livePointPosition = liveHashEntryPosition + Vector3i(x, y, z);
					int vmIndex;
					Vector3f warp_t = readVoxel(warpLocalVBA, warpHashTable, livePointPosition, vmIndex).warp_t;
					//use truncated value here, for it will yield the correct neighbors during lookup
					findPointNeighborsGeneric(points, sdfVals, colorVals, livePointPosition,
					                          oldLocalVBA, oldHashTable);
					float sdfCanonical;
					Vector3u colorCanonical;
//					interpolateTrilinear(sdfCanonical, colorCanonical, sdfVals, colorVals, points,
//					                     livePointPosition);

				}
			}
		}
	}
	DIEWITHEXCEPTION("Scene tracking iteration not yet implemented");
}





