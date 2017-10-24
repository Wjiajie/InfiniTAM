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



template<class TVoxel, class TIndex>
float
ITMSceneMotionTracker_CPU<TVoxel, TIndex>::UpdateWarpField(ITMScene<TVoxel, TIndex>* canonicalScene,
                                                                       ITMScene<TVoxel, TIndex>* liveScene) {

	const TVoxel* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	const TVoxel* liveVoxels = liveScene->localVBA.GetVoxelBlocks();

	const ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();

	int noTotalEntries = canonicalScene->index.noTotalEntries;

	typename TIndex::IndexCache cacheCanonical;
	typename TIndex::IndexCache cacheLive;

	float sdf_z_buffer[SDF_BLOCK_SIZE][SDF_BLOCK_SIZE];
	Vector3f color_z_buffer[SDF_BLOCK_SIZE][SDF_BLOCK_SIZE];
	bool z_buffer[SDF_BLOCK_SIZE][SDF_BLOCK_SIZE];



	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		Vector3i canonicalHashEntryPosition;
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[entryId];

		if (currentCanonicalHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		canonicalHashEntryPosition = currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i voxelPosition = canonicalHashEntryPosition + Vector3i(x, y, z);
					int foundVoxel;
					TVoxel canonicalVoxel = readVoxel(canonicalVoxels, canonicalHashTable, voxelPosition, foundVoxel, cacheCanonical);
					if(foundVoxel){
						Vector3f projectedPosition = voxelPosition.toFloat() + canonicalVoxel.warp_t;
						float sdf;
						Vector3f color;
						interpolateTrilinearly(liveVoxels,liveHashTable,projectedPosition,cacheLive,sdf,color);

					}
				}
			}
		}
	}

	DIEWITHEXCEPTION("Scene tracking iteration not yet implemented");
	return 0;
}

template<class TVoxel, class TIndex>
void ITMSceneMotionTracker_CPU<TVoxel, TIndex>::DeformScene(ITMScene<TVoxel, TIndex>* sceneOld,
                                                                        ITMScene<TVoxel, TIndex>* sceneNew) {

	DIEWITHEXCEPTION("Scene tracking iteration not yet implemented");
}
