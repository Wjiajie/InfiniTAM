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
#include "../../Objects/Scene/ITMRepresentationAccess.h"

using namespace ITMLib;


//TODO: assume we do have color for now. Make provisions later to account for TVoxel with no color. -Greg (GitHub:Algomorph)
/**
 * \brief find neighboring SDF values and point locations.
 * This is different from findPointNeighbors because it returns 1.0 for truncated voxels and voxels beyond the scene boundary.
 * \tparam TVoxel the voxel type
 * \param p [out] pointer to memory where to store 8 values Vector3i for all the neighbors
 * \param sdf [out] pointer to memory where to store 8 SDF values for all the neighbors
 * \param blockLocation the actual block location
 * \param localVBA
 * \param hashTable
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void findPointNeighborsGeneric(THREADPTR(Vector3f) *p,
                                                         THREADPTR(float) *sdf,
                                                         THREADPTR(Vector3u) *colorVals,
                                                         Vector3i blockLocation,
                                                         const CONSTPTR(TVoxel) *localVBA,
                                                         const CONSTPTR(ITMHashEntry) *hashTable)
{
	int vmIndex; Vector3i localBlockLocation;

	Vector3i(0, 0, 0);
	TVoxel voxel;
#define PROCESS_VOXEL(location, index)\
	localBlockLocation = blockLocation + location;\
	p[index] = localBlockLocation.toFloat();\
	voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);\
	sdf[index] = TVoxel::valueToFloat(voxel.sdf);\
	colorVals[index] = voxel.clr;

	PROCESS_VOXEL(Vector3i(0, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, 1, 1), 0);
	PROCESS_VOXEL(Vector3i(1, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(1, 1, 0), 0);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 0);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 0);
	PROCESS_VOXEL(Vector3i(1, 0, 1), 0);
	PROCESS_VOXEL(Vector3i(1, 1, 1), 0);
#undef PROCESS_VOXEL
}


template<class TVoxel, class TWarpField, class TIndex>
float
ITMSceneMotionTracker_CPU<TVoxel, TWarpField, TIndex>::PerformUpdateIteration(ITMScene<TVoxel, TIndex>* canonicalScene,
                                                                              ITMScene<TVoxel, TIndex>* liveScene,
                                                                              ITMScene<TWarpField, TIndex>* warpField) {

	const TVoxel* liveLocalVBA = liveScene->localVBA.GetVoxelBlocks();
	const TVoxel* warpLocalVBA = warpField->localVBA.GetVoxelBlocks();
	const TVoxel* canonicalLocalVBA = canonicalScene->localVBA.GetVoxelBlocks();

	//should match //TODO: combine warp field and the live scene into a single voxel grid to accelerate lookups
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	const ITMHashEntry* warpHashTable = warpField->index.GetEntries();
	int noTotalLiveEntries = liveScene->index.noTotalEntries;


	for (int entryId = 0; entryId < noTotalLiveEntries; entryId++) {
		Vector3i livePointGlobalPosition;
		const ITMHashEntry& currentLiveHashEntry = liveHashTable[entryId];
		const ITMHashEntry& currentWarpHashEntry = warpHashTable[entryId];

		if (currentLiveHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		livePointGlobalPosition = currentLiveHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3f points[8]; float sdfVals[8]; Vector3u colorVals[8];
					Vector3i livePointLocalPosition(x, y, z);
					Vector3i livePointPosition = livePointGlobalPosition + livePointLocalPosition;
					int vmIndex;
					Vector3f warp_t = readVoxel(warpLocalVBA, warpHashTable, livePointPosition, vmIndex).warp_t;
					Vector3f warpedLivePointLocalPosition = livePointPosition.toFloat() + warp_t;
					//use truncated value here, for it will yield the correct neighbors during lookup
					findPointNeighborsGeneric(points, sdfVals, colorVals, warpedLivePointLocalPosition.toInt(), canonicalLocalVBA, liveHashTable);
					//TODO

				}
			}
		}
	}

	DIEWITHEXCEPTION("Scene tracking iteration not yet implemented");
	return 0;
}





