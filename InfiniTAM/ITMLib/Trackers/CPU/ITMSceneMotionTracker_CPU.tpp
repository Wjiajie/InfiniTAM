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

inline void interpolateTrilinear(float& sdf, Vector3u& color, float* sdfVals, Vector3u* colorVals, Vector3f* points, Vector3f pointPosition ){
	const int neighborCount = 8;
	Vector3f colorValsF[neighborCount];
	for(int iVoxel = 0; iVoxel < neighborCount; iVoxel++){
		colorValsF[iVoxel] = colorVals[iVoxel].toFloat();
	}
	Vector3f ratios = (pointPosition - points[0]) / (points[7] - points[0]);
	Vector3f invRatios = Vector3f(1.f) - ratios;
	Vector3f colorF;
#define INTERPOLATE_TRILINEAR(type,prefix,output,array,ratios,invRatios)\
					type prefix##_00 = array[0]*invRatios.x + array[4]*ratios.x;\
					type prefix##_01 = array[1]*invRatios.x + array[5]*ratios.x;\
					type prefix##_10 = array[2]*invRatios.x + array[6]*ratios.x;\
					type prefix##_11 = array[3]*invRatios.x + array[7]*ratios.x;\
					type prefix##_0 = prefix##_00*invRatios.y + prefix##_10*ratios.y;\
					type prefix##_1 = prefix##_01*invRatios.y + prefix##_11*ratios.y;\
					output = prefix##_0*invRatios.z + prefix##_1 * ratios.z;
	INTERPOLATE_TRILINEAR(float,sdf,sdf,sdfVals,ratios,invRatios);
	INTERPOLATE_TRILINEAR(Vector3f,color,colorF,colorValsF,ratios,invRatios);
#undef INTERPOLATE_TRILINEAR
	color = colorF.toUChar();
}


template<class TVoxel, class TWarpField, class TIndex>
float
ITMSceneMotionTracker_CPU<TVoxel, TWarpField, TIndex>::PerformUpdateIteration(ITMScene<TVoxel, TIndex>* canonicalScene,
                                                                              ITMScene<TVoxel, TIndex>* liveScene,
                                                                              ITMScene<TWarpField, TIndex>* warpField) {

	const TVoxel* liveLocalVBA = liveScene->localVBA.GetVoxelBlocks();
	const TWarpField* warpLocalVBA = warpField->localVBA.GetVoxelBlocks();
	const TVoxel* canonicalLocalVBA = canonicalScene->localVBA.GetVoxelBlocks();

	//should match //TODO: combine warp field and the live scene into a single voxel grid to accelerate lookups
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	const ITMHashEntry* warpHashTable = warpField->index.GetEntries();
	int noTotalLiveEntries = liveScene->index.noTotalEntries;


	for (int entryId = 0; entryId < noTotalLiveEntries; entryId++) {
		Vector3i liveHashEntryPosition;
		const ITMHashEntry& currentLiveHashEntry = liveHashTable[entryId];
		const ITMHashEntry& currentWarpHashEntry = warpHashTable[entryId];

		if (currentLiveHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		liveHashEntryPosition = currentLiveHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					const int neighborCount = 8;
					Vector3f points[neighborCount]; float sdfVals[neighborCount]; Vector3u colorVals[neighborCount];

					Vector3i livePointPosition = liveHashEntryPosition + Vector3i(x, y, z);
					int vmIndex;
					Vector3f warp_t = readVoxel(warpLocalVBA, warpHashTable, livePointPosition, vmIndex).warp_t;
					Vector3f warpedLivePointPosition = livePointPosition.toFloat() + warp_t;
					//use truncated value here, for it will yield the correct neighbors during lookup
					findPointNeighborsGeneric(points, sdfVals, colorVals, warpedLivePointPosition.toInt(), canonicalLocalVBA, liveHashTable);
					float sdfCanonical; Vector3u colorCanonical;
					interpolateTrilinear(sdfCanonical,colorCanonical,sdfVals,colorVals,points,warpedLivePointPosition);

				}
			}
		}
	}

	DIEWITHEXCEPTION("Scene tracking iteration not yet implemented");
	return 0;
}





