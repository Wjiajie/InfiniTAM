//  ================================================================
//  Created by Gregory Kramida on 11/3/17.
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
#include "ITMSceneManipulation.h"
#include "../../Utils/ITMMath.h"
#include "ITMVoxelBlockHash.h"
#include "ITMRepresentationAccess.h"
#include "../../ITMLibDefines.h"
#include "ITMScene.h"

namespace ITMLib {

	bool AllocateHashEntry_CPU(const Vector3s& hashEntryPosition,
	                           ITMHashEntry* hashTable,
	                           ITMHashEntry*& resultEntry,
	                           int& lastFreeVoxelBlockId,
	                           int& lastFreeExcessListId,
	                           const int* voxelAllocationList,
	                           const int* excessAllocationList) {
		int resultEntryIndex = hashIndex(hashEntryPosition);
		ITMHashEntry hashEntry = hashTable[resultEntryIndex];
		if (!IS_EQUAL3(hashEntry.pos, hashEntryPosition) || hashEntry.ptr < -1) {
			bool isExcess = false;
			//seach excess list only if there is no room in ordered part
			if (hashEntry.ptr >= -1) {
				while (hashEntry.offset >= 1) {
					resultEntryIndex = SDF_BUCKET_NUM + hashEntry.offset - 1;
					hashEntry = hashTable[resultEntryIndex];
					if (IS_EQUAL3(hashEntry.pos, hashEntryPosition) && hashEntry.ptr >= -1) {
						resultEntry = &hashTable[resultEntryIndex];
						return true;
					}
				}
				isExcess = true;

			}
			//still not found, allocate
			if (isExcess && lastFreeVoxelBlockId >= 0 && lastFreeExcessListId >= 0) {
				//there is room in the voxel block array and excess list
				ITMHashEntry newHashEntry;
				newHashEntry.pos = hashEntryPosition;
				newHashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
				newHashEntry.offset = 0;
				int exlOffset = excessAllocationList[lastFreeExcessListId];
				hashTable[resultEntryIndex].offset = exlOffset + 1; //connect to child
				hashTable[SDF_BUCKET_NUM +
				          exlOffset] = newHashEntry; //add child to the excess list
				resultEntry = &hashTable[SDF_BUCKET_NUM +
				                         exlOffset];
				lastFreeVoxelBlockId--;
				lastFreeExcessListId--;
				return true;
			} else if (lastFreeVoxelBlockId >= 0) {
				//there is room in the voxel block array
				ITMHashEntry newHashEntry;
				newHashEntry.pos = hashEntryPosition;
				newHashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
				newHashEntry.offset = 0;
				hashTable[resultEntryIndex] = newHashEntry;
				resultEntry = &hashTable[resultEntryIndex];
				lastFreeVoxelBlockId--;
				return true;
			}else{
				return false;
			}
		} else {
			//HashEntry already exists, return the pointer to it
			resultEntry = &hashTable[resultEntryIndex];
			return true;
		}
	}

	void CopySceneWithOffset_CPU(ITMScene<ITMVoxelAux, ITMVoxelIndex>& destination, ITMScene<ITMVoxel, ITMVoxelIndex>& source,
	                             Vector3i offset) {
		ITMVoxel* originalVoxels = source.localVBA.GetVoxelBlocks();
		const ITMHashEntry* originalHashTable = source.index.GetEntries();
		int noTotalEntries = source.index.noTotalEntries;
		int sdfBelow1Count = 0;

		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			Vector3i canonicalHashEntryPosition;
			const ITMHashEntry& currentOriginalHashEntry = originalHashTable[entryId];
			if (currentOriginalHashEntry.ptr < 0) continue;

			//position of the current entry in 3D space
			canonicalHashEntryPosition = currentOriginalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

			ITMVoxel* localVoxelBlock = &(originalVoxels[currentOriginalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
			for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
				for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
					for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
						Vector3i originalPosition = canonicalHashEntryPosition + Vector3i(x, y, z);
						Vector3i offsetPosition = originalPosition + offset;

						//_DEBUG
						//Vector3i test(-135,-216,574);
//						Vector3i test(-136,-215,575);
//						if(originalPosition == test){
//							std::cout << "HI" << std::endl;
//						}

						int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						ITMVoxel& voxelSource = localVoxelBlock[locId];
						ITMVoxelAux voxelDest;
						voxelDest.sdf = voxelSource.sdf;
						//_DEBUG
						if((1.0f - fabs(voxelSource.sdf)) > 1.0e-10f){
							sdfBelow1Count++;
						}
						voxelDest.clr = voxelSource.clr;
						voxelDest.w_color = voxelSource.w_color;
						voxelDest.w_depth = voxelSource.w_depth;
						voxelDest.confidence = voxelSource.confidence;
						SetVoxel_CPU<ITMVoxelAux, ITMVoxelIndex>(destination, offsetPosition, voxelDest);
					}
				}
			}
		}
		//_DEBUG
		std::cout << "Count of voxels with ||SDF|| < 1.0 in source: " << sdfBelow1Count << std::endl;
	}
}//namespace ITMLib