//  ================================================================
//  Created by Gregory Kramida on 9/25/19.
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


#include "../../../Utils/ITMMath.h"
#include "../../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"

_CPU_AND_GPU_CODE_
inline
bool FindOrAllocateHashEntry(const Vector3s& hashEntryPosition, ITMHashEntry* hashTable, ITMHashEntry*& resultEntry,
                             int& lastFreeVoxelBlockId, int& lastFreeExcessListId, const int* voxelAllocationList,
                             const int* excessAllocationList, int& hash) {
	hash = hashIndex(hashEntryPosition);
	ITMHashEntry hashEntry = hashTable[hash];
	if (!IS_EQUAL3(hashEntry.pos, hashEntryPosition) || hashEntry.ptr < -1) {
		bool isExcess = false;
		//search excess list only if there is no room in ordered part
		if (hashEntry.ptr >= -1) {
			while (hashEntry.offset >= 1) {
				hash = SDF_BUCKET_NUM + hashEntry.offset - 1;
				hashEntry = hashTable[hash];
				if (IS_EQUAL3(hashEntry.pos, hashEntryPosition) && hashEntry.ptr >= -1) {
					resultEntry = &hashTable[hash];
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
			hashTable[hash].offset = exlOffset + 1; //connect to child
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
			hashTable[hash] = newHashEntry;
			resultEntry = &hashTable[hash];
			lastFreeVoxelBlockId--;
			return true;
		} else {
			return false;
		}
	} else {
		//HashEntry already exists, return the pointer to it
		resultEntry = &hashTable[hash];
		return true;
	}
}
