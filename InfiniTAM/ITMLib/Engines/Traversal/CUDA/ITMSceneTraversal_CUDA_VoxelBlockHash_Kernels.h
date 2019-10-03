//  ================================================================
//  Created by Gregory Kramida on 8/13/19.
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

#include "../../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../../../ORUtils/JetbrainsCUDASyntax.hpp"
#include "../Shared/ITMSceneTraversal_Shared.h"
#include "../../../Utils/Analytics/ITMIsAltered.h"



struct HashPair {
	int primaryHash;
	int secondaryHash;
};

struct UnmatchedHash{
	int hash;
	bool primary;
};

struct HashMatchInfo{
	int matchedHashCount;
	int unmatchedHashCount;
};

namespace {
__global__ void matchUpHashEntriesByPosition(const ITMHashEntry* primaryHashTable,
                                             const ITMHashEntry* secondaryHashTable,
                                             int totalHashEntryCount,
                                             HashPair* hashPairs,
                                             UnmatchedHash* unmatchedHashes,
                                             HashMatchInfo* hashMatchInfo) {

	int hash = blockIdx.x * blockDim.x + threadIdx.x;
	if (hash > totalHashEntryCount) return;

	const ITMHashEntry& primaryHashEntry = primaryHashTable[hash];
	const ITMHashEntry secondaryHashEntry = secondaryHashTable[hash];
	int secondaryHash = hash;
	if (primaryHashEntry.ptr < 0) {
		if (secondaryHashEntry.ptr < 0) {
			// neither primary nor secondary hash blocks allocated for this hash
			return;
		} else {
			int alternativePrimaryHash;
			// found no primary at hash index, but did find secondary. Ensure we have a matching primary.
			if (!FindHashAtPosition(alternativePrimaryHash, secondaryHashEntry.pos, primaryHashTable)) {
				int unmatchedHashIdx = atomicAdd(hashMatchInfo->unmatchedHashCount);
				unmatchedHashes[unmatchedHashIdx].hash = hash;
				unmatchedHashes[unmatchedHashIdx].primary = false;
				return;
			} else {
				// found a matching primary, meaning secondary hash will be processed by a different thread
				return;
			}
		}
	}

	// the rare case where we have different positions for primary & secondary voxel block with the same index:
	// we have a hash bucket miss, find the secondary voxel block with the matching coordinates
	if (secondaryHashEntry.pos != primaryHashEntry.pos) {
		if (secondaryHashEntry.ptr >= 0) {
			int alternativePrimaryHash;
			if (!FindHashAtPosition(alternativePrimaryHash, secondaryHashEntry.pos, primaryHashTable)) {
				int unmatchedHashIdx = atomicAdd(hashMatchInfo->unmatchedHashCount);
				unmatchedHashes[unmatchedHashIdx].hash = hash;
				unmatchedHashes[unmatchedHashIdx].primary = false;
				return;
			}
		}

		if (!FindHashAtPosition(secondaryHash, primaryHashEntry.pos, secondaryHashTable)) {
			// If we cannot find the matching secondary hash, we will check whether the primary voxel block has been altered later
			int unmatchedHashIdx = atomicAdd(hashMatchInfo->unmatchedHashCount);
			unmatchedHashes[unmatchedHashIdx].hash = hash;
			unmatchedHashes[unmatchedHashIdx].primary = true;
			return;
		}
	}

	int matchedPairIdx = atomicAdd(hashMatchInfo->matchedHashCount);
	hashPairs[matchedPairIdx].primaryHash = hash;
	hashPairs[matchedPairIdx].secondaryHash = secondaryHash;
}

template<typename TBooleanFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void dualVoxelTraversal_AllTrue_device(
		TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
		const ITMHashEntry* primaryHashTable, const ITMHashEntry* secondaryHashTable,
		int totalHashEntryCount, TBooleanFunctor* functor, bool* falseEncountered) {


	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;


}

}// end anonymous namespace (CUDA kernels)
