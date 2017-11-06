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

#include "ITMSceneManipulation.h"
#include "ITMScene.h"
#include "ITMRepresentationAccess.h"

namespace ITMLib {


	template<class TVoxel, class TIndex>
	void CopySceneWithOffset_CPU(ITMScene <TVoxel, TIndex>& destination, ITMScene <TVoxel, TIndex>& source, Vector3i offset) {
		TVoxel* originalVoxels = source.localVBA.GetVoxelBlocks();
		const ITMHashEntry* originalHashTable = source.index.GetEntries();
		typename TIndex::IndexCache originalCache;
		int noTotalEntries = source.index.noTotalEntries;

		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			Vector3i canonicalHashEntryPosition;
			const ITMHashEntry& currentOriginalHashEntry = originalHashTable[entryId];
			if (currentOriginalHashEntry.ptr < 0) continue;

			//position of the current entry in 3D space
			canonicalHashEntryPosition = currentOriginalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

			TVoxel* localVoxelBlock = &(originalVoxels[currentOriginalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
			for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
				for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
					for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
						Vector3i originalPosition = canonicalHashEntryPosition + Vector3i(x, y, z);
						Vector3i offsetPosition = originalPosition + offset;
						int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						SetVoxel_CPU(destination, offsetPosition, localVoxelBlock[locId]);
					}
				}
			}
		}

	}

	template<class TVoxel, class TIndex>
	bool SetVoxel_CPU(ITMScene<TVoxel, TIndex>& scene, Vector3i at, TVoxel voxel) {
		int lastFreeVoxelBlockId = scene.localVBA.lastFreeBlockId;
		int lastFreeExcessListId = scene.index.GetLastFreeExcessListId();
		ITMHashEntry* hashTable = scene.index.GetEntries();
		ITMHashEntry* entry = NULL;
		TVoxel* voxels = scene.localVBA.GetVoxelBlocks();
		int* voxelAllocationList = scene.localVBA.GetAllocationList();
		int* excessAllocationList = scene.index.GetExcessAllocationList();
		Vector3s hashEntryPosition(at.x / SDF_BLOCK_SIZE, at.y / SDF_BLOCK_SIZE, at.z / SDF_BLOCK_SIZE);
		if (AllocateHashEntry_CPU(hashEntryPosition, hashTable, entry,
		                          lastFreeVoxelBlockId, lastFreeExcessListId, voxelAllocationList,
		                          excessAllocationList)) {
			TVoxel* localVoxelBlock = &(voxels[entry->ptr * (SDF_BLOCK_SIZE3)]);
			Vector3s globalPos = entry->pos;
			Vector3i localPos = Vector3i(at.x - globalPos.x * SDF_BLOCK_SIZE,
			                             at.y - globalPos.y * SDF_BLOCK_SIZE,
			                             at.z - globalPos.z * SDF_BLOCK_SIZE);
			int locId = localPos.x + localPos.y * SDF_BLOCK_SIZE + localPos.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
			localVoxelBlock[locId] = voxel;
		} else {
			return false;
		}
		scene.localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
		scene.index.SetLastFreeExcessListId(lastFreeExcessListId);
		return true;
	};

	template<class TVoxel, class TIndex>
	TVoxel ReadVoxel(ITMScene<TVoxel, TIndex>& scene, Vector3i at) {
		TVoxel* voxels = scene.localVBA.GetVoxelBlocks();
		ITMHashEntry* hashTable = scene.index.GetEntries();
		int vmIndex;
		return readVoxel(voxels, hashTable, at, vmIndex);
	};
}