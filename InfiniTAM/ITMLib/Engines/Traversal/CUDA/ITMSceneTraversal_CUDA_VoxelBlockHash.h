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
#include "ITMSceneTraversal_CUDA_VoxelBlockHash_Kernels.h"
#include "../Interface/ITMSceneTraversal.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"

namespace ITMLib {

template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMVoxelBlockHash, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CUDA> {
public:
	template<typename TBooleanFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
			TBooleanFunctor& functor) {
		// assumes functor is allocated in main memory

		int hashEntryCount = primaryScene->index.noTotalEntries;

		// allocate intermediate-result buffers for use on the GPU in subsequent routine calls
		ORUtils::MemoryBlock<bool> badResultEncountered_device(1, true, true);
		*badResultEncountered_device.GetData(MEMORYDEVICE_CPU) = false;
		badResultEncountered_device.UpdateDeviceFromHost();
		ORUtils::MemoryBlock<HashMatchInfo> hashMatchInfo(1, true, true);
		hashMatchInfo.GetData(MEMORYDEVICE_CPU)->matchedHashCount = 0;
		hashMatchInfo.GetData(MEMORYDEVICE_CPU)->unmatchedHashCount = 0;
		hashMatchInfo.UpdateDeviceFromHost();
		ORUtils::MemoryBlock<HashPair> matchedHashPairs(hashEntryCount, true, true);
		ORUtils::MemoryBlock<UnmatchedHash> unmatchedHashes(hashEntryCount, true, true);

		// these will be needed for various matching & traversal operations
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		const ITMHashEntry* primaryHashTable = primaryScene->index.getIndexData();
		const ITMHashEntry* secondaryHashTable = secondaryScene->index.getIndexData();

		dim3 cudaBlockSize_HashPerThread(256, 1);
		dim3 gridSize_MultipleHashBlocks(static_cast<int>(ceil(static_cast<float>(hashEntryCount) /
		                                                       static_cast<float>(cudaBlockSize_HashPerThread.x))));

		matchUpHashEntriesByPosition
				<< < gridSize_MultipleHashBlocks, cudaBlockSize_HashPerThread >> >
		                                          (primaryHashTable, secondaryHashTable, hashEntryCount,
				                                          matchedHashPairs.GetData(MEMORYDEVICE_CUDA),
				                                          unmatchedHashes.GetData(MEMORYDEVICE_CUDA),
				                                          hashMatchInfo.GetData(MEMORYDEVICE_CUDA));
		ORcudaKernelCheck;

		// check unmatched hashes
		hashMatchInfo.UpdateHostFromDevice();
		dim3 cudaBlockSize_BlockVoxelPerThread(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
		int unmatchedHashCount = hashMatchInfo.GetData(MEMORYDEVICE_CPU)->unmatchedHashCount;
		if(unmatchedHashCount > 0){
			dim3 gridSize_UnmatchedBlocks(unmatchedHashCount);
			checkIfUnmatchedVoxelBlocksAreAltered
					<< < gridSize_UnmatchedBlocks, cudaBlockSize_BlockVoxelPerThread >> >
			                                       (primaryVoxels, secondaryVoxels,
					                                       primaryHashTable, secondaryHashTable,
					                                       unmatchedHashes.GetData(MEMORYDEVICE_CUDA),
					                                       hashMatchInfo.GetData(MEMORYDEVICE_CUDA),
					                                       badResultEncountered_device.GetData(MEMORYDEVICE_CUDA));
			ORcudaKernelCheck;
		}

		// if an unmatched block in either volume was altered, return false
		badResultEncountered_device.UpdateHostFromDevice();
		if (*badResultEncountered_device.GetData(MEMORYDEVICE_CPU)) return false;

		int matchedHashCount = hashMatchInfo.GetData(MEMORYDEVICE_CPU)->matchedHashCount;

		if (matchedHashCount == 0) return true;

		// transfer functor from RAM to VRAM (has to be done manually, i.e. without MemoryBlock at this point)
		TBooleanFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TBooleanFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TBooleanFunctor), cudaMemcpyHostToDevice));

		// perform voxel traversal on the GPU, matching individual voxels at corresponding locations
		dim3 gridSize_MatchedBlocks(matchedHashCount);
		checkIfMatchingHashBlockVoxelsYieldTrue<TBooleanFunctor, TVoxelPrimary, TVoxelSecondary>
				<< < gridSize_MatchedBlocks, cudaBlockSize_BlockVoxelPerThread >> >
		                                     (primaryVoxels, secondaryVoxels,
				                                     primaryHashTable, secondaryHashTable,
				                                     matchedHashPairs.GetData(MEMORYDEVICE_CUDA),
				                                     hashMatchInfo.GetData(MEMORYDEVICE_CUDA),
				                                     functor_device,
				                                     badResultEncountered_device.GetData(MEMORYDEVICE_CUDA));
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM (in case there was any alteration to the functor object)
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TBooleanFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));

		badResultEncountered_device.UpdateHostFromDevice();
		return !(*badResultEncountered_device.GetData(MEMORYDEVICE_CPU));
	}


};

} // namespace ITMLib



