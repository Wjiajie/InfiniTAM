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

//Nota Bene: "STATIC" and "DYNAMIC" in region titles refer to the way functors are used, i.e.
// for "static" traversal, the template argument needs to have a static "run" function, and no actual functor object
// needs to be passed. For "dynamic" traversal, the functor is actually passed in, which can be critical for any
// kind of traversal where some state is updated in a thread-safe manner: this state can be stored as a part of the
// functor.

template<typename TVoxel>
class ITMSceneTraversalEngine<TVoxel, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CUDA> {
public:
// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void StaticVoxelTraversal(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene) {
		TVoxel* voxelArray = scene->localVBA.GetVoxelBlocks();
		const ITMHashEntry* hashTable = scene->index.GetIndexData();
		int hashEntryCount = scene->index.hashEntryCount;

		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize_HashPerBlock(hashEntryCount);

		staticVoxelTraversal_device<TStaticFunctor, TVoxel>
				<< < gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >> > (voxelArray, hashTable);
		ORcudaKernelCheck;
	}

// endregion ===========================================================================================================
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================
	template<typename TFunctor>
	inline static void
	VoxelTraversal(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor) {
		TVoxel* voxelArray = scene->localVBA.GetVoxelBlocks();
		const ITMHashEntry* hashTable = scene->index.GetIndexData();
		int hashEntryCount = scene->index.hashEntryCount;

		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize_HashPerBlock(hashEntryCount);

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		voxelTraversal_device<TFunctor, TVoxel>
				<< < gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >> >
		                                    (voxelArray, hashTable, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}
// endregion ===========================================================================================================
};

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

		int hashEntryCount = primaryScene->index.hashEntryCount;

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
		const ITMHashEntry* primaryHashTable = primaryScene->index.GetIndexData();
		const ITMHashEntry* secondaryHashTable = secondaryScene->index.GetIndexData();

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
		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		int unmatchedHashCount = hashMatchInfo.GetData(MEMORYDEVICE_CPU)->unmatchedHashCount;
		if (unmatchedHashCount > 0) {
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

	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
			TFunctor& functor) {
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		const ITMHashEntry* primaryHashTable = primaryScene->index.GetIndexData();
		const ITMHashEntry* secondaryHashTable = secondaryScene->index.GetIndexData();
		int hashEntryCount = primaryScene->index.hashEntryCount;

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize_HashPerBlock(hashEntryCount);

		dualVoxelPositionTraversal_device<TFunctor, TVoxelPrimary, TVoxelSecondary>
				<< < gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >> >
		                                    (primaryVoxels, secondaryVoxels, primaryHashTable,
				                                    secondaryHashTable, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

};


template<typename TVoxel, typename TWarp>
class ITMDualSceneWarpTraversalEngine<TVoxel, TWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CUDA> {
	/**
	 * \brief Concurrent traversal of 2 scenes with the same voxel type and a warp field
	 */
public:
// region ================================ DYNAMIC TWO-SCENE TRAVERSAL WITH WARPS ======================================


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal(
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* secondaryScene,
			ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
			TFunctor& functor) {
// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();

		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		ITMHashEntry* warpHashTable = warpField->index.GetEntries();

		int hashEntryCount = warpField->index.hashEntryCount;

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize_HashPerBlock(hashEntryCount);

		dualVoxelWarpPositionTraversal_device<TFunctor, TVoxel, TVoxel, TWarp>
				<< < gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >> >
		                                    (primaryVoxels, secondaryVoxels, warpVoxels,
				                                    primaryHashTable, secondaryHashTable, warpHashTable, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

// endregion
};

} // namespace ITMLib



