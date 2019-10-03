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
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMVoxelBlockHash, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CUDA>{
	template<typename TBooleanFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
			TBooleanFunctor& functor) {

		// allocate boolean varaible for answer
		bool* falseEncountered_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &falseEncountered_device, sizeof(bool)));
		ORcudaSafeCall(cudaMemset(falseEncountered_device, 0, sizeof(bool)));

		// transfer functor from RAM to VRAM
		TBooleanFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TBooleanFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TBooleanFunctor), cudaMemcpyHostToDevice));


		// perform traversal on the GPU
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		const ITMHashEntry* primaryHashTable = primaryScene->index.getIndexData();
		const ITMHashEntry* secondaryHashTable = secondaryScene->index.getIndexData();
		int totalPrimaryHashEntryCount = primaryScene->index.noTotalEntries;
		dim3 cudaBlockSize(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
		dim3 gridSize(totalPrimaryHashEntryCount);

		dualVoxelTraversal_AllTrue_device<TBooleanFunctor, TVoxelPrimary, TVoxelSecondary>
		<< < gridSize, cudaBlockSize >> >
		(primaryVoxels, secondaryVoxels, primaryHashTable, secondaryHashTable, functor_device, falseEncountered_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TBooleanFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));

		bool falseEncountered;
		ORcudaSafeCall(cudaMemcpy(&falseEncountered, falseEncountered_device, sizeof(bool), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(falseEncountered_device));

		return !falseEncountered;
	}


};

} // namespace ITMLib



