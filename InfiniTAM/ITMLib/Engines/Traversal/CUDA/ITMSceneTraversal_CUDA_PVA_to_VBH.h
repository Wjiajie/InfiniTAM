//  ================================================================
//  Created by Gregory Kramida on 10/4/19.
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


//local
#include "../Shared/ITMSceneTraversal_Shared.h"
#include "../Interface/ITMSceneTraversal.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Utils/Analytics/ITMIsAltered.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "ITMSceneTraversal_CUDA_PVA_to_VBH_Kernels.h"

namespace ITMLib {


template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMPlainVoxelArray, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CUDA> {
public:
	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param primaryVolume the primary volume -- indexed using plain voxel array (PVA)
	 * \param secondaryVolume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TBooleanFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			ITMVoxelVolume<TVoxelPrimary, ITMPlainVoxelArray>* primaryVolume,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryVolume,
			TBooleanFunctor& functor) {
		ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo = primaryVolume->index.getIndexData();
		int hashEntryCount = secondaryVolume->index.noTotalEntries;
		ITMHashEntry* hashTable = secondaryVolume->index.getIndexData();

		ORUtils::MemoryBlock<int> hashesNotSpanned(hashEntryCount, true, true);
		ORUtils::MemoryBlock<int> countHashesNotSpanned(1, true, true);
		*countHashesNotSpanned.GetData(MEMORYDEVICE_CPU) = 0;
		countHashesNotSpanned.UpdateDeviceFromHost();

		dim3 cudaBlockSize_HashPerThread(256);
		dim3 gridSize_MultipleHashBlocks(static_cast<int>(ceil(static_cast<float>(hashEntryCount) /
		                                                       static_cast<float>(cudaBlockSize_HashPerThread.x))));
		findBlocksNotSpannedByArray << < gridSize_MultipleHashBlocks, cudaBlockSize_HashPerThread >> >
		                                                              (arrayInfo, hashTable, hashEntryCount,
				                                                              hashesNotSpanned.GetData(
						                                                              MEMORYDEVICE_CUDA),
				                                                              countHashesNotSpanned.GetData(
						                                                              MEMORYDEVICE_CUDA));
		ORcudaKernelCheck;
		countHashesNotSpanned.UpdateHostFromDevice();
		int countHashesNotSpanned_host = *countHashesNotSpanned.GetData(MEMORYDEVICE_CPU);

		//*** used in the next two kernels
		ORUtils::MemoryBlock<bool> falseOrAlteredEncountered(1, true, true);
		*falseOrAlteredEncountered.GetData(MEMORYDEVICE_CPU) = false;
		falseOrAlteredEncountered.UpdateDeviceFromHost();
		dim3 cudaBlockSize_BlockVoxelPerThread(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
		TVoxelSecondary* voxelsHash = secondaryVolume->localVBA.GetVoxelBlocks();

		if (countHashesNotSpanned_host > 0) {
			// if there are any blocks in the hash that are not spanned by the array volume, we must check
			// whether any of them are altered
			dim3 gridSize_UnspannedHashBlocks(static_cast<int>(ceil(static_cast<float>(countHashesNotSpanned_host) /
			                                                        static_cast<float>(cudaBlockSize_HashPerThread.x))));
			checkIfHashVoxelBlocksAreAltered
					<< < gridSize_UnspannedHashBlocks, cudaBlockSize_BlockVoxelPerThread >> >
			                                           (voxelsHash, hashTable, hashesNotSpanned.GetData(
					                                           MEMORYDEVICE_CUDA),
					                                           countHashesNotSpanned_host,
					                                           falseOrAlteredEncountered.GetData(
							                                           MEMORYDEVICE_CUDA));
			ORcudaKernelCheck;
		}
		falseOrAlteredEncountered.UpdateHostFromDevice();
		if (*falseOrAlteredEncountered.GetData(MEMORYDEVICE_CPU)) {
			return false;
		}

		TVoxelPrimary* voxelsArray = primaryVolume->localVBA.GetVoxelBlocks();
		Vector3i arrayOffset = primaryVolume->index.getVolumeOffset();
		Vector3i arraySize = primaryVolume->index.getVolumeSize();

		Vector3s minBlockPos(
				static_cast<int>(floor(static_cast<float>(arrayOffset.x) / SDF_BLOCK_SIZE)),
				static_cast<int>(floor(static_cast<float>(arrayOffset.y) / SDF_BLOCK_SIZE)),
				static_cast<int>(floor(static_cast<float>(arrayOffset.z) / SDF_BLOCK_SIZE)));
		Vector3i minIndexCoord(minBlockPos.x * SDF_BLOCK_SIZE,
		                       minBlockPos.y * SDF_BLOCK_SIZE,
		                       minBlockPos.z * SDF_BLOCK_SIZE);
		Vector3i minArrayCoord = arrayOffset - minIndexCoord; // inclusive
		Vector3i maxArrayCoord = minArrayCoord + arraySize; // exclusive

		// transfer functor from RAM to VRAM (has to be done manually, i.e. without MemoryBlock at this point)
		TBooleanFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TBooleanFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TBooleanFunctor), cudaMemcpyHostToDevice));

		dim3 gridSize_ArrayBlockEnvelope(
				static_cast<int>(ceil(static_cast<float>(maxArrayCoord.x) / SDF_BLOCK_SIZE)),
				static_cast<int>(ceil(static_cast<float>(maxArrayCoord.y) / SDF_BLOCK_SIZE)),
				static_cast<int>(ceil(static_cast<float>(maxArrayCoord.z) / SDF_BLOCK_SIZE))
		);

		checkIfArrayContentIsUnalteredOrYieldsTrue
				<< < gridSize_ArrayBlockEnvelope, cudaBlockSize_BlockVoxelPerThread >> >
		                                          (voxelsArray, arrayInfo, voxelsHash, hashTable, hashEntryCount,
				                                          minArrayCoord, maxArrayCoord, minBlockPos, functor_device,
				                                          falseOrAlteredEncountered.GetData(MEMORYDEVICE_CUDA));
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM (in case there was any alteration to the functor object)
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TBooleanFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
		falseOrAlteredEncountered.UpdateHostFromDevice();

		return !(*falseOrAlteredEncountered.GetData(MEMORYDEVICE_CPU));

	}


};


template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMVoxelBlockHash, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CUDA> {
public:
	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param primaryVolume the primary volume -- indexed using plain voxel array (PVA)
	 * \param secondaryVolume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryVolume,
			ITMVoxelVolume<TVoxelSecondary, ITMPlainVoxelArray>* secondaryVolume,
			TFunctor& functor) {
		ITMFlipArgumentBooleanFunctor<TVoxelPrimary, TVoxelSecondary, TFunctor> flipFunctor(functor);
		return ITMDualSceneTraversalEngine<TVoxelSecondary, TVoxelPrimary, ITMPlainVoxelArray, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CUDA>::
		DualVoxelTraversal_AllTrue(secondaryVolume, primaryVolume, flipFunctor);

	}


	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue_AllocatedOnly(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryVolume,
			ITMVoxelVolume<TVoxelSecondary, ITMPlainVoxelArray>* secondaryVolume,
			TFunctor& functor) {
		ITMFlipArgumentBooleanFunctor<TVoxelPrimary, TVoxelSecondary, TFunctor> flipFunctor(functor);
		return ITMDualSceneTraversalEngine<TVoxelSecondary, TVoxelPrimary, ITMPlainVoxelArray, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CUDA>::
		DualVoxelTraversal_AllTrue_AllocatedOnly(secondaryVolume, primaryVolume, flipFunctor);
	}
};
}//namespace ITMLib
