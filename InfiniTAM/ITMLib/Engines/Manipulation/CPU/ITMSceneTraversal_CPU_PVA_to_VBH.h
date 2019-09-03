//  ================================================================
//  Created by Gregory Kramida on 8/29/19.
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
#include "ITMSceneTraversal_CPU_AuxilaryFunctions.h"
#include "../Interface/ITMSceneTraversal.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Utils/Analytics/ITMIsAltered.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"

namespace ITMLib {



template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMPlainVoxelArray, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU> {

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
			ITMVoxelVolume<TVoxelPrimary, ITMPlainVoxelArray>* primaryVolume,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryVolume,
			TFunctor& functor) {
// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryVolume->localVBA.GetVoxelBlocks();

		TVoxelPrimary* primaryVoxels = primaryVolume->localVBA.GetVoxelBlocks();
		int voxelCount = primaryVolume->index.getVolumeSize().x * primaryVolume->index.getVolumeSize().y *
		                 primaryVolume->index.getVolumeSize().z;
		const ITMVoxelBlockHash::IndexData* hashTable = secondaryVolume->index.getIndexData();
		const ITMPlainVoxelArray::IndexData* pvaIndexData = primaryVolume->index.getIndexData();
		Vector3i pvaOffset = pvaIndexData->offset;
		Vector3i pvaSize = pvaIndexData->size;
		Vector3i endVoxel = pvaOffset + pvaSize; // open last traversal bound (this voxel doesn't get processed)

		//determine the opposite corners of the 3d extent, [start, end), in voxel hash blocks, to traverse.
		Vector3s startBlockCoords(pvaOffset.x / SDF_BLOCK_SIZE - (pvaOffset.x % SDF_BLOCK_SIZE != 0),
		                          pvaOffset.y / SDF_BLOCK_SIZE - (pvaOffset.y % SDF_BLOCK_SIZE != 0),
		                          pvaOffset.z / SDF_BLOCK_SIZE - (pvaOffset.z % SDF_BLOCK_SIZE != 0));
		Vector3s endBlockCoords((endVoxel.x - 1) / SDF_BLOCK_SIZE + ((endVoxel.x - 1) % SDF_BLOCK_SIZE != 0) + 1,
		                        (endVoxel.y - 1) / SDF_BLOCK_SIZE + ((endVoxel.y - 1) % SDF_BLOCK_SIZE != 0) + 1,
		                        (endVoxel.z - 1) / SDF_BLOCK_SIZE + ((endVoxel.z - 1) % SDF_BLOCK_SIZE != 0) + 1);


		ORUtils::MemoryBlock<bool> hashBlockUsed_MemBlock(ITMVoxelBlockHash::noTotalEntries, MEMORYDEVICE_CPU);
		bool* hashBlocksUsed = hashBlockUsed_MemBlock.GetData(MEMORYDEVICE_CPU);
		memset(hashBlocksUsed, false, static_cast<size_t>(ITMVoxelBlockHash::noTotalEntries));

		int linearIndex = 0;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (short zVoxelBlock = startBlockCoords.z; zVoxelBlock < endBlockCoords.z; zVoxelBlock++) {
			int zVoxelStart = std::max(zVoxelBlock * SDF_BLOCK_SIZE, pvaOffset.y);
			int zVoxelEnd = std::min((zVoxelBlock + 1) * SDF_BLOCK_SIZE, endVoxel.y);
			for (short yVoxelBlock = startBlockCoords.y; yVoxelBlock < endBlockCoords.y; yVoxelBlock++) {
				int yVoxelStart = std::max(yVoxelBlock * SDF_BLOCK_SIZE, pvaOffset.y);
				int yVoxelEnd = std::min((yVoxelBlock + 1) * SDF_BLOCK_SIZE, endVoxel.y);
				for (short xVoxelBlock = startBlockCoords.x; xVoxelBlock < endBlockCoords.x; xVoxelBlock++) {
					int xVoxelStart = std::max(xVoxelBlock * SDF_BLOCK_SIZE, pvaOffset.x);
					int xVoxelEnd = std::min((xVoxelBlock + 1) * SDF_BLOCK_SIZE, endVoxel.x);
					Vector3s voxelBlockCoords(xVoxelBlock, yVoxelBlock, zVoxelBlock);
					int voxelBlockHash;
					bool voxelBlockAllocated = false;
					TVoxelPrimary* localSecondaryVoxelBlock = nullptr;
					// see if the voxel block is allocated at all in VBH-indexed volume
					if (FindHashAtPosition(voxelBlockHash, voxelBlockCoords, hashTable)) {
						voxelBlockAllocated = true;
						localSecondaryVoxelBlock = &(secondaryVoxels[hashTable[voxelBlockHash].ptr *
						                                             (SDF_BLOCK_SIZE3)]);
					}
					// traverse the current voxel block volume to see if PVA-indexed volume's voxels satisfy the boolean
					// functional when matched with corresponding voxels from the VBH-indexed volume.
					// If the block is not allocated in the VBH volume at all but the PVA voxel is modified,
					// short-circuit to "false"
					int locId = 0;
					for (int z = zVoxelStart; z < zVoxelEnd; z++) {
						for (int y = yVoxelStart; y < yVoxelEnd; y++) {
							for (int x = xVoxelStart; x < xVoxelEnd; x++) {
								Vector3i voxelPosition = Vector3i(x, y, z);
								//TODO: remove the check once you run this on some non-empty scenes
								Vector3i gtVoxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(
										pvaIndexData, linearIndex);
								if (gtVoxelPosition != voxelPosition) {
									DIEWITHEXCEPTION_REPORTLOCATION(
											"The voxel positions didn't match -- this routine has a logic error.");
								}
								TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
								if (voxelBlockAllocated) {
									TVoxelSecondary& secondaryVoxel = localSecondaryVoxelBlock[locId];
									if (!(functor(primaryVoxel, secondaryVoxel))) {
										return false;
									}
								} else {
									if (isAltered(primaryVoxel)) {
										return false;
									}
								}
								linearIndex++;
								locId++;
							}
						}
					}
				}
			}
		}
		return true;
	}

};

template<typename TVoxelPrimary, typename TVoxelSecondary, typename TFunctor>
struct ITMFlipArgumentBooleanFunctor{
	ITMFlipArgumentBooleanFunctor(TFunctor functor) : functor(functor){}
	bool operator()(TVoxelPrimary& voxelPrimary, TVoxelSecondary& voxelSecondary){
		return  functor(voxelSecondary,voxelPrimary);
	}
	TFunctor functor;
};

template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMVoxelBlockHash, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU> {

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
			ITMVoxelVolume<TVoxelPrimary, ITMPlainVoxelArray>* primaryVolume,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryVolume,
			TFunctor& functor) {
		auto flipFunctor(functor);
		return ITMDualSceneTraversalEngine<TVoxelSecondary, TVoxelPrimary, ITMPlainVoxelArray, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
		        DualVoxelTraversal_AllTrue(secondaryVolume,primaryVolume,flipFunctor);

	}
};
} // namespace ITMLib