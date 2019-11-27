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
//stdlib
#include <unordered_set>

//local
#include "../Shared/ITMSceneTraversal_Shared.h"
#include "../Interface/ITMSceneTraversal.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Utils/Analytics/ITMIsAltered.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"

namespace ITMLib {


template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMPlainVoxelArray, ITMVoxelBlockHash, MEMORYDEVICE_CPU> {
private:

	template<typename TFunctor, typename TFunctionCall>
	inline static bool
	DualVoxelTraversal_AllTrue_Generic(
			ITMVoxelVolume <TVoxelPrimary, ITMPlainVoxelArray>* primaryVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMVoxelBlockHash>* secondaryVolume,
			TFunctor& functor, TFunctionCall&& functionCall
	) {

	}

	template<typename TFunctor, typename TFunctionCall>
	inline static bool
	DualVoxelTraversal_AllTrue_MatchingFlags_Generic(
			ITMVoxelVolume <TVoxelPrimary, ITMPlainVoxelArray>* arrayVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMVoxelBlockHash>* hashVolume,
			VoxelFlags flags,
			TFunctor& functor, TFunctionCall&& functionCall
	) {


	}

	template<typename TFunctor, typename TFunctionCall>
	inline static bool
	DualVoxelTraversal_AllTrue_AllocatedOnly_Generic(
			ITMVoxelVolume <TVoxelPrimary, ITMPlainVoxelArray>* primaryVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMVoxelBlockHash>* secondaryVolume,
			TFunctor& functor, TFunctionCall&& functionCall) {
		volatile bool foundMismatch = false;
		int totalHashEntryCount = secondaryVolume->index.hashEntryCount;
		TVoxelSecondary* secondaryVoxels = secondaryVolume->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryVolume->localVBA.GetVoxelBlocks();
		const ITMVoxelBlockHash::IndexData* hashTable = secondaryVolume->index.GetIndexData();
		const ITMPlainVoxelArray::IndexData* pvaIndexData = primaryVolume->index.GetIndexData();
		Vector3i startVoxel = pvaIndexData->offset;
		Vector3i pvaSize = pvaIndexData->size;
		Vector3i endVoxel = startVoxel + pvaSize; // open last traversal bound (this voxel doesn't get processed)
		Vector6i pvaBounds(startVoxel.x, startVoxel.y, startVoxel.z, endVoxel.x, endVoxel.y, endVoxel.z);

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(foundMismatch)
#endif
		for (int hash = 0; hash < totalHashEntryCount; hash++) {
			if (foundMismatch) continue;
			ITMHashEntry secondaryHashEntry = hashTable[hash];
			if (secondaryHashEntry.ptr < 0) continue;
			Vector3i voxelBlockMinPoint = secondaryHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hashEntryMaxPoint = voxelBlockMinPoint + Vector3i(VOXEL_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(voxelBlockMinPoint, hashEntryMaxPoint, pvaBounds)) {
				continue;
			}
			TVoxelSecondary* secondaryVoxelBlock = &(secondaryVoxels[secondaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			Vector6i localBounds = computeLocalBounds(voxelBlockMinPoint, hashEntryMaxPoint, pvaBounds);
			for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
				for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
					for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelCoordinateWithinBlock = Vector3i(x, y, z);
						Vector3i voxelAbsolutePosition = voxelBlockMinPoint + voxelCoordinateWithinBlock;
						Vector3i voxelPositionSansOffset = voxelAbsolutePosition - startVoxel;
						int linearIndex = voxelPositionSansOffset.x + voxelPositionSansOffset.y * pvaSize.x +
						                  voxelPositionSansOffset.z * pvaSize.x * pvaSize.y;
						TVoxelSecondary& secondaryVoxel = secondaryVoxelBlock[locId];
						TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];

						if (!std::forward<TFunctionCall>(functionCall)(functor, primaryVoxel, secondaryVoxel,
						                                               voxelAbsolutePosition)) {
							foundMismatch = true;
							break;
						}
					}
				}
			}
		}
		return !foundMismatch;

	}

public:
	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param arrayVolume the primary volume -- indexed using plain voxel array (PVA)
	 * \param hashVolume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			ITMVoxelVolume <TVoxelPrimary, ITMPlainVoxelArray>* arrayVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMVoxelBlockHash>* hashVolume,
			TFunctor& functor) {


		const int hashEntryCount = hashVolume->index.hashEntryCount;
		ITMHashEntry* hashTable = hashVolume->index.GetIndexData();

		TVoxelPrimary* arrayVoxels = arrayVolume->localVBA.GetVoxelBlocks();
		TVoxelSecondary* hashVoxels = hashVolume->localVBA.GetVoxelBlocks();

		ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo = arrayVolume->index.GetIndexData();
		Vector3i arrayMinVoxels = arrayInfo->offset;
		Vector3i arrayMaxVoxels = arrayInfo->offset + arrayInfo->size;

		volatile bool mismatchFound = false;

		// *** find voxel blocks in hash not spanned by array ***
#ifdef WITH_OPENMP
#pragma omp parallel for default(none), shared(mismatchFound)
#endif
		for (int hash = 0; hash < hashEntryCount; hash++) {
			if (mismatchFound) continue;
			ITMHashEntry& hashEntry = hashTable[hash];
			if (hashEntry.ptr < 0) {
				continue;
			}
			Vector3i blockMinVoxels = hashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i blockMaxVoxels = blockMinVoxels + Vector3i(VOXEL_BLOCK_SIZE);
			if (blockMaxVoxels.x < arrayMinVoxels.x || blockMinVoxels.x > arrayMaxVoxels.x ||
			    blockMaxVoxels.y < arrayMinVoxels.y || blockMinVoxels.y > arrayMaxVoxels.y ||
			    blockMaxVoxels.z < arrayMinVoxels.z || blockMinVoxels.z > arrayMaxVoxels.z) {
				if (isVoxelBlockAltered(hashVoxels[hashEntry.ptr * VOXEL_BLOCK_SIZE3])) {
					mismatchFound = true;
				}
			} else if (blockMaxVoxels.x > arrayMaxVoxels.x || blockMinVoxels.x < arrayMinVoxels.x ||
			           blockMaxVoxels.y > arrayMaxVoxels.y || blockMinVoxels.y < arrayMinVoxels.y ||
			           blockMaxVoxels.z > arrayMaxVoxels.z || blockMinVoxels.z < arrayMinVoxels.z
					) {
				TVoxelSecondary* voxelBlock = hashVoxels[hashEntry.ptr * VOXEL_BLOCK_SIZE3];
				for (int linearIndexInBlock = 0; linearIndexInBlock < VOXEL_BLOCK_SIZE3; linearIndexInBlock++) {
					TVoxelSecondary& voxel = voxelBlock[linearIndexInBlock];
					if (isAltered(voxel)) {
						Vector3i voxelPosition =
								ComputePositionVectorFromLinearIndex_VoxelBlockHash(hashEntry.pos, linearIndexInBlock);
						if (isPointInBounds(voxelPosition, *arrayInfo)) {
							mismatchFound = true;
						}
					}
				}
			}
		}
		if (mismatchFound) return false;

		// *** check all voxels inside array, return false if one's altered but unallocated in the hash or if
		// the functor on it and the corresponding one from the hash-block volume returns false ***

		int voxelCount = arrayVolume->index.GetVoxelBlockSize();
#ifndef WITH_OPENMP
		ITMVoxelBlockHash::IndexCache cache;
#endif

		auto extentHasMismatch = [&](Extent3D extent) {
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(mismatchFound)
#endif
			for (int z = extent.min_z; z < extent.max_z; z++) {
				if(mismatchFound) continue;
				for (int y = extent.min_y; y < extent.max_y; y++) {
					for (int x = extent.min_x; x < extent.max_x; x++) {
						Vector3i position(x, y, z);
						int linearArrayIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(arrayInfo, position);
						int vmIndex;
#ifndef WITH_OPENMP
						int hashVoxelIndex = findVoxel(hashTable, position, vmIndex, cache);
#else
						int hashVoxelIndex = findVoxel(hashTable, position, vmIndex);
#endif
						if(hashVoxelIndex == -1 && isAltered(arrayVoxels[linearArrayIndex])){
							mismatchFound = true;
						}
					}
				}
			}
			return mismatchFound;
		};
		//TODO
		DIEWITHEXCEPTION_REPORTLOCATION("Not finished!");
	}

	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue_Old(
			ITMVoxelVolume <TVoxelPrimary, ITMPlainVoxelArray>* primaryVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMVoxelBlockHash>* secondaryVolume,
			TFunctor& functor) {
// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryVolume->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryVolume->localVBA.GetVoxelBlocks();
		int totalPrimaryVoxelCount = primaryVolume->index.GetVolumeSize().x * primaryVolume->index.GetVolumeSize().y *
		                             primaryVolume->index.GetVolumeSize().z;
		const ITMVoxelBlockHash::IndexData* hashTable = secondaryVolume->index.GetIndexData();
		const ITMPlainVoxelArray::IndexData* pvaIndexData = primaryVolume->index.GetIndexData();
		Vector3i pvaOffset = pvaIndexData->offset;
		Vector3i pvaSize = pvaIndexData->size;
		Vector3i endVoxel = pvaOffset + pvaSize; // open last traversal bound (this voxel doesn't get processed)

		//determine the opposite corners of the 3d extent, [start, end), in voxel hash blocks, to traverse.
		Vector3i startBlockCoords, endBlockCoords;
		pointToVoxelBlockPos(pvaOffset, startBlockCoords);
		pointToVoxelBlockPos(endVoxel - Vector3i(1), endBlockCoords);
		endBlockCoords += Vector3i(1);
		std::unordered_set<int> coveredBlockHashes;
		volatile bool foundMismatch = false;

		//TODO: inner loop traversal should go over *entire VBH block*, not just the portion of it that is spanned by PVA.
		// Case in point: the VBH volume has an allocated hash block that spans somewhat outside the PVA extent.
		//                This VBH volume has a voxel altered within the region of this block that is outside of the PVA
		//                extent. This voxel ideally constitutes a mismatch, but this case is not covered here.
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(foundMismatch)
#endif
		for (int zVoxelBlock = startBlockCoords.z; zVoxelBlock < endBlockCoords.z; zVoxelBlock++) {
			if (foundMismatch) continue;
			int zVoxelStart = std::max(zVoxelBlock * VOXEL_BLOCK_SIZE, pvaOffset.z);
			int zVoxelEnd = std::min((zVoxelBlock + 1) * VOXEL_BLOCK_SIZE, endVoxel.z);
			for (int yVoxelBlock = startBlockCoords.y;
			     yVoxelBlock < endBlockCoords.y && !foundMismatch; yVoxelBlock++) {
				int yVoxelStart = std::max(yVoxelBlock * VOXEL_BLOCK_SIZE, pvaOffset.y);
				int yVoxelEnd = std::min((yVoxelBlock + 1) * VOXEL_BLOCK_SIZE, endVoxel.y);
				for (int xVoxelBlock = startBlockCoords.x;
				     xVoxelBlock < endBlockCoords.x && !foundMismatch; xVoxelBlock++) {
					int xVoxelStart = std::max(xVoxelBlock * VOXEL_BLOCK_SIZE, pvaOffset.x);
					int xVoxelEnd = std::min((xVoxelBlock + 1) * VOXEL_BLOCK_SIZE, endVoxel.x);
					Vector3s voxelBlockCoords(xVoxelBlock, yVoxelBlock, zVoxelBlock);
					int hash;
					bool voxelBlockAllocated = false;
					TVoxelPrimary* localSecondaryVoxelBlock = nullptr;
					// see if the voxel block is allocated at all in VBH-indexed volume
					if (FindHashAtPosition(hash, voxelBlockCoords, hashTable)) {
						voxelBlockAllocated = true;
						localSecondaryVoxelBlock = &(secondaryVoxels[hashTable[hash].ptr *
						                                             (VOXEL_BLOCK_SIZE3)]);
#ifdef WITH_OPENMP
#pragma omp critical
#endif
						{
							coveredBlockHashes.insert(hash);
						};
					}
					// traverse the current voxel block volume to see if PVA-indexed volume's voxels satisfy the boolean
					// functional when matched with corresponding voxels from the VBH-indexed volume.
					// If the block is not allocated in the VBH volume at all but the PVA voxel is modified,
					// short-circuit to "false"
					for (int z = zVoxelStart; z < zVoxelEnd; z++) {
						for (int y = yVoxelStart; y < yVoxelEnd; y++) {
							for (int x = xVoxelStart; x < xVoxelEnd; x++) {
								Vector3i voxelPosition = Vector3i(x, y, z);
								Vector3i voxelPositionSansOffset = voxelPosition - pvaOffset;
								int linearIndex = voxelPositionSansOffset.x + voxelPositionSansOffset.y * pvaSize.x +
								                  voxelPositionSansOffset.z * pvaSize.x * pvaSize.y;
								int locId =
										voxelPosition.x + (voxelPosition.y - voxelBlockCoords.x) * VOXEL_BLOCK_SIZE +
										(voxelPosition.z - voxelBlockCoords.y) * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE -
										voxelBlockCoords.z * VOXEL_BLOCK_SIZE3;
								TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
								if (voxelBlockAllocated) {
									TVoxelSecondary& secondaryVoxel = localSecondaryVoxelBlock[locId];
									if (!(functor(primaryVoxel, secondaryVoxel))) {
										foundMismatch = true;
										break;
									}
								} else {
									if (isAltered(primaryVoxel)) {
										foundMismatch = true;
										break;
									}
								}
							}
						}
					}
				}
			}
		}
		if (foundMismatch) {
			return false;
		}
		int totalHashEntryCount = secondaryVolume->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(foundMismatch)
#endif
		for (int hash = 0; hash < totalHashEntryCount && !; hash++) {
			if (foundMismatch) continue;
			ITMHashEntry secondaryHashEntry = hashTable[hash];
			if (secondaryHashEntry.ptr < 0 || coveredBlockHashes.find(hash) != coveredBlockHashes.end()) continue;
			//found allocated hash block that wasn't spanned by the volume, check to make sure it wasn't altered
			TVoxelSecondary* secondaryVoxelBlock = &(secondaryVoxels[secondaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			if (isVoxelBlockAltered(secondaryVoxelBlock)) {
				foundMismatch = true;
			}
		}
		return !foundMismatch;
	}


	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location, which ignores unallocated spaces in either scene.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated. Areas where either or both of the scenes don't have allocated voxels are
	 * ignored, even if only one of the volumes does, in fact, have potentially-altered voxels there.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param primaryVolume the primary volume -- indexed using plain voxel array (PVA)
	 * \param secondaryVolume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue_AllocatedOnly(
			ITMVoxelVolume <TVoxelPrimary, ITMPlainVoxelArray>* primaryVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMVoxelBlockHash>* secondaryVolume,
			TFunctor& functor) {
		return DualVoxelTraversal_AllTrue_AllocatedOnly_Generic(primaryVolume, secondaryVolume, functor, []
				(TFunctor& functor, TVoxelPrimary& voxelPrimary, TVoxelSecondary& voxelSecondary,
				 Vector3i& voxelPosition) {
			return functor(voxelPrimary, voxelSecondary);
		});
	}

	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location, which ignores unallocated spaces in either scene.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated. Areas where either or both of the scenes don't have allocated voxels are
	 * ignored, even if only one of the volumes does, in fact, have potentially-altered voxels there.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param primaryVolume the primary volume -- indexed using plain voxel array (PVA)
	 * \param secondaryVolume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels and their mutually shared position by reference as arguments
	 * and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	DualVoxelPositionTraversal_AllTrue_AllocatedOnly(
			ITMVoxelVolume <TVoxelPrimary, ITMPlainVoxelArray>* primaryVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMVoxelBlockHash>* secondaryVolume,
			TFunctor& functor) {
		return DualVoxelTraversal_AllTrue_AllocatedOnly_Generic(primaryVolume, secondaryVolume, functor, []
				(TFunctor& functor, TVoxelPrimary& voxelPrimary, TVoxelSecondary& voxelSecondary,
				 Vector3i& voxelPosition) {
			return functor(voxelPrimary, voxelSecondary, voxelPosition);
		});
	}


};


template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMVoxelBlockHash, ITMPlainVoxelArray, MEMORYDEVICE_CPU> {
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
			ITMVoxelVolume <TVoxelPrimary, ITMVoxelBlockHash>* primaryVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMPlainVoxelArray>* secondaryVolume,
			TFunctor& functor) {
		ITMFlipArgumentBooleanFunctor<TVoxelPrimary, TVoxelSecondary, TFunctor> flipFunctor(functor);
		return ITMDualSceneTraversalEngine<TVoxelSecondary, TVoxelPrimary, ITMPlainVoxelArray, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::
		DualVoxelTraversal_AllTrue(secondaryVolume, primaryVolume, flipFunctor);

	}


	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue_AllocatedOnly(
			ITMVoxelVolume <TVoxelPrimary, ITMVoxelBlockHash>* primaryVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMPlainVoxelArray>* secondaryVolume,
			TFunctor& functor) {
		ITMFlipArgumentBooleanFunctor<TVoxelPrimary, TVoxelSecondary, TFunctor> flipFunctor(functor);
		return ITMDualSceneTraversalEngine<TVoxelSecondary, TVoxelPrimary, ITMPlainVoxelArray, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::
		DualVoxelTraversal_AllTrue_AllocatedOnly(secondaryVolume, primaryVolume, flipFunctor);
	}
};
} // namespace ITMLib