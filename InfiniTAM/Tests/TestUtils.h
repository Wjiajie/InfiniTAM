//  ================================================================
//  Created by Gregory Kramida on 10/24/17.
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
#pragma once

#include "../ITMLib/Objects/Scene/ITMRepresentationAccess.h"

inline void allocateHashEntry_CPU(Vector3i& voxelLocation,
                                  ITMHashEntry* hashTable,
                                  int& lastFreeVoxelBlockId,
                                  int& lastFreeExcessListId,
                                  int* voxelAllocationList,
                                  int* excessAllocationList) {
	int vbaIdx, exlIdx;

	Vector3s blockPos = voxelLocation.toShortFloor();
	int hashIdx = hashIndex(blockPos);
	//check if hash table contains entry
	bool isFound = false;

	ITMHashEntry hashEntry = hashTable[hashIdx];

	if (!(IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)) {
		bool isExcess = false;
		if (hashEntry.ptr >= -1) //search excess list only if there is no room in ordered part
		{
			while (hashEntry.offset >= 1) {
				hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
				hashEntry = hashTable[hashIdx];

				if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1) {
					//entry has been streamed out but is visible or in memory and visible
					isFound = true;
					break;
				}
			}
			isExcess = true;
		}

		if (!isFound) //still not found
		{
			//allocate
			if (isExcess) {
				//needs allocation in the excess list
				vbaIdx = lastFreeVoxelBlockId;
				lastFreeVoxelBlockId--;
				exlIdx = lastFreeExcessListId;
				lastFreeExcessListId--;

				if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
				{
					Vector4s pt_block_all(blockPos.x, blockPos.y, blockPos.z, 1);
					ITMHashEntry hashEntry;
					hashEntry.pos.x = pt_block_all.x;
					hashEntry.pos.y = pt_block_all.y;
					hashEntry.pos.z = pt_block_all.z;
					hashEntry.ptr = voxelAllocationList[vbaIdx];
					hashEntry.offset = 0;

					int exlOffset = excessAllocationList[exlIdx];

					hashTable[hashIdx].offset = exlOffset + 1; //connect to child

					hashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list
				} else {
					// No need to mark the entry as not visible since buildHashAllocAndVisibleTypePP did not mark it.
					// Restore previous value to avoid leaks.
					lastFreeVoxelBlockId++;
					lastFreeExcessListId++;
				}
			} else {
				//needs allocation, fits in the ordered list
				vbaIdx = lastFreeVoxelBlockId;
				lastFreeVoxelBlockId--;
				if (vbaIdx >= 0) //there is room in the voxel block array
				{
					Vector4s pt_block_all(blockPos.x, blockPos.y, blockPos.z, 1);

					ITMHashEntry hashEntry;
					hashEntry.pos.x = pt_block_all.x;
					hashEntry.pos.y = pt_block_all.y;
					hashEntry.pos.z = pt_block_all.z;
					hashEntry.ptr = voxelAllocationList[vbaIdx];
					hashEntry.offset = 0;

					hashTable[hashIdx] = hashEntry;
				} else {
					// Restore previous value to avoid leaks.
					lastFreeVoxelBlockId++;
				}
			}
		}
	}
}


//TODO: assume we do have color for now. Make provisions later to account for TVoxel with no color. -Greg (GitHub:Algomorph)
/**
 * \brief find neighboring voxels and write their pointers to array
 * This is different from findPointNeighbors because it returns 1.0 for truncated voxels and voxels beyond the scene boundary.
 * \tparam TVoxel the voxel type
 * \param p [out] pointer to memory where to store 8 values Vector3i for all the neighbors
 * \param sdf [out] pointer to memory where to store 8 SDF values for all the neighbors
 * \param blockLocation the actual block location
 * \param localVBA the voxel grid
 * \param hashTable the hash table index of the voxel entries
 */
_CPU_AND_GPU_CODE_
inline void findOrAllocatePointNeighbors(THREADPTR(Vector3f)* voxelPositions, THREADPTR(int)* voxelIndexes,
                                         const CONSTPTR(Vector3i)& blockLocation,
                                         ITMHashEntry* hashTable,
                                         int& lastFreeVoxelBlockId, int& lastFreeExcessListId,
                                         int* voxelAllocationList, int* excessAllocationList) {
	int foundPoint;
	Vector3i location;

	ITMLib::ITMVoxelBlockHash::IndexCache cache;

#define PROCESS_VOXEL(localPos, index) \
    location = blockLocation + (localPos);\
    voxelPositions[index] = (location).toFloat();\
    voxelIndexes[index] = findVoxel(hashTable, location, foundPoint, cache);\
	if (!foundPoint) {\
		allocateHashEntry_CPU(location, hashTable, lastFreeVoxelBlockId,\
		                  lastFreeExcessListId, voxelAllocationList, excessAllocationList);\
		voxelIndexes[index] = findVoxel(hashTable, location, foundPoint, cache);\
	}

	PROCESS_VOXEL(Vector3i(0, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 1);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 2);
	PROCESS_VOXEL(Vector3i(0, 1, 1), 3);
	PROCESS_VOXEL(Vector3i(1, 0, 0), 4);
	PROCESS_VOXEL(Vector3i(1, 0, 1), 5);
	PROCESS_VOXEL(Vector3i(1, 1, 0), 6);
	PROCESS_VOXEL(Vector3i(1, 1, 1), 7);
#undef PROCESS_VOXEL

}


/**
 * \brief Compute trilienar coefficients for the given target point
 * \param targetPoint [in] an arbitrary point located inside a regular voxel grid
 * \param point000 [in] location of the voxel that coincides wit the truncated coordinates of the target point
 * \param point111 [in] location of the voxel with an increment of one (voxel) in each direction from point000
 * \param coefficients [out] the resulting trilinear coefficients
 *
 */
_CPU_AND_GPU_CODE_
inline void computeTrilinearCoefficients(const CONSTPTR(Vector3f)& ratios,
                                         THREADPTR(float) * trilinearCoefficients) {
	Vector3f invRatios = Vector3f(1.f) - ratios;
	float ratio_00 = invRatios.y * invRatios.z;
	float ratio_01 = invRatios.y * ratios.z;
	float ratio_10 = ratios.y * invRatios.z;
	float ratio_11 = ratios.y * ratios.z;
	trilinearCoefficients[0] = ratio_00 * invRatios.x;
	trilinearCoefficients[1] = ratio_01 * invRatios.x;
	trilinearCoefficients[2] = ratio_10 * invRatios.x;
	trilinearCoefficients[3] = ratio_11 * invRatios.x;
	trilinearCoefficients[4] = ratio_00 * ratios.x;
	trilinearCoefficients[5] = ratio_01 * ratios.x;
	trilinearCoefficients[6] = ratio_10 * ratios.x;
	trilinearCoefficients[7] = ratio_11 * ratios.x;
}

/**
 * \brief Distribute the value at an arbitrary target point to the nearest 8 voxels using a trilinear interpolation scheme
 * \tparam TVoxel type of voxel
 * \tparam TIndex type of index used
 * \param voxel a pseudo-voxel that contains all the values at the arbitrary target point (usually, it would come from a different voxel grid)
 * \param point the target point position in 3D space (in voxel units)
 * \param localVBA the voxel grid
 * \param hashTable the hash table index of the voxel entries
 */
template<typename TVoxel>
_CPU_AND_GPU_CODE_
inline void distributeTrilinearly(const CONSTPTR(TVoxel)& voxel,
                                  const CONSTPTR(Vector3f)& point,
                                  THREADPTR(TVoxel)* voxelData,
                                  ITMHashEntry* hashTable,
                                  int& lastFreeVoxelBlockId,
                                  int& lastFreeExcessListId,
                                  int* voxelAllocationList,
                                  int* excessAllocationList) {
	const int neighborCount = 8;

	//TODO: provide alternatives for voxel types that do not support all of these values.
	Vector3f colorF = voxel.clr.toFloat();
	float sdf = voxel.sdf;
	uchar w_depth = voxel.w_depth;
	uchar w_color = voxel.w_color;
	float confidence = voxel.confidence;

	// determine neighborhood starting coordinate, i.e. point000 for trilinear weight scheme
	Vector3i neighbor0BlockLocation = point.toIntRound() - Vector3i(1);
	Vector3f neighborPositions[neighborCount];
	int neighborIndices[neighborCount];

	// find the actual neighbors and compute their 3D coordinates in Vector3f format
	findOrAllocatePointNeighbors(neighborPositions, neighborIndices, neighbor0BlockLocation, hashTable,
	                             lastFreeVoxelBlockId, lastFreeExcessListId, voxelAllocationList,excessAllocationList);

	//compute the trilinear coefficients for each neighbor
	float trilinearCoefficients[neighborCount];
	Vector3f ratios;
	Vector3i baseVoxelPos;
	TO_INT_FLOOR3(baseVoxelPos,ratios,point);
	computeTrilinearCoefficients(ratios, trilinearCoefficients);

	//apply contribution of the sdf and color values to each neighbor weighed by the neighbor's coefficient
	for (int iVoxel = 0; iVoxel < neighborCount; iVoxel++) {
		TVoxel* neighbor = &voxelData[neighborIndices[iVoxel]];
		float weight = trilinearCoefficients[iVoxel];
		neighbor->sdf += weight * sdf;
		neighbor->clr += (weight * colorF).toIntRound().toUChar();
		neighbor->w_depth += weight * w_depth;
		neighbor->w_color += weight * w_color;
		neighbor->confidence += weight * confidence;
	}


}