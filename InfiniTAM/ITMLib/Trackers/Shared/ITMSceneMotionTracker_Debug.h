//  ================================================================
//  Created by Gregory Kramida on 2/20/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../Utils/ITMHashBlockProperties.h"
#include "../../Utils/ITMNeighborVoxelIterationInfo.h"
#include "../../Utils/ITMPrintHelpers.h"
#include "../../Objects/Scene/ITMTrilinearInterpolation.h"

using namespace ITMLib;

//======================================================================================================================
//=========================================== DEBUG ROUTINES FOR SAVING INFORMATION DURING OPTIMIZATION ================
//======================================================================================================================
//DEBUG
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel
ReadVoxelAndLinearIndex(const CONSTPTR(TVoxel)* voxelData,
                        const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* voxelIndex,
                        const THREADPTR(Vector3i)& point, THREADPTR(int)& vmIndex,
                        THREADPTR(ITMLib::ITMVoxelBlockHash::IndexCache)& cache, THREADPTR(int)& linearIdx) {
	Vector3i blockPos;
	linearIdx = pointToVoxelBlockPos(point, blockPos);

	if IS_EQUAL3(blockPos, cache.blockPos) {
		return voxelData[cache.blockPtr + linearIdx];
	}

	int hashIdx = hashIndex(blockPos);

	while (true) {
		ITMHashEntry hashEntry = voxelIndex[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0) {
			cache.blockPos = blockPos;
			cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
			vmIndex = hashIdx + 1; // add 1 to support legacy true / false operations for isFound
			return voxelData[cache.blockPtr + linearIdx];
		}

		if (hashEntry.offset < 1) break;
		hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
	}

	vmIndex = false;
	linearIdx = 0;
	return TVoxel();
}

_CPU_AND_GPU_CODE_
template<typename TVoxelCanonical, typename TVoxelLive, typename TLiveCache>
inline void FindHighlightNeighborInfo(std::array<ITMLib::ITMNeighborVoxelIterationInfo, 9>& neighbors,
                                      const CONSTPTR(Vector3i)& highlightPosition,
                                      const CONSTPTR(int)& highlightHash,
                                      const CONSTPTR(TVoxelCanonical)* canonicalVoxelData,
                                      const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                      const CONSTPTR(TVoxelLive)* liveVoxelData,
                                      const CONSTPTR(ITMHashEntry)* liveHashTable,
                                      THREADPTR(TLiveCache)& liveCache) {
	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	Vector3i locations[9] = {Vector3i(-1, 0, 0), Vector3i(0, -1, 0), Vector3i(0, 0, -1),
	                         Vector3i(1, 0, 0), Vector3i(0, 1, 0), Vector3i(0, 0, 1),
	                         Vector3i(1, 1, 0), Vector3i(0, 1, 1), Vector3i(1, 0, 1)};
	int vmIndex, localId = 0;
	vmIndex = highlightHash + 1;
	ITMLib::ITMVoxelBlockHash::IndexCache cache;
	int iNeighbor = 0;
	for (auto location : locations) {
		ITMLib::ITMNeighborVoxelIterationInfo& info = neighbors[iNeighbor];
		Vector3i neighborPosition = highlightPosition + (location);
		TVoxelCanonical voxel = ReadVoxelAndLinearIndex(canonicalVoxelData, canonicalHashTable, neighborPosition,
		                                                vmIndex, cache,
		                                                localId);
		if (vmIndex != 0) {
			info.unknown = voxel.flags == ITMLib::VOXEL_TRUNCATED;
			info.hash = vmIndex - 1;
		} else {
			info.notAllocated = true;
			info.hash = 0;
			vmIndex = highlightHash + 1;//reset
		}
		info.localId = localId;
		info.warp = voxel.warp_t;
		info.warpUpdate = voxel.warp_t_update;
		info.sdf = voxel.sdf;
		info.liveSdf = InterpolateTrilinearly_SetUnknownToVal_StruckChecks(liveVoxelData, liveHashTable, info.sdf,
		                                                                   TO_FLOAT3(neighborPosition) + voxel.warp_t,
		                                                                   liveCache, info.struckNarrowBand,
		                                                                   info.struckKnownVoxels);
		iNeighbor++;
	}
}

//======================================================================================================================
//====================================== DEBUG PRINTING ROUTINES =======================================================
//======================================================================================================================

//_DEBUG printing routine
_CPU_AND_GPU_CODE_
inline void _DEBUG_PrintDataTermStuff(const CONSTPTR(Vector3f)& liveSdfJacobian) {
	const std::string cyan("\033[0;36m");
	const std::string reset("\033[0m");
	std::cout << std::endl;
	std::cout << "Jacobian of live SDF at current warp: " << cyan << liveSdfJacobian << reset << std::endl;
}

//_DEBUG printing routine
_CPU_AND_GPU_CODE_
inline void _DEBUG_PrintLevelSetTermStuff(const CONSTPTR(Vector3f)& liveSdfJacobian,
                                          const CONSTPTR(Vector3f)& liveSdf_Center_WarpForward,
                                          const CONSTPTR(Vector3f)& warpedSdfJacobian,
                                          const CONSTPTR(Matrix3f)& warpedSdfHessian) {
	std::cout << std::endl;
	std::cout << "Warped SDF Jacobian [Difference from neighbor's lookup values from live SDF]: " << green
	          << warpedSdfJacobian << reset << std::endl;
	std::cout << "Change in warped SDF Jacobian when warp changes (by one): " << std::endl
	          << green << warpedSdfHessian << reset << std::endl;
}


//_DEBUG printing routine
_CPU_AND_GPU_CODE_
inline void _DEBUG_PrintKillingTermStuff(const CONSTPTR(Vector3f*) neighborWarps,
                                         const CONSTPTR(bool*) neighborAllocated,
                                         const CONSTPTR(bool*) neighborTruncated,
                                         THREADPTR(Matrix3f)& jacobian, //in
                                         THREADPTR(Matrix3f)* hessian //in, x3
) {

	const int neighborhoodSize = 9;
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	Vector3i neighborPositions[] = {Vector3i(-1, 0, 0), Vector3i(0, -1, 0), Vector3i(0, 0, -1), Vector3i(1, 0, 0),
	                                Vector3i(0, 1, 0), Vector3i(0, 0, 1), Vector3i(1, 1, 0), Vector3i(0, 1, 1),
	                                Vector3i(1, 0, 1),};

	std::cout << green;
	std::cout << "Neighbors' warps: " << std::endl;
	for (int iNeightbor = 0; iNeightbor < neighborhoodSize; iNeightbor++) {
		std::cout << reset << neighborPositions[iNeightbor] << " (Neighbor " << iNeightbor << ")" << ": " << green
		          << neighborWarps[iNeightbor] << ", " << std::endl;
	}

	std::cout << std::endl << reset << "Unallocated neighbors: ";
	for (int iNeightbor = 0; iNeightbor < neighborhoodSize; iNeightbor++) {
		if (!neighborAllocated[iNeightbor]) {
			std::cout << iNeightbor << ", ";
		}
	}
	std::cout << std::endl;
	std::cout << "Truncated neighbors: ";
	for (int iNeightbor = 0; iNeightbor < neighborhoodSize; iNeightbor++) {
		if (neighborTruncated[iNeightbor]) {
			std::cout << iNeightbor << ", ";
		}
	}
	std::cout << std::endl;
	std::cout << std::endl << yellow;
	std::cout << "Jacobian: " << std::endl << jacobian << std::endl << cyan;
	std::cout << "Hessian: " << std::endl << hessian[0] << hessian[1] << hessian[2] << reset << std::endl;
};