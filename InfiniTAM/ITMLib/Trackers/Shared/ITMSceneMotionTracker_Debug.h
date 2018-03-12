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

//_DEBUG
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
		info.liveSdf = InterpolateTrilinearly(liveVoxelData, liveHashTable, TO_FLOAT3(neighborPosition) + voxel.warp_t,
		                                      liveCache, info.liveFound);
		iNeighbor++;
	}
}

//======================================================================================================================
//====================================== DEBUG ROUTINES FOR LEVEL SET TERM =============================================
//======================================================================================================================

// This function samples the live scene using warps of canonical voxels in a neighborhood around a particular voxel.
// From the computed values, the gradient (jacobian) is computed, as well as its derivative ("hessian") with respect
// to changing the warp at the current voxel.
template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void _DEBUG_ComputeWarpedJacobianAndHessian(const CONSTPTR(Vector3f*) neighborWarps,
                                                   const CONSTPTR(Vector3i)& canonicalVoxelPosition,
                                                   const CONSTPTR(float)& canonicalSdf,
                                                   const CONSTPTR(Vector3f)& liveSdf_Center_WarpForward,
                                                   const CONSTPTR(float)& liveSdf,//lookup value at center warp_t
                                                   const CONSTPTR(TVoxel)* liveVoxels,
                                                   const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                   THREADPTR(TCache)& liveCache,
                                                   THREADPTR(Vector3f)& warpedSdfJacobian, //out
                                                   THREADPTR(Matrix3f)& warpedSdfHessian //out
) {
	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	Vector3f liveSdf_at_current_voxel(liveSdf);
	Vector3f centerPosition = canonicalVoxelPosition.toFloat();
	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === check lookups for voxels forward by 1 in x, y, z of the canonical frame, use their own warps
	Vector3f warpedPositionOfNeighbor_at_x_plus_one = centerPosition + Vector3f(1.f, 0.f, 0.f) + neighborWarps[3];
	Vector3f warpedPositionOfNeighbor_at_y_plus_one = centerPosition + Vector3f(0.f, 1.f, 0.f) + neighborWarps[4];
	Vector3f warpedPositionOfNeighbor_at_z_plus_one = centerPosition + Vector3f(0.f, 0.f, 1.f) + neighborWarps[5];

	Vector3f liveSdf_at_warped_neighbors(
			InterpolateTrilinearly_SetTruncatedToVal(liveVoxels, liveHashTable, liveSdf,
			                                         warpedPositionOfNeighbor_at_x_plus_one, liveCache),
			InterpolateTrilinearly_SetTruncatedToVal(liveVoxels, liveHashTable, liveSdf,
			                                         warpedPositionOfNeighbor_at_y_plus_one, liveCache),
			InterpolateTrilinearly_SetTruncatedToVal(liveVoxels, liveHashTable, liveSdf,
			                                         warpedPositionOfNeighbor_at_z_plus_one, liveCache)
	);
	//=========== WARPED SDF JACOBIAN ==================================================================================
	// derivatives of the type [d phi (warp) / dx]
	// d pheta_n(warp) / d x
	// d pheta_n(warp) / d y
	// d pheta_n(warp) / d z
	warpedSdfJacobian = liveSdf_at_warped_neighbors - liveSdf_at_current_voxel;

	//=========== COMPUTE 2ND PARTIAL DERIVATIVES IN SAME DIRECTION ====================================================
	Vector3f deltaWarpChanged = liveSdf_at_warped_neighbors - liveSdf_Center_WarpForward;
	//we keep the neighbors warps fixed, but we see how the gradient (jacobian above) changes if we change current
	//voxel's warp
	Vector3f sdfDerivatives_ux_vy_wz =
			deltaWarpChanged - warpedSdfJacobian;

	//=== corner-voxel auxiliary first partial first derivatives for later 2nd derivative approximations
	// vx - v //how does a change of warp in v affect the change [in calonical neighborhood] in x
	// uy - u
	// uz - u
	Vector3f changedWarpFirstDerivatives1 = liveSdf_at_warped_neighbors - Vector3f(liveSdf_Center_WarpForward.v,
	                                                                               liveSdf_Center_WarpForward.u,
	                                                                               liveSdf_Center_WarpForward.u);
	// wx - w
	// wy - w
	// vz - v
	Vector3f changedWarpFirstDerivatives2 = liveSdf_at_warped_neighbors - Vector3f(liveSdf_Center_WarpForward.w,
	                                                                               liveSdf_Center_WarpForward.w,
	                                                                               liveSdf_Center_WarpForward.u);

	//===Compute the 2nd partial derivatives for different direction sequences
	Vector3f sdfDerivatives_vx_uy_uz = changedWarpFirstDerivatives1 - warpedSdfJacobian;
	Vector3f sdfDerivatives_wx_wy_vz = changedWarpFirstDerivatives2 - warpedSdfJacobian;

	// "Hessian":
	// ux uy uz
	// vx vy vz
	// wx wy wz, column-major:
	float vals[] = {sdfDerivatives_ux_vy_wz[0], sdfDerivatives_vx_uy_uz[0], sdfDerivatives_wx_wy_vz[0],//col 1
	                sdfDerivatives_vx_uy_uz[1], sdfDerivatives_ux_vy_wz[1], sdfDerivatives_wx_wy_vz[1],//col 2
	                sdfDerivatives_vx_uy_uz[2], sdfDerivatives_wx_wy_vz[2], sdfDerivatives_ux_vy_wz[2]};

	warpedSdfHessian.setValues(vals);
};


//======================================================================================================================
//====================================== DEBUG ROUTINES FOR DATA TERM ==================================================
//======================================================================================================================
// this set of functions simply computes the jacobian of the live scene at an interpolated warped position, without
// computing the hessian. The jacobian vector defines the direction of data term's contribution to the warp updates.
template<typename TVoxelLive, typename TCache>
_CPU_AND_GPU_CODE_
inline void _DEBUG_ComputeLiveSdf_Center_WarpForward(
		const CONSTPTR(Vector3i)& voxelPosition,
		const CONSTPTR(Vector3f)& voxelWarp,
		const CONSTPTR(float)& voxelSdf,
		const CONSTPTR(TVoxelLive)* liveVoxels,       //| ===============  |
		const CONSTPTR(ITMHashEntry)* liveHashTable,  //| live scene data  |
		THREADPTR(TCache)& liveCache,                 //| ===============  |
		THREADPTR(Vector3f)& liveSdf_Center_WarpForward
) {
	//position projected with the current warp
	Vector3f warpedPosition = voxelPosition.toFloat() + voxelWarp;
	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === increment the warp by 1 in each direction and use them to check what interpolated values from live frame map there
	float live_at_u_plus_one = InterpolateTrilinearly_SetTruncatedToVal(
			liveVoxels, liveHashTable, voxelSdf, warpedPosition + Vector3f(1.f, 0.f, 0.f), liveCache);
	float live_at_v_plus_one = InterpolateTrilinearly_SetTruncatedToVal(
			liveVoxels, liveHashTable, voxelSdf, warpedPosition + Vector3f(0.f, 1.f, 0.f), liveCache);
	float live_at_w_plus_one = InterpolateTrilinearly_SetTruncatedToVal(
			liveVoxels, liveHashTable, voxelSdf, warpedPosition + Vector3f(0.f, 0.f, 1.f), liveCache);

	liveSdf_Center_WarpForward =
			Vector3f(live_at_u_plus_one, live_at_v_plus_one, live_at_w_plus_one);
};

//without color
template<typename TVoxelLive, typename TCache>
_CPU_AND_GPU_CODE_
inline void _DEBUG_ComputePerPointLiveJacobian(const CONSTPTR(Vector3i)& voxelPosition,
                                               const CONSTPTR(Vector3f)& voxelWarp,
                                               const CONSTPTR(float)& voxelSdf,
                                               const CONSTPTR(
		                                               TVoxelLive)* liveVoxels,       //| ===============  |
                                               const CONSTPTR(
		                                               ITMHashEntry)* liveHashTable,  //| live scene data  |
                                               THREADPTR(
		                                               TCache)& liveCache,                 //| ===============  |
                                               const CONSTPTR(float) liveSdf,
                                               THREADPTR(Vector3f)& liveSdfJacobian,           //out
                                               THREADPTR(Vector3f)& liveSdf_Center_WarpForward//out
) {
	Vector3f liveSdf_Center_WarpCenter(liveSdf);
	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================

	//=========== COMPUTE JACOBIAN =====================================================================================
	liveSdfJacobian = liveSdf_Center_WarpForward - liveSdf_Center_WarpCenter;
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
	std::cout << "Jacobian of live SDF at current warp: " << cyan << liveSdfJacobian <<  reset << std::endl;
}

//_DEBUG printing routine
_CPU_AND_GPU_CODE_
inline void _DEBUG_PrintLevelSetTermStuff(const CONSTPTR(Vector3f)& liveSdfJacobian,
                                          const CONSTPTR(Vector3f)& liveSdf_Center_WarpForward,
                                          const CONSTPTR(Vector3f)& warpedSdfJacobian,
                                          const CONSTPTR(Matrix3f)& warpedSdfHessian) {
	const std::string yellow("\033[0;33m");
	const std::string red("\033[0;31m");
	const std::string cyan("\033[0;36m");
	const std::string green("\033[0;32m");
	const std::string reset("\033[0m");
	std::cout << std::endl;
	std::cout << "Warped SDF Jacobian [Difference from neighbor's lookup values from live SDF]: " << green << warpedSdfJacobian << reset << std::endl;
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
	const std::string yellow("\033[0;33m");
	const std::string red("\033[0;31m");
	const std::string cyan("\033[0;36m");
	const std::string green("\033[0;32m");
	const std::string reset("\033[0m");
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