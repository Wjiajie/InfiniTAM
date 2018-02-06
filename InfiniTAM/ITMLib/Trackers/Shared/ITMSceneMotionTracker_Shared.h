//  ================================================================
//  Created by Gregory Kramida on 10/19/17.
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

#include "../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../Objects/Scene/ITMTrilinearInterpolation.h"
#include "../../Utils/ITMHashBlockProperties.h"


_CPU_AND_GPU_CODE_
inline float squareDistance(const CONSTPTR(Vector3f)& vec1,
                            const CONSTPTR(Vector3f)& vec2) {
	Vector3f difference = vec1 - vec2;
	return dot(difference, difference);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeLookupJacobianAndHessian(const CONSTPTR(Vector3f*) warp_tNeighbors,
                                            const CONSTPTR(Vector3i)& originalPosition,
                                            const CONSTPTR(float)& warpedSdf,//lookup value at center warp_t

                                            const CONSTPTR(TVoxelLive)* liveVoxels,
                                            const CONSTPTR(ITMHashEntry)* liveHashTable,
                                            THREADPTR(TCache)& liveCache,

                                            THREADPTR(Vector3f)& lookupSdfJacobian,
                                            THREADPTR(Matrix3f)& lookupSdfHessian,
                                            bool printResult) {
	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)

	Vector3f centerPos = originalPosition.toFloat();

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	Vector3f lookupSdf_Center_WarpCenter(warpedSdf);

	// === decrease u, v, w by 1 in each direction
	Vector3f lookupPos_xCenter_WarpForward = centerPos + Vector3f(1.f, 0.f, 0.f);
	Vector3f lookupPos_yCenter_WarpForward = centerPos + Vector3f(0.f, 1.f, 0.f);
	Vector3f lookupPos_zCenter_WarpForward = centerPos + Vector3f(0.f, 0.f, 1.f);
	Vector3f lookupSdf_Center_WarpForward(
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_xCenter_WarpForward, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_yCenter_WarpForward, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_zCenter_WarpForward, liveCache));

	// === check lookups for voxels forward by 1 in x, y, z of the canonical frame,
	// decrease u, v, w by 1 in each direction
	// (vectors like Vector3f(1.f, 0.f, 0.f) implicitly subtracted from voxel position to get centerPos)
	Vector3f lookupPos_xForward_WarpBack = centerPos + warp_tNeighbors[3];
	Vector3f lookupPos_yForward_WarpBack = centerPos + warp_tNeighbors[4];
	Vector3f lookupPos_zForward_WarpBack = centerPos + warp_tNeighbors[5];
	Vector3f lookupSdf_Forward_WarpBack(
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_xForward_WarpBack, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_yForward_WarpBack, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_zForward_WarpBack, liveCache)
	);

	// === check lookups for voxels forward by 1 in x, y, z of the canonical frame, use their own warps
	Vector3f lookupPos_xForward_WarpCenter = lookupPos_xForward_WarpBack + Vector3f(1.f, 0.f, 0.f);
	Vector3f lookupPos_yForward_WarpCenter = lookupPos_yForward_WarpBack + Vector3f(0.f, 1.f, 0.f);
	Vector3f lookupPos_zForward_WarpCenter = lookupPos_zForward_WarpBack + Vector3f(0.f, 0.f, 1.f);
	Vector3f lookupSdf_Forward_WarpCenter(
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_xForward_WarpBack, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_yForward_WarpBack, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_zForward_WarpBack, liveCache)
	);

	// === check lookups for voxels forward by 1 in x, y, z of the canonical frame, use their own warps
	Vector3f lookupPos_xForward_WarpForward = lookupPos_xForward_WarpCenter + Vector3f(1.f, 0.f, 0.f);
	Vector3f lookupPos_yForward_WarpForward = lookupPos_yForward_WarpCenter + Vector3f(0.f, 1.f, 0.f);
	Vector3f lookupPos_zForward_WarpForward = lookupPos_zForward_WarpCenter + Vector3f(0.f, 0.f, 1.f);
	Vector3f lookupSdf_Forward_WarpForward(
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_xForward_WarpCenter, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_yForward_WarpCenter, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_zForward_WarpCenter, liveCache)
	);

	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	// === u-y, u-z, v-x, v-z, w-x, and w-y plane forward corners for 2nd derivatives
	Vector3f cornerSdf_uy = centerPos + warp_tNeighbors[4] + Vector3f(1.f, 1.f, 0.f);
	Vector3f cornerSdf_uz = centerPos + warp_tNeighbors[5] + Vector3f(1.f, 0.f, 1.f);
	Vector3f cornerSdf_vx = centerPos + warp_tNeighbors[3] + Vector3f(1.f, 1.f, 0.f);
	Vector3f cornerSdf_vz = centerPos + warp_tNeighbors[5] + Vector3f(1.f, 0.f, 1.f);
	Vector3f cornerSdf_wx = centerPos + warp_tNeighbors[3] + Vector3f(1.f, 0.f, 1.f);
	Vector3f cornerSdf_wy = centerPos + warp_tNeighbors[4] + Vector3f(0.f, 1.f, 1.f);
	Vector3f lookupSdfCorners1(
			interpolateTrilinearly(liveVoxels, liveHashTable, cornerSdf_uy, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, cornerSdf_vx, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, cornerSdf_wx, liveCache));
	Vector3f lookupSdfCorners2(
			interpolateTrilinearly(liveVoxels, liveHashTable, cornerSdf_uz, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, cornerSdf_vz, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, cornerSdf_wy, liveCache));

	//=========== COMPUTE JACOBIAN =====================================================================================
	//simple difference between live lookups neighboring by 1 unit in canonical space

	lookupSdfJacobian = lookupSdf_Forward_WarpCenter - lookupSdf_Center_WarpCenter;

	//=========== COMPUTE 2ND PARTIAL DERIVATIVES IN SAME DIRECTION ====================================================
	Vector3f sdfDerivatives_ux_vy_wz =
			(lookupSdf_Forward_WarpForward - lookupSdf_Center_WarpForward) - lookupSdfJacobian;

	//=== corner-voxel auxiliary first partial derivatives for later 2nd derivative approximations
	// uy - u
	// vx - v
	// wx - w
	Vector3f cornerSdfFirstDerivatives1 = lookupSdfCorners1 - lookupSdf_Center_WarpForward;
	// uz - u
	// vz - v
	// wy - w
	Vector3f cornerSdfFirstDerivatives2 = lookupSdfCorners2 - lookupSdf_Center_WarpForward;

	//===Compute the 2nd partial derivatives for different direction sequences
	Vector3f sdfDerivatives_uy_vx_wx = Vector3f(cornerSdfFirstDerivatives1.x - lookupSdfJacobian.y,
	                                            cornerSdfFirstDerivatives1.y - lookupSdfJacobian.x,
	                                            cornerSdfFirstDerivatives1.z - lookupSdfJacobian.x);
	Vector3f sdfDerivatives_uz_vz_wy = Vector3f(cornerSdfFirstDerivatives2.x - lookupSdfJacobian.z,
	                                            cornerSdfFirstDerivatives2.y - lookupSdfJacobian.z,
	                                            cornerSdfFirstDerivatives2.z - lookupSdfJacobian.y);

	//Hessian:
	// ux uy uz
	// vx vy vz
 	// wx wy wz, column-major:
	float vals[] = {sdfDerivatives_ux_vy_wz.x, sdfDerivatives_uy_vx_wx.y, sdfDerivatives_uy_vx_wx.z,//col 1
	                sdfDerivatives_uy_vx_wx.x, sdfDerivatives_ux_vy_wz.y, sdfDerivatives_uz_vz_wy.z,//col 2
	                sdfDerivatives_uz_vz_wy.x, sdfDerivatives_uz_vz_wy.y, sdfDerivatives_ux_vy_wz.z};
	lookupSdfHessian.setValues(vals);

	if (printResult) {
		std::cout << "Back-projected SDF Jacobian: " << lookupSdfJacobian << std::endl << std::endl;
	}


};

// =====================================================================================================================
// ================================================= WARPED LIVE JACOBIAN ==============================================
// =====================================================================================================================
// this set of functions simply computes the jacobian of the live frame at an interpolated warped position, without
// computing the hessian. The jacobian vector defines the direction of data term's contribution to the warp updates.

// with color
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobian(const CONSTPTR(Vector3i)& originalPosition,
                                              const CONSTPTR(Vector3f)& originalWarp_t,
                                              const CONSTPTR(TVoxelCanonical)* canonicalVoxels,
                                              const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                              THREADPTR(TCache)& canonicalCache,
                                              const CONSTPTR(TVoxelLive)* liveVoxels,
                                              const CONSTPTR(ITMHashEntry)* liveHashTable,
                                              THREADPTR(TCache)& liveCache,
                                              const CONSTPTR(float)& warpedSdf,
                                              const CONSTPTR(Vector3f)& warpedColor,
                                              THREADPTR(Vector3f)& sdfJacobian,
                                              THREADPTR(Vector3f)& colorJacobian) {
	//position projected with the current warp
	Vector3f currentProjectedPosition = originalPosition.toFloat() + originalWarp_t;

	//=== shifted warped sdf locations, shift vector, alternative projected position
	//warpedSdf = interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition, liveCache, warpedColor);
	Vector3f warpedColorForward[3];

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === forward by 1 in each direction
	Vector3f warpedSdfForward(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 0), liveCache,
			                       warpedColorForward[0]),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 0), liveCache,
			                       warpedColorForward[1]),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, 1), liveCache,
			                       warpedColorForward[2]));
	//=========== COMPUTE JACOBIAN =====================================================================================
	sdfJacobian = warpedSdfForward - Vector3f(warpedSdf);
	colorJacobian = Vector3f(
			squareDistance(warpedColorForward[0], warpedColor),
			squareDistance(warpedColorForward[1], warpedColor),
			squareDistance(warpedColorForward[2], warpedColor));
};

//without color
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobian(const CONSTPTR(Vector3i)& originalPosition,
                                              const CONSTPTR(Vector3f)& originalWarp_t,
                                              const CONSTPTR(TVoxelCanonical)* canonicalVoxels,
                                              const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                              THREADPTR(TCache)& canonicalCache,
                                              const CONSTPTR(TVoxelLive)* liveVoxels,
                                              const CONSTPTR(ITMHashEntry)* liveHashTable,
                                              THREADPTR(TCache)& liveCache,
                                              const CONSTPTR(float) warpedSdf,
                                              THREADPTR(Vector3f)& liveSdfJacobian,
                                              const CONSTPTR(bool) printResult = false) {
	//position projected with the current warp
	Vector3f currentProjectedPosition = originalPosition.toFloat() + originalWarp_t;

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === forward by 1 in each direction
	Vector3f warpedSdfForward(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1.f, 0.f, 0.f),
			                       liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0.f, 1.f, 0.f),
			                       liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0.f, 0.f, 1.f),
			                       liveCache));
	//=========== COMPUTE JACOBIAN =====================================================================================
	liveSdfJacobian = warpedSdfForward - Vector3f(warpedSdf);

	//=========== PRINT RESULT IF REQUESTED ============================================================================
	if (printResult) {
		std::cout << "Live SDF Forward at warp: " << warpedSdfForward << std::endl;
		std::cout << "Live SDF Jacobian at warp: " << liveSdfJacobian << std::endl;
	}
}

// =====================================================================================================================
// ====================================== WARPED LIVE JACOBIAN AND HESSIAN =============================================
// =====================================================================================================================
// This set of functions computes both the jacobain AND hessian of the live frame at the specified lookup position
// This hessian is perhaps an incorrect way to compute the level set term (since live frame doesn't have distortion)
//with color
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobianAndHessian(const CONSTPTR(Vector3i)& originalPosition,
                                                        const CONSTPTR(Vector3f)& originalWarp_t,
                                                        const CONSTPTR(TVoxelCanonical)* canonicalVoxels,
                                                        const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                                        THREADPTR(TCache)& canonicalCache,
                                                        const CONSTPTR(TVoxelLive)* liveVoxels,
                                                        const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                        THREADPTR(TCache)& liveCache,
                                                        THREADPTR(float)& warpedSdf,
                                                        THREADPTR(Vector3f)& warpedColor,
                                                        THREADPTR(Vector3f)& sdfJacobian,
                                                        THREADPTR(Vector3f)& colorJacobian,
                                                        THREADPTR(Matrix3f)& sdfHessian) {
	//position projected with the current warp
	Vector3f currentProjectedPosition = originalPosition.toFloat() + originalWarp_t;

	//=== shifted warped sdf locations, shift vector, alternative projected position
	//warpedSdf = interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition, liveCache, warpedColor);
	Vector3f warpedColorForward[3];

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === forward by 1 in each direction
	Vector3f warpedSdfForward(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 0), liveCache,
			                       warpedColorForward[0]),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 0), liveCache,
			                       warpedColorForward[1]),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, 1), liveCache,
			                       warpedColorForward[2]));
	// === back by 1 in each direction
	Vector3f warpedSdfBackward(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(-1, 0, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, -1, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, -1),
			                       liveCache));
	// === x-y, y-z, and x-z plane forward corners for 2nd derivatives
	Vector3f warpedSdfCorners(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 1, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 1), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 1), liveCache));
	//=========== COMPUTE JACOBIAN =====================================================================================
	sdfJacobian = warpedSdfForward - Vector3f(warpedSdf);
	colorJacobian = Vector3f(
			squareDistance(warpedColorForward[0], warpedColor),
			squareDistance(warpedColorForward[1], warpedColor),
			squareDistance(warpedColorForward[2], warpedColor));

	//=========== COMPUTE 2ND PARTIAL DERIVATIVES IN SAME DIRECTION ====================================================
	Vector3f sdfDerivatives_xx_yy_zz = warpedSdfForward - Vector3f(2.0f * warpedSdf) + warpedSdfBackward;

	//=== corner-voxel auxiliary derivatives for later 2nd derivative approximations
	//Along the x axis, case 1: (0,1,0)->(1,1,0)
	//Along the y axis, case 1: (0,0,1)->(0,1,1)
	//Along the z axis, case 1: (1,0,0)->(1,0,1)
	Vector3f cornerSdfDerivatives = Vector3f(
			warpedSdfCorners.x - warpedSdfForward.y,
			warpedSdfCorners.y - warpedSdfForward.z,
			warpedSdfCorners.z - warpedSdfForward.x);

	//===Compute the 2nd partial derivatives for different direction sequences
	Vector3f sdfDerivatives_xy_yz_zx = cornerSdfDerivatives - sdfJacobian;

	float vals[] = {sdfDerivatives_xx_yy_zz.x, sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xy_yz_zx.z,//r1
	                sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xx_yy_zz.y, sdfDerivatives_xy_yz_zx.y,//r2
	                sdfDerivatives_xy_yz_zx.z, sdfDerivatives_xy_yz_zx.y, sdfDerivatives_xx_yy_zz.z};
	sdfHessian.setValues(vals);
};

//_DEBUG -- this is an alternative version where truncated values' sign is determined by adding up the non-truncated values
//and checking the sign of the result
//without color
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobianAndHessian_TruncationFix(const CONSTPTR(Vector3i)& originalPosition,
                                                                      const CONSTPTR(Vector3f)& originalWarp_t,
                                                                      const CONSTPTR(TVoxelCanonical)* canonicalVoxels,
                                                                      const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                                                      THREADPTR(TCache)& canonicalCache,
                                                                      const CONSTPTR(TVoxelLive)* liveVoxels,
                                                                      const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                                      THREADPTR(TCache)& liveCache,
                                                                      const CONSTPTR(float) warpedSdf,
                                                                      THREADPTR(Vector3f)& sdfJacobian,
                                                                      THREADPTR(Matrix3f)& sdfHessian,
                                                                      const CONSTPTR(bool) printResult = false) {
	//position projected with the current warp
	Vector3f projPos = originalPosition.toFloat() + originalWarp_t;

	const int neighborCount = 9;
	const int neighborVecCount = 3;
	const Vector3f positions[neighborCount] = {Vector3f(1.f, 0.f, 0.f), Vector3f(0.f, 1.f, 0.f),
	                                           Vector3f(0.f, 0.f, 1.f),
	                                           Vector3f(-1.f, 0.f, 1.f), Vector3f(0.f, -1.f, 1.f),
	                                           Vector3f(0.f, 0.f, -1.f),
	                                           Vector3f(1.f, 1.f, 0.f), Vector3f(0.f, 1.f, 1.f),
	                                           Vector3f(1.f, 0.f, 1.f)};

	Vector3f neighborValues[neighborVecCount];

	int iNeighbor = 0;
	bool found;
	bool neighborsFound[neighborCount] = {0};
	float sumNonTruncated = 0.0f;
	bool haveTruncatedValues = false;

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	for (int iNeighborVec = 0; iNeighborVec < neighborVecCount; iNeighborVec++) {
		for (int iVecElement = 0; iVecElement < 3; iVecElement++, iNeighbor++) {
			float value =
					interpolateTrilinearly(liveVoxels, liveHashTable, projPos + positions[iNeighbor], liveCache, found);
			neighborValues[iNeighborVec][iVecElement] = value;
			neighborsFound[iNeighbor] = found;
			if (found) {
				sumNonTruncated += value;
			} else {
				haveTruncatedValues = true;
			}
		}
	}
	if (haveTruncatedValues) {
		iNeighbor = 0;
		float truncatedVal = copysign(1.0, sumNonTruncated);
		for (int iNeighborVec = 0; iNeighborVec < neighborVecCount; iNeighborVec++) {
			for (int iVecElement = 0; iVecElement < 3; iVecElement++, iNeighbor++) {
				if (!neighborsFound[iNeighbor]) {
					neighborValues[iNeighborVec][iVecElement] = truncatedVal;
				}
			}
		}
	}


	// === forward by 1 in each direction
	Vector3f& warpedSdfForward = neighborValues[0];
	// === back by 1 in each direction
	Vector3f& warpedSdfBackward = neighborValues[1];
	// === x-y, y-z, and x-z plane forward corners for 2nd derivatives
	Vector3f& warpedSdfCorners = neighborValues[2];

	//=========== COMPUTE JACOBIAN =====================================================================================
	sdfJacobian = warpedSdfForward - Vector3f(warpedSdf);

	//=========== COMPUTE 2ND PARTIAL DERIVATIVES IN SAME DIRECTION ====================================================
	Vector3f sdfDerivatives_xx_yy_zz = warpedSdfForward - Vector3f(2.0f * warpedSdf) + warpedSdfBackward;

	//=== corner-voxel auxiliary derivatives for later 2nd derivative approximations
	//Along the x axis, case 1: (0,1,0)->(1,1,0)
	//Along the y axis, case 1: (0,0,1)->(0,1,1)
	//Along the z axis, case 1: (1,0,0)->(1,0,1)
	Vector3f cornerSdfDerivatives = Vector3f(
			warpedSdfCorners.x - warpedSdfForward.y,
			warpedSdfCorners.y - warpedSdfForward.z,
			warpedSdfCorners.z - warpedSdfForward.x);

	//===Compute the 2nd partial derivatives for different direction sequences
	Vector3f sdfDerivatives_xy_yz_zx = cornerSdfDerivatives - sdfJacobian;

	float vals[] = {sdfDerivatives_xx_yy_zz.x, sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xy_yz_zx.z,
	                sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xx_yy_zz.y, sdfDerivatives_xy_yz_zx.y,
	                sdfDerivatives_xy_yz_zx.z, sdfDerivatives_xy_yz_zx.y, sdfDerivatives_xx_yy_zz.z};
	sdfHessian.setValues(vals);

	if (printResult) {
		std::cout << "Warped SDF Forward: " << warpedSdfForward << std::endl;
		std::cout << "Warped SDF Backward: " << warpedSdfBackward << std::endl;
		std::cout << "Warped SDF Corners: " << warpedSdfCorners << std::endl;
		std::cout << "Warped SDF Jacobian: " << sdfJacobian << std::endl << std::endl;
	}
};


//_DEBUG -- this is exactly the same code as below, just using the truncation-corrected version of interpolation
//without color
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobianAndHessian_Corrected2(const CONSTPTR(Vector3i)& originalPosition,
                                                                   const CONSTPTR(Vector3f)& originalWarp_t,
                                                                   const CONSTPTR(TVoxelCanonical)* canonicalVoxels,
                                                                   const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                                                   THREADPTR(TCache)& canonicalCache,
                                                                   const CONSTPTR(TVoxelLive)* liveVoxels,
                                                                   const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                                   THREADPTR(TCache)& liveCache,
                                                                   const CONSTPTR(float) warpedSdf,
                                                                   THREADPTR(Vector3f)& sdfJacobian,
                                                                   THREADPTR(Matrix3f)& sdfHessian,
                                                                   const CONSTPTR(bool) printResult = false) {
	//position projected with the current warp
	Vector3f currentProjectedPosition = originalPosition.toFloat() + originalWarp_t;

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === forward by 1 in each direction
	Vector3f warpedSdfForward(
			interpolateTrilinearly_Corrected(liveVoxels, liveHashTable,
			                                 currentProjectedPosition + Vector3f(1.f, 0.f, 0.f),
			                                 liveCache),
			interpolateTrilinearly_Corrected(liveVoxels, liveHashTable,
			                                 currentProjectedPosition + Vector3f(0.f, 1.f, 0.f),
			                                 liveCache),
			interpolateTrilinearly_Corrected(liveVoxels, liveHashTable,
			                                 currentProjectedPosition + Vector3f(0.f, 0.f, 1.f),
			                                 liveCache));
	// === back by 1 in each direction
	Vector3f warpedSdfBackward(
			interpolateTrilinearly_Corrected(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(-1, 0, 0),
			                                 liveCache),
			interpolateTrilinearly_Corrected(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, -1, 0),
			                                 liveCache),
			interpolateTrilinearly_Corrected(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, -1),
			                                 liveCache)
	);
	// === x-y, y-z, and x-z plane forward corners for 2nd derivatives
	Vector3f warpedSdfCorners(
			interpolateTrilinearly_Corrected(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 1, 0),
			                                 liveCache),
			interpolateTrilinearly_Corrected(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 1),
			                                 liveCache),
			interpolateTrilinearly_Corrected(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 1),
			                                 liveCache));


	//=========== COMPUTE JACOBIAN =====================================================================================
	sdfJacobian = warpedSdfForward - Vector3f(warpedSdf);

	//=========== COMPUTE 2ND PARTIAL DERIVATIVES IN SAME DIRECTION ====================================================
	Vector3f sdfDerivatives_xx_yy_zz = warpedSdfForward - Vector3f(2.0f * warpedSdf) + warpedSdfBackward;

	//=== corner-voxel auxiliary derivatives for later 2nd derivative approximations
	//Along the x axis, case 1: (0,1,0)->(1,1,0)
	//Along the y axis, case 1: (0,0,1)->(0,1,1)
	//Along the z axis, case 1: (1,0,0)->(1,0,1)
	Vector3f cornerSdfDerivatives = Vector3f(
			warpedSdfCorners.x - warpedSdfForward.y,
			warpedSdfCorners.y - warpedSdfForward.z,
			warpedSdfCorners.z - warpedSdfForward.x);

	//===Compute the 2nd partial derivatives for different direction sequences
	Vector3f sdfDerivatives_xy_yz_zx = cornerSdfDerivatives - sdfJacobian;

	float vals[] = {sdfDerivatives_xx_yy_zz.x, sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xy_yz_zx.z,//r1
	                sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xx_yy_zz.y, sdfDerivatives_xy_yz_zx.y,//r2
	                sdfDerivatives_xy_yz_zx.z, sdfDerivatives_xy_yz_zx.y, sdfDerivatives_xx_yy_zz.z};
	sdfHessian.setValues(vals);

	if (printResult) {
		std::cout << "Warped SDF Forward: " << warpedSdfForward << std::endl;
		std::cout << "Warped SDF Backward: " << warpedSdfBackward << std::endl;
		std::cout << "Warped SDF Corners: " << warpedSdfCorners << std::endl;
		std::cout << "Warped SDF Jacobian: " << sdfJacobian << std::endl << std::endl;
	}
};

//_DEBUG -- this is exactly the same code as the "Old" one, but with optional result-printing
//without color
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobianAndHessian(const CONSTPTR(Vector3i)& originalPosition,
                                                        const CONSTPTR(Vector3f)& originalWarp_t,
                                                        const CONSTPTR(TVoxelCanonical)* canonicalVoxels,
                                                        const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                                        THREADPTR(TCache)& canonicalCache,
                                                        const CONSTPTR(TVoxelLive)* liveVoxels,
                                                        const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                        THREADPTR(TCache)& liveCache,
                                                        const CONSTPTR(float) warpedSdf,
                                                        THREADPTR(Vector3f)& sdfJacobian,
                                                        THREADPTR(Matrix3f)& sdfHessian,
                                                        const CONSTPTR(bool) printResult = false) {
	//position projected with the current warp
	Vector3f currentProjectedPosition = originalPosition.toFloat() + originalWarp_t;

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === forward by 1 in each direction
	Vector3f warpedSdfForward(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1.f, 0.f, 0.f),
			                       liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0.f, 1.f, 0.f),
			                       liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0.f, 0.f, 1.f),
			                       liveCache));
	// === back by 1 in each direction
	Vector3f warpedSdfBackward(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(-1, 0, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, -1, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, -1), liveCache)
	);
	// === x-y, y-z, and x-z plane forward corners for 2nd derivatives
	Vector3f warpedSdfCorners(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 1, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 1), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 1), liveCache));


	//=========== COMPUTE JACOBIAN =====================================================================================
	sdfJacobian = warpedSdfForward - Vector3f(warpedSdf);

	//=========== COMPUTE 2ND PARTIAL DERIVATIVES IN SAME DIRECTION ====================================================
	Vector3f sdfDerivatives_xx_yy_zz = warpedSdfForward - Vector3f(2.0f * warpedSdf) + warpedSdfBackward;

	//=== corner-voxel auxiliary derivatives for later 2nd derivative approximations
	//Along the x axis, case 1: (0,1,0)->(1,1,0)
	//Along the y axis, case 1: (0,0,1)->(0,1,1)
	//Along the z axis, case 1: (1,0,0)->(1,0,1)
	Vector3f cornerSdfDerivatives = Vector3f(
			warpedSdfCorners.x - warpedSdfForward.y,
			warpedSdfCorners.y - warpedSdfForward.z,
			warpedSdfCorners.z - warpedSdfForward.x);

	//===Compute the 2nd partial derivatives for different direction sequences
	Vector3f sdfDerivatives_xy_yz_zx = cornerSdfDerivatives - sdfJacobian;

	float vals[] = {sdfDerivatives_xx_yy_zz.x, sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xy_yz_zx.z,//r1
	                sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xx_yy_zz.y, sdfDerivatives_xy_yz_zx.y,//r2
	                sdfDerivatives_xy_yz_zx.z, sdfDerivatives_xy_yz_zx.y, sdfDerivatives_xx_yy_zz.z};
	sdfHessian.setValues(vals);

	if (printResult) {
		std::cout << "Warped SDF Forward: " << warpedSdfForward << std::endl;
		std::cout << "Warped SDF Backward: " << warpedSdfBackward << std::endl;
		std::cout << "Warped SDF Corners: " << warpedSdfCorners << std::endl;
		std::cout << "Warped SDF Jacobian: " << sdfJacobian << std::endl << std::endl;
	}
}

//without color, no result printing
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobianAndHessian_Old(const CONSTPTR(Vector3i)& originalPosition,
                                                            const CONSTPTR(Vector3f)& originalWarp_t,
                                                            const CONSTPTR(TVoxelCanonical)* canonicalVoxels,
                                                            const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                                            THREADPTR(TCache)& canonicalCache,
                                                            const CONSTPTR(TVoxelLive)* liveVoxels,
                                                            const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                            THREADPTR(TCache)& liveCache,
                                                            const CONSTPTR(float) liveSdf,
                                                            THREADPTR(Vector3f)& sdfJacobian,
                                                            THREADPTR(Matrix3f)& sdfHessian) {
	//position projected with the current warp
	Vector3f currentProjectedPosition = originalPosition.toFloat() + originalWarp_t;

	//=== shifted warped sdf locations, shift vector, alternative projected position
	//liveSdf = interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition, liveCache);

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === back by 1 in each direction
	Vector3f warpedSdfBackward(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(-1, 0, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, -1, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, -1), liveCache)
	);
	// === forward by 1 in each direction
	Vector3f warpedSdfForward(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, 1), liveCache));
	// === x-y, y-z, and x-z plane forward corners for 2nd derivatives
	Vector3f warpedSdfCorners(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 1, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 1), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 1), liveCache));
	//=========== COMPUTE JACOBIAN =====================================================================================
	sdfJacobian = warpedSdfForward - Vector3f(liveSdf);

	//=========== COMPUTE 2ND PARTIAL DERIVATIVES IN SAME DIRECTION ====================================================
	Vector3f sdfDerivatives_xx_yy_zz = warpedSdfForward - Vector3f(2.0f * liveSdf) + warpedSdfBackward;

	//=== corner-voxel auxiliary derivatives for later 2nd derivative approximations
	//Along the x axis, case 1: (0,1,0)->(1,1,0)
	//Along the y axis, case 1: (0,0,1)->(0,1,1)
	//Along the z axis, case 1: (1,0,0)->(1,0,1)
	Vector3f cornerSdfDerivatives = Vector3f(
			warpedSdfCorners.x - warpedSdfForward.y,
			warpedSdfCorners.y - warpedSdfForward.z,
			warpedSdfCorners.z - warpedSdfForward.x);

	//===Compute the 2nd partial derivatives for different direction sequences
	Vector3f sdfDerivatives_xy_yz_zx = cornerSdfDerivatives - sdfJacobian;

	float vals[] = {sdfDerivatives_xx_yy_zz.x, sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xy_yz_zx.z,//r1
	                sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xx_yy_zz.y, sdfDerivatives_xy_yz_zx.y,//r2
	                sdfDerivatives_xy_yz_zx.z, sdfDerivatives_xy_yz_zx.y, sdfDerivatives_xx_yy_zz.z};
	sdfHessian.setValues(vals);
};

/**
 * \brief Finds neighbor voxel's relevant values in the order specified below.
 *     0        1        2          3         4         5           6         7         8
 *	(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
 * \tparam TVoxel
 * \tparam TCache
 * \param[out] warp_tData
 * \param[out] found
 * \param[in] voxelPosition exact position of voxel in the scene.
 * \param[in] voxelData
 * \param[in] hashTable
 * \param[in] cache
 */
template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPoint2ndDerivativeNeighborhoodWarp(THREADPTR(Vector3f)* warp_tData, //x9, out
                                                   THREADPTR(bool)* found, //x9, out
                                                   const CONSTPTR(Vector3i)& voxelPosition,
                                                   const CONSTPTR(TVoxel)* voxelData,
                                                   const CONSTPTR(ITMHashEntry)* hashTable,
                                                   THREADPTR(TCache)& cache) {
	int vmIndex;

	TVoxel voxel;
#define PROCESS_VOXEL(location, index)\
    voxel = readVoxel(voxelData, hashTable, voxelPosition + (location), vmIndex, cache);\
    warp_tData[index] = voxel.warp_t;\
    found[index] = vmIndex != 0;


	//necessary for 2nd derivatives in same direction, e.g. xx and zz
	PROCESS_VOXEL(Vector3i(-1, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, -1, 0), 1);
	PROCESS_VOXEL(Vector3i(0, 0, -1), 2);

	//necessary for 1st derivatives
	PROCESS_VOXEL(Vector3i(1, 0, 0), 3);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 4);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 5);

	//necessary for 2nd derivatives in mixed directions, e.g. xy and yz
	PROCESS_VOXEL(Vector3i(1, 1, 0), 6);//xy corner
	PROCESS_VOXEL(Vector3i(0, 1, 1), 7);//yz corner
	PROCESS_VOXEL(Vector3i(1, 0, 1), 8);//xz corner
#undef PROCESS_VOXEL
}

template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpJacobianAndHessian(const CONSTPTR(Vector3f)& originalWarp_t,
                                                  const CONSTPTR(Vector3i)& originalPosition,
                                                  const CONSTPTR(Vector3f*) warp_tNeighbors,
                                                  const CONSTPTR(bool*) neighborFound,
                                                  THREADPTR(Matrix3f)& jacobian,
                                                  THREADPTR(Matrix3f)* hessian, //x3
                                                  bool verbose = false) {
	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)

	// |u_x, u_y, u_z|       |m00, m10, m20|
	// |v_x, v_y, v_z|       |m01, m11, m21|
	// |w_x, w_y, w_z|       |m02, m12, m22|
	Vector3f zeroVector(0.0f);
	jacobian.setColumn(0, warp_tNeighbors[3] - originalWarp_t);//1st derivative in x
	jacobian.setColumn(1, warp_tNeighbors[4] - originalWarp_t);//1st derivative in y
	jacobian.setColumn(2, warp_tNeighbors[5] - originalWarp_t);//1st derivative in z

	Matrix3f backwardDifferences;
	// |u_x, u_y, u_z|
	// |v_x, v_y, v_z|
	// |w_x, w_y, w_z|
	backwardDifferences.setColumn(0, originalWarp_t - warp_tNeighbors[0]);//1st derivative in x
	backwardDifferences.setColumn(1, originalWarp_t - warp_tNeighbors[1]);//1st derivative in y
	backwardDifferences.setColumn(2, originalWarp_t - warp_tNeighbors[2]);//1st derivative in z

	//second derivatives in same direction
	// |u_xx, u_yy, u_zz|       |m00, m10, m20|
	// |v_xx, v_yy, v_zz|       |m01, m11, m21|
	// |w_xx, w_yy, w_zz|       |m02, m12, m22|
	Matrix3f dd_XX_YY_ZZ = jacobian - backwardDifferences;

	Matrix3f neighborDifferences;
	neighborDifferences.setColumn(0, warp_tNeighbors[6] - warp_tNeighbors[4]);//(0,1,0)->(1,1,0)
	neighborDifferences.setColumn(1, warp_tNeighbors[7] - warp_tNeighbors[5]);//(0,0,1)->(0,1,1)
	neighborDifferences.setColumn(2, warp_tNeighbors[8] - warp_tNeighbors[3]);//(1,0,0)->(1,0,1)


	//second derivatives in different directions
	// |u_xy, u_yz, u_zx|      |m00, m10, m20|
	// |v_xy, v_yz, v_zx|      |m01, m11, m21|
	// |w_xy, w_yz, w_zx|      |m02, m12, m22|
	Matrix3f dd_XY_YZ_ZX = neighborDifferences - jacobian;

	//NOTE: Hessian matrices are symmetric in this case
	// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
	// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
	// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
	float valsU[9] = {dd_XX_YY_ZZ.m00, dd_XY_YZ_ZX.m00, dd_XY_YZ_ZX.m20,
	                  dd_XY_YZ_ZX.m00, dd_XX_YY_ZZ.m10, dd_XY_YZ_ZX.m10,
	                  dd_XY_YZ_ZX.m20, dd_XY_YZ_ZX.m10, dd_XX_YY_ZZ.m20};
	hessian[0].setValues(valsU);

	// |0, 3, 6|     |m00, m10, m20|      |v_xx, v_xy, v_xz|
	// |1, 4, 7|     |m01, m11, m21|      |v_xy, v_yy, v_yz|
	// |2, 5, 8|     |m02, m12, m22|      |v_xz, v_yz, v_zz|
	float valsV[9] = {dd_XX_YY_ZZ.m01, dd_XY_YZ_ZX.m01, dd_XY_YZ_ZX.m21,
	                  dd_XY_YZ_ZX.m01, dd_XX_YY_ZZ.m11, dd_XY_YZ_ZX.m11,
	                  dd_XY_YZ_ZX.m21, dd_XY_YZ_ZX.m11, dd_XX_YY_ZZ.m21};
	hessian[1].setValues(valsV);

	// |0, 3, 6|     |m00, m10, m20|      |w_xx, w_xy, w_xz|
	// |1, 4, 7|     |m01, m11, m21|      |w_xy, w_yy, w_yz|
	// |2, 5, 8|     |m02, m12, m22|      |w_xz, w_yz, w_zz|
	float valsW[9] = {dd_XX_YY_ZZ.m02, dd_XY_YZ_ZX.m02, dd_XY_YZ_ZX.m22,
	                  dd_XY_YZ_ZX.m02, dd_XX_YY_ZZ.m12, dd_XY_YZ_ZX.m12,
	                  dd_XY_YZ_ZX.m22, dd_XY_YZ_ZX.m12, dd_XX_YY_ZZ.m22};
	hessian[2].setValues(valsW);
	if (verbose) {
		const int neighborhoodSize = 9;
		const std::string yellow("\033[0;33m");
		const std::string cyan("\033[0;36m");
		const std::string green("\033[0;32m");
		const std::string reset("\033[0m");
		//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
		Vector3i neighborPositions[] = {Vector3i(-1, 0, 0), Vector3i(0, -1, 0), Vector3i(0, 0, -1), Vector3i(1, 0, 0),
		                                Vector3i(0, 1, 0), Vector3i(0, 0, 1), Vector3i(1, 1, 0), Vector3i(0, 1, 1),
		                                Vector3i(1, 0, 1),};
		std::cout << "Boundary neighbors: ";
		for (int iNeightbor = 0; iNeightbor < neighborhoodSize; iNeightbor++) {
			if (!neighborFound[iNeightbor]) {
				std::cout << iNeightbor << ", ";
			}
		}
		std::cout << std::endl << green;
		std::cout << "Neighbors' warps: " << std::endl;
		for (int iNeightbor = 0; iNeightbor < neighborhoodSize; iNeightbor++) {
			std::cout << neighborPositions[iNeightbor] << ": " << warp_tNeighbors[iNeightbor] << ", " << std::endl;
		}

		std::cout << std::endl << yellow;
		std::cout << "Jacobian: " << std::endl << jacobian << std::endl << cyan;
		std::cout << "Hessian: " << std::endl << hessian[0] << hessian[1] << hessian[2] << reset << std::endl;
	}
};


//TODO: refactor so this just simply returns the resulting allocation type (and updates hashIdx) -Greg (GitHub: Algomorph)
/**
 * \brief Determines whether the hash block at the specified block position needs it's voxels to be allocated, as well
 * as whether they should be allocated in the excess list or the ordered list of the hash table.
 * If any of these are true, marks the corresponding entry in \param entriesAllocType
 * \param[in,out] entriesAllocType  array where to set the allocation type at final hashIdx index
 * \param[in,out] blockCoords  array block coordinates for the new hash blocks at final hashIdx index
 * \param[in,out] hashIdx  final index of the hash block to be allocated (may be updated based on hash closed chaining)
 * \param[in] hashBlockPosition  position of the hash block to check / allocate
 * \param[in] hashTable  hash table with existing blocks
 * \return true if the block needs allocation, false otherwise
 */
_CPU_AND_GPU_CODE_
inline bool MarkIfNeedsAllocation(DEVICEPTR(uchar)* entriesAllocType,
                                  DEVICEPTR(Vector3s)* blockCoords,
                                  THREADPTR(int)& hashIdx,
                                  const CONSTPTR(Vector3s)& hashBlockPosition,
                                  const CONSTPTR(ITMHashEntry)* hashTable) {


	ITMHashEntry hashEntry = hashTable[hashIdx];
	//check if hash table contains entry

	if (!(IS_EQUAL3(hashEntry.pos, hashBlockPosition) && hashEntry.ptr >= -1)) {
		if (hashEntry.ptr >= -1) {
			//search excess list only if there is no room in ordered part
			while (hashEntry.offset >= 1) {
				hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
				hashEntry = hashTable[hashIdx];

				if (IS_EQUAL3(hashEntry.pos, hashBlockPosition) && hashEntry.ptr >= -1) {
					return false;
				}
			}
			entriesAllocType[hashIdx] = ITMLib::NEEDS_ALLOC_IN_EXCESS_LIST;
			blockCoords[hashIdx] = hashBlockPosition;
			return true;
		}
		entriesAllocType[hashIdx] = ITMLib::NEEDS_ALLOC_IN_ORDERED_LIST;
		blockCoords[hashIdx] = hashBlockPosition;
		return true;
	}
	// already have hash block, no allocation needed
	return false;

};