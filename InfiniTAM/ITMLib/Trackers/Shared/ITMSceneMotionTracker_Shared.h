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
#include "../../Utils/ITMNeighborVoxelIterationInfo.h"


_CPU_AND_GPU_CODE_
inline float squareDistance(const CONSTPTR(Vector3f)& vec1,
                            const CONSTPTR(Vector3f)& vec2) {
	Vector3f difference = vec1 - vec2;
	return dot(difference, difference);
}

// =====================================================================================================================
// ========================================== WARPED JACOBIAN AND HESSIAN ==============================================
// =====================================================================================================================
// This function samples the live scene using warps of canonical voxels in a neighborhood around a particular voxel.
// From the computed values, the gradient (jacobian) is computed, as well as its derivative ("hessian") with respect
// to changing the warp at the current voxel.
template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeWarpedJacobianAndHessian(const CONSTPTR(Vector3f*) warp_tNeighbors,
                                            const CONSTPTR(Vector3i)& voxelPosition,
                                            const CONSTPTR(Vector3f)& liveSdf_Center_WarpForward,
                                            const CONSTPTR(float)& liveSdf,//lookup value at center warp_t
                                            const CONSTPTR(TVoxel)* liveVoxels,
                                            const CONSTPTR(ITMHashEntry)* liveHashTable,
                                            THREADPTR(TCache)& liveCache,
                                            THREADPTR(Vector3f)& warpedSdfJacobian, //out
                                            THREADPTR(Matrix3f)& warpedSdfHessian, //out
                                            bool printResult) {
	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	Vector3f warpedSdf_Center_WarpCenter(liveSdf);
	Vector3f centerPos = voxelPosition.toFloat();
	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================

	// === check lookups for voxels forward by 1 in x, y, z of the canonical frame, use their own warps
	Vector3f lookupPos_xForward_WarpCenter = centerPos + Vector3f(1.f, 0.f, 0.f) + warp_tNeighbors[3];
	Vector3f lookupPos_yForward_WarpCenter = centerPos + Vector3f(0.f, 1.f, 0.f) + warp_tNeighbors[4];
	Vector3f lookupPos_zForward_WarpCenter = centerPos + Vector3f(0.f, 0.f, 1.f) + warp_tNeighbors[5];
	Vector3f warpedSdf_Forward_WarpCenter(
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_xForward_WarpCenter, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_yForward_WarpCenter, liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, lookupPos_zForward_WarpCenter, liveCache)
	);
	//=========== WARPED SDF JACOBIAN ==================================================================================
	// derivatives of the type [d phi (warp) / dx]
	warpedSdfJacobian = warpedSdf_Forward_WarpCenter - warpedSdf_Center_WarpCenter;

	//=========== COMPUTE 2ND PARTIAL DERIVATIVES IN SAME DIRECTION ====================================================
	//we keep the neighbors warps fixed, but we see how the gradient (jacobian above) changes if we change current
	//voxel's warp
	Vector3f sdfDerivatives_ux_vy_wz =
			(warpedSdf_Forward_WarpCenter - liveSdf_Center_WarpForward) - warpedSdfJacobian;

	//=== corner-voxel auxiliary first partial first derivatives for later 2nd derivative approximations
	// vx - v //how does a change of warp in v affect the change [in calonical neighborhood] in x
	// uy - u
	// uz - u
	Vector3f changedWarpFirstDerivatives1 = warpedSdf_Forward_WarpCenter - Vector3f(liveSdf_Center_WarpForward.v,
	                                                                                liveSdf_Center_WarpForward.u,
	                                                                                liveSdf_Center_WarpForward.u);
	// wx - w
	// wy - w
	// vz - v
	Vector3f changedWarpFirstDerivatives2 = warpedSdf_Forward_WarpCenter - Vector3f(liveSdf_Center_WarpForward.w,
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

	if (printResult) {
		std::cout << "Back-projected SDF Jacobian: " << warpedSdfJacobian << std::endl << std::endl;
	}
};

// =====================================================================================================================
// ================================================= WARPED LIVE JACOBIAN ==============================================
// =====================================================================================================================
// this set of functions simply computes the jacobian of the live scene at an interpolated warped position, without
// computing the hessian. The jacobian vector defines the direction of data term's contribution to the warp updates.

// with color
template<typename TVoxelLive, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobian(const CONSTPTR(Vector3i)& originalPosition,
                                              const CONSTPTR(Vector3f)& originalWarp_t,
                                              const CONSTPTR(TVoxelLive)* liveVoxels,
                                              const CONSTPTR(ITMHashEntry)* liveHashTable,
                                              THREADPTR(TCache)& liveCache,
                                              const CONSTPTR(float)& liveSdf,
                                              const CONSTPTR(Vector3f)& liveColor,
                                              THREADPTR(Vector3f)& liveSdfJacobian,
                                              THREADPTR(Vector3f)& liveSdf_Center_WarpForward,
                                              THREADPTR(Vector3f)& colorJacobian) {
	//position projected with the current warp
	Vector3f centerPos = originalPosition.toFloat();
	Vector3f projectedPosition = centerPos + originalWarp_t;
	Vector3f warpedSdf_Center_WarpCenter(liveSdf);

	//=== shifted warped sdf locations, shift vector, alternative projected position
	Vector3f warpedColor_Center_WarpForward[3];

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === forward by 1 in each direction
	liveSdf_Center_WarpForward = Vector3f(
			interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition + Vector3f(1, 0, 0), liveCache,
			                       warpedColor_Center_WarpForward[0]),
			interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition + Vector3f(0, 1, 0), liveCache,
			                       warpedColor_Center_WarpForward[1]),
			interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition + Vector3f(0, 0, 1), liveCache,
			                       warpedColor_Center_WarpForward[2]));
	//=========== COMPUTE JACOBIAN =====================================================================================
	liveSdfJacobian = liveSdf_Center_WarpForward - warpedSdf_Center_WarpCenter;
	colorJacobian = Vector3f(
			squareDistance(warpedColor_Center_WarpForward[0], liveColor),
			squareDistance(warpedColor_Center_WarpForward[1], liveColor),
			squareDistance(warpedColor_Center_WarpForward[2], liveColor));
};

//without color
template<typename TVoxelLive, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobian(const CONSTPTR(Vector3i)& originalPosition,
                                              const CONSTPTR(Vector3f)& originalWarp_t,

                                              const CONSTPTR(TVoxelLive)* liveVoxels,
                                              const CONSTPTR(ITMHashEntry)* liveHashTable,
                                              THREADPTR(TCache)& liveCache,

                                              const CONSTPTR(float) liveSdf,
                                              THREADPTR(Vector3f)& liveSdfJacobian,
                                              THREADPTR(Vector3f)& liveSdf_Center_WarpForward,
                                              const CONSTPTR(bool) printResult = false) {
	//position projected with the current warp
	Vector3f centerPos = originalPosition.toFloat();
	Vector3f projectedPosition = centerPos + originalWarp_t;
	Vector3f liveSdf_Center_WarpCenter(liveSdf);

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === increment the warp by 1 in each direction and use them to check what interpolated values from live frame map there
	liveSdf_Center_WarpForward =
			Vector3f(interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition + Vector3f(1.f, 0.f, 0.f),
			                                liveCache),
			         interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition + Vector3f(0.f, 1.f, 0.f),
			                                liveCache),
			         interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition + Vector3f(0.f, 0.f, 1.f),
			                                liveCache));
	//=========== COMPUTE JACOBIAN =====================================================================================
	liveSdfJacobian = liveSdf_Center_WarpForward - liveSdf_Center_WarpCenter;

	//=========== PRINT RESULT IF REQUESTED ============================================================================
	if (printResult) {
		std::cout << "Live SDF Forward at warp: " << liveSdf_Center_WarpForward << std::endl;
		std::cout << "Live SDF Jacobian at warp: " << liveSdfJacobian << std::endl;
	}
}

// =====================================================================================================================
// ====================================== WARPED LIVE JACOBIAN AND HESSIAN =============================================
// =====================================================================================================================
// This set of functions computes both the jacobain AND hessian of the live frame at the specified lookup position
// This hessian is perhaps an incorrect way to compute the level set term (since live frame doesn't have distortion)
//with color
template<typename TVoxelLive, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointLiveJacobianAndHessian(const CONSTPTR(Vector3i)& originalPosition,
                                                  const CONSTPTR(Vector3f)& originalWarp_t,
                                                  const CONSTPTR(TVoxelLive)* liveVoxels,
                                                  const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                  THREADPTR(TCache)& liveCache,
                                                  THREADPTR(float)& liveSdf,
                                                  THREADPTR(Vector3f)& liveColor,
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
	sdfJacobian = warpedSdfForward - Vector3f(liveSdf);
	colorJacobian = Vector3f(
			squareDistance(warpedColorForward[0], liveColor),
			squareDistance(warpedColorForward[1], liveColor),
			squareDistance(warpedColorForward[2], liveColor));

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


//_DEBUG -- this is exactly the same code as the "Old" one, but with optional result-printing
//without color
template<typename TVoxelLive, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointLiveJacobianAndHessian(const CONSTPTR(Vector3i)& originalPosition,
                                                  const CONSTPTR(Vector3f)& originalWarp_t,
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

//_DEBUG -- this is an alternative version where truncated values' sign is determined by adding up the non-truncated values
//and checking the sign of the result
//without color
template<typename TVoxelLive, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobianAndHessian_TruncationFix(const CONSTPTR(Vector3i)& originalPosition,
                                                                      const CONSTPTR(Vector3f)& originalWarp_t,
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
		float truncatedVal = std::copysign(1.0f, sumNonTruncated);
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
 * \brief Finds neighbor voxel's warps in the order specified below.
 *     0        1        2          3         4         5           6         7         8
 *	(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
 * \tparam TVoxel
 * \tparam TCache
 * \param[out] neighborWarps
 * \param[out] neighborAllocated - current behavior is:
 * 1) record unallocated voxels as non-found
 * 2) truncated voxels marked unknown or known as found
 * 3) everything else (non-truncated), of course, as found
 * \param[in] voxelPosition exact position of voxel in the scene.
 * \param[in] voxelData
 * \param[in] hashTable
 * \param[in] cache
 */
template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPoint2ndDerivativeNeighborhoodWarp(THREADPTR(Vector3f)* neighborWarps, //x9, out
                                                   THREADPTR(bool)* neighborAllocated, //x9, out
                                                   THREADPTR(bool)* neighborTruncated, //x9, out
                                                   const CONSTPTR(Vector3i)& voxelPosition,
                                                   const CONSTPTR(TVoxel)* voxelData,
                                                   const CONSTPTR(ITMHashEntry)* hashTable,
                                                   THREADPTR(TCache)& cache) {
	int vmIndex;

	TVoxel voxel;
#define PROCESS_VOXEL(location, index)\
    voxel = readVoxel(voxelData, hashTable, voxelPosition + (location), vmIndex, cache);\
    neighborWarps[index] = voxel.warp_t;\
    neighborAllocated[index] = vmIndex != 0;/* && voxel.flags != ITMLib::VOXEL_UNKNOWN;*/\
	neighborTruncated[index] = voxel.flags != ITMLib::VOXEL_NONTRUNCATED;

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

_CPU_AND_GPU_CODE_
inline void ComputePerVoxelWarpJacobianAndHessian(const CONSTPTR(Vector3f)& voxelWarp,
                                                  const CONSTPTR(Vector3i)& voxelPosition,
                                                  const CONSTPTR(Vector3f*) neighborWarps,
                                                  THREADPTR(Matrix3f)& jacobian, //out
                                                  THREADPTR(Matrix3f)* hessian //out, x3
) {
	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)

	// |u_x, u_y, u_z|       |m00, m10, m20|
	// |v_x, v_y, v_z|       |m01, m11, m21|
	// |w_x, w_y, w_z|       |m02, m12, m22|
	Vector3f zeroVector(0.0f);
	jacobian.setColumn(0, neighborWarps[3] - voxelWarp);//1st derivative in x
	jacobian.setColumn(1, neighborWarps[4] - voxelWarp);//1st derivative in y
	jacobian.setColumn(2, neighborWarps[5] - voxelWarp);//1st derivative in z

	Matrix3f backwardDifferences;
	// |u_x, u_y, u_z|
	// |v_x, v_y, v_z|
	// |w_x, w_y, w_z|
	backwardDifferences.setColumn(0, voxelWarp - neighborWarps[0]);//1st derivative in x
	backwardDifferences.setColumn(1, voxelWarp - neighborWarps[1]);//1st derivative in y
	backwardDifferences.setColumn(2, voxelWarp - neighborWarps[2]);//1st derivative in z

	//second derivatives in same direction
	// |u_xx, u_yy, u_zz|       |m00, m10, m20|
	// |v_xx, v_yy, v_zz|       |m01, m11, m21|
	// |w_xx, w_yy, w_zz|       |m02, m12, m22|
	Matrix3f dd_XX_YY_ZZ = jacobian - backwardDifferences;

	Matrix3f neighborDifferences;
	neighborDifferences.setColumn(0, neighborWarps[6] - neighborWarps[4]);//(0,1,0)->(1,1,0)
	neighborDifferences.setColumn(1, neighborWarps[7] - neighborWarps[5]);//(0,0,1)->(0,1,1)
	neighborDifferences.setColumn(2, neighborWarps[8] - neighborWarps[3]);//(1,0,0)->(1,0,1)


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

};

_CPU_AND_GPU_CODE_
inline void PrintPerVoxelWarpJacobianAndHessian(const CONSTPTR(Vector3f*) neighborWarps,
                                                const CONSTPTR(bool*) neighborFound,
                                                THREADPTR(Matrix3f)& jacobian, //in
                                                THREADPTR(Matrix3f)* hessian //in, x3
) {

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
		std::cout << neighborPositions[iNeightbor] << ": " << neighborWarps[iNeightbor] << ", " << std::endl;
	}

	std::cout << std::endl << yellow;
	std::cout << "Jacobian: " << std::endl << jacobian << std::endl << cyan;
	std::cout << "Hessian: " << std::endl << hessian[0] << hessian[1] << hessian[2] << reset << std::endl;
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


template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel
readVoxel(const CONSTPTR(TVoxel)* voxelData, const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* voxelIndex,
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
		TVoxelCanonical voxel = readVoxel(canonicalVoxelData, canonicalHashTable, neighborPosition, vmIndex, cache,
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
		info.liveSdf = interpolateTrilinearly(liveVoxelData, liveHashTable, TO_FLOAT3(neighborPosition) + voxel.warp_t,
		                                      liveCache, info.liveFound);
		iNeighbor++;
	}
}
