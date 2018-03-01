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
#include "../../Objects/Scene/ITMTrilinearInterpolation.h"
#include "../../Utils/ITMHashBlockProperties.h"
#include <cmath>

// *********************************************************************************************************************
// ********************************* DEPRECATED (WILL BE REMOVED BEFORE MASTER MERGE) **********************************
// *********************************************************************************************************************
_CPU_AND_GPU_CODE_
inline float squareDistance2(const CONSTPTR(Vector3f)& vec1,
                            const CONSTPTR(Vector3f)& vec2) {
	Vector3f difference = vec1 - vec2;
	return dot(difference, difference);
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
//warpedSdf = InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition, liveCache, warpedColor);
	Vector3f warpedColorForward[3];

//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
// === forward by 1 in each direction
	Vector3f warpedSdfForward(
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 0), liveCache,
			                       warpedColorForward[0]),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 0), liveCache,
			                       warpedColorForward[1]),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, 1), liveCache,
			                       warpedColorForward[2]));
// === back by 1 in each direction
	Vector3f warpedSdfBackward(
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(-1, 0, 0), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, -1, 0), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, -1),
			                       liveCache));
// === x-y, y-z, and x-z plane forward corners for 2nd derivatives
	Vector3f warpedSdfCorners(
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 1, 0), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 1), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 1), liveCache));
//=========== COMPUTE JACOBIAN =====================================================================================
	sdfJacobian = warpedSdfForward - Vector3f(liveSdf);
	colorJacobian = Vector3f(
			squareDistance2(warpedColorForward[0], liveColor),
			squareDistance2(warpedColorForward[1], liveColor),
			squareDistance2(warpedColorForward[2], liveColor));

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
inline void ComputePerPointLiveJacobianAndHessian(const CONSTPTR(Vector3i)& voxelPosition,
                                                  const CONSTPTR(Vector3f)& voxelWarp,
                                                  const CONSTPTR(TVoxelLive)* liveVoxels,
                                                  const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                  THREADPTR(TCache)& liveCache,
                                                  const CONSTPTR(float) warpedSdf,
                                                  THREADPTR(Vector3f)& sdfJacobian,
                                                  THREADPTR(Matrix3f)& sdfHessian) {
//position projected with the current warp
	Vector3f currentProjectedPosition = voxelPosition.toFloat() + voxelWarp;

//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
// === forward by 1 in each direction
	Vector3f warpedSdfForward(
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1.f, 0.f, 0.f),
			                       liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0.f, 1.f, 0.f),
			                       liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0.f, 0.f, 1.f),
			                       liveCache));
// === back by 1 in each direction
	Vector3f warpedSdfBackward(
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(-1, 0, 0), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, -1, 0), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, -1), liveCache)
	);
// === x-y, y-z, and x-z plane forward corners for 2nd derivatives
	Vector3f warpedSdfCorners(
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 1, 0), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 1), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 1), liveCache));


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
					InterpolateTrilinearly(liveVoxels, liveHashTable, projPos + positions[iNeighbor], liveCache, found);
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
			InterpolateTrilinearly_TruncatedCopySign(liveVoxels, liveHashTable,
			                                         currentProjectedPosition + Vector3f(1.f, 0.f, 0.f),
			                                         liveCache),
			InterpolateTrilinearly_TruncatedCopySign(liveVoxels, liveHashTable,
			                                         currentProjectedPosition + Vector3f(0.f, 1.f, 0.f),
			                                         liveCache),
			InterpolateTrilinearly_TruncatedCopySign(liveVoxels, liveHashTable,
			                                         currentProjectedPosition + Vector3f(0.f, 0.f, 1.f),
			                                         liveCache));
// === back by 1 in each direction
	Vector3f warpedSdfBackward(
			InterpolateTrilinearly_TruncatedCopySign(liveVoxels, liveHashTable,
			                                         currentProjectedPosition + Vector3f(-1, 0, 0),
			                                         liveCache),
			InterpolateTrilinearly_TruncatedCopySign(liveVoxels, liveHashTable,
			                                         currentProjectedPosition + Vector3f(0, -1, 0),
			                                         liveCache),
			InterpolateTrilinearly_TruncatedCopySign(liveVoxels, liveHashTable,
			                                         currentProjectedPosition + Vector3f(0, 0, -1),
			                                         liveCache)
	);
// === x-y, y-z, and x-z plane forward corners for 2nd derivatives
	Vector3f warpedSdfCorners(
			InterpolateTrilinearly_TruncatedCopySign(liveVoxels, liveHashTable,
			                                         currentProjectedPosition + Vector3f(1, 1, 0),
			                                         liveCache),
			InterpolateTrilinearly_TruncatedCopySign(liveVoxels, liveHashTable,
			                                         currentProjectedPosition + Vector3f(0, 1, 1),
			                                         liveCache),
			InterpolateTrilinearly_TruncatedCopySign(liveVoxels, liveHashTable,
			                                         currentProjectedPosition + Vector3f(1, 0, 1),
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
//liveSdf = InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition, liveCache);

//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
// === back by 1 in each direction
	Vector3f warpedSdfBackward(
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(-1, 0, 0), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, -1, 0), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, -1), liveCache)
	);
// === forward by 1 in each direction
	Vector3f warpedSdfForward(
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 0), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 0), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, 1), liveCache));
// === x-y, y-z, and x-z plane forward corners for 2nd derivatives
	Vector3f warpedSdfCorners(
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 1, 0), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 1), liveCache),
			InterpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 1), liveCache));
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
