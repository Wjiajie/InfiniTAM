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
#include "../../Utils/ITMHashBlockProperties.h"
#include "../../Objects/Scene/ITMTrilinearInterpolation.h"

#include <cfloat>

#define FLT_EPSILON2 FLT_EPSILON //_DEBUG (convert define uses to FLT_EPSILON?)
//#define FLT_EPSILON2 10e-5 //_DEBUG

_CPU_AND_GPU_CODE_
inline float squareDistance(const CONSTPTR(Vector3f)& vec1,
                            const CONSTPTR(Vector3f)& vec2) {
	Vector3f difference = vec1 - vec2;
	return dot(difference, difference);
}

// =====================================================================================================================
// ========================================EXPLORATION OF NEIGHBORHOOD AROUND CANONICAL VOXEL===========================
// =====================================================================================================================
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
    neighborAllocated[index] = vmIndex != 0;\
    neighborTruncated[index] = voxel.flags == ITMLib::VOXEL_TRUNCATED;

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

// =====================================================================================================================
// ================================================= WARPED LIVE JACOBIAN (DATA TERM) ==================================
// =====================================================================================================================
// this set of functions simply computes the jacobian of the live scene at an interpolated warped position, without
// computing the hessian. The jacobian vector defines the direction of data term's contribution to the warp updates.
template<typename TVoxelLive, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeLiveSdf_Center_WarpForward(
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
inline void ComputePerPointLiveJacobian(const CONSTPTR(Vector3i)& voxelPosition,
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

// with color
template<typename TVoxelLive, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobian(const CONSTPTR(Vector3i)& voxelPosition,
                                              const CONSTPTR(Vector3f)& voxelWarp,
                                              const CONSTPTR(TVoxelLive)* liveVoxels,      //| =============== |
                                              const CONSTPTR(ITMHashEntry)* liveHashTable, //| live scene data |
                                              THREADPTR(TCache)& liveCache,                //| =============== |
                                              const CONSTPTR(float)& liveSdf,
                                              const CONSTPTR(Vector3f)& liveColor,
                                              THREADPTR(Vector3f)& liveSdfJacobian,
                                              THREADPTR(Vector3f)& liveSdf_Center_WarpForward,
                                              THREADPTR(Vector3f)& colorJacobian) {
	//position projected with the current warp
	Vector3f centerPos = voxelPosition.toFloat();
	Vector3f projectedPosition = centerPos + voxelWarp;
	Vector3f warpedSdf_Center_WarpCenter(liveSdf);

	//=== shifted warped sdf locations, shift vector, alternative projected position
	Vector3f warpedColor_Center_WarpForward[3];

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === forward by 1 in each direction
	liveSdf_Center_WarpForward = Vector3f(
			InterpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition + Vector3f(1, 0, 0), liveCache,
			                       warpedColor_Center_WarpForward[0]),
			InterpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition + Vector3f(0, 1, 0), liveCache,
			                       warpedColor_Center_WarpForward[1]),
			InterpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition + Vector3f(0, 0, 1), liveCache,
			                       warpedColor_Center_WarpForward[2]));
	//=========== COMPUTE JACOBIAN =====================================================================================
	liveSdfJacobian = liveSdf_Center_WarpForward - warpedSdf_Center_WarpCenter;
	colorJacobian = Vector3f(
			squareDistance(warpedColor_Center_WarpForward[0], liveColor),
			squareDistance(warpedColor_Center_WarpForward[1], liveColor),
			squareDistance(warpedColor_Center_WarpForward[2], liveColor));
};

// =====================================================================================================================
// ========================================== WARPED JACOBIAN AND HESSIAN (LEVEL SET TERM) =============================
// =====================================================================================================================

// This function samples the live scene using warps of canonical voxels in a neighborhood around a particular voxel.
// From the computed values, the gradient (jacobian) is computed, as well as its derivative ("hessian") with respect
// to changing the warp at the current voxel.
template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeWarpedJacobianAndHessian(const CONSTPTR(Vector3f*) neighborWarps,
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
//========================================= WARP JACOBIAN AND HESSIAN (KILLING TERM) ===================================
//======================================================================================================================
//Computes the jacobian and hessian approximation for the warp vectors themselves in a given neighborhood
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







