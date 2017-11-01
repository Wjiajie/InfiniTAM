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


_CPU_AND_GPU_CODE_
inline float squareDistance(const CONSTPTR(Vector3f)& vec1,
                            const CONSTPTR(Vector3f)& vec2) {
	Vector3f difference = vec1 - vec2;
	return dot(difference, difference);
}

//with color
template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobianAndHessian(const CONSTPTR(Vector3i)& originalPosition,
                                                        const CONSTPTR(Vector3f)& originalWarp_t,
                                                        const CONSTPTR(TVoxel)* canonicalVoxels,
                                                        const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                                        THREADPTR(TCache)& canonicalCache,
                                                        const CONSTPTR(ITMVoxelAux)* liveVoxels,
                                                        const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                        THREADPTR(TCache)& liveCache,
                                                        THREADPTR(float)& warpedSdf,
                                                        THREADPTR(Vector3f)& warpedColor,
                                                        THREADPTR(Vector3f)& sdfJacobean,
                                                        THREADPTR(Vector3f)& colorJacobean,
                                                        THREADPTR(Matrix3f)& sdfHessian) {
	//position projected with the current warp
	Vector3f currentProjectedPosition = originalPosition.toFloat() + originalWarp_t;

	//=== shifted warped sdf locations, shift vector, alternative projected position
	warpedSdf = interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition, liveCache, warpedColor);
	Vector3f warpedColorForward[3];

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === forward by 1 in each direction
	Vector3f warpedSdfForward(
			interpolateTrilinearly(liveVoxels, liveHashTable,currentProjectedPosition + Vector3f(1, 0, 0), liveCache,
			                       warpedColorForward[0]),
			interpolateTrilinearly(liveVoxels, liveHashTable,currentProjectedPosition + Vector3f(0, 1, 0), liveCache,
			                       warpedColorForward[1]),
			interpolateTrilinearly(liveVoxels, liveHashTable,currentProjectedPosition + Vector3f(0, 0, 1), liveCache,
			                       warpedColorForward[2]));
	// === back by 1 in each direction
	Vector3f warpedSdfBackward(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(-1, 0, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, -1, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, -1), liveCache));
	// === x-y, y-z, and x-z plane forward corners for 2nd derivatives
	Vector3f warpedSdfCorners(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 1, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 1), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 1), liveCache));
	//=========== COMPUTE JACOBEAN =====================================================================================
	sdfJacobean = warpedSdfForward - Vector3f(warpedSdf);
	colorJacobean = Vector3f(
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
	Vector3f sdfDerivatives_xy_yz_zx = cornerSdfDerivatives - sdfJacobean;

	float vals[] = {sdfDerivatives_xx_yy_zz.x, sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xy_yz_zx.z,//r1
	                sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xx_yy_zz.y, sdfDerivatives_xy_yz_zx.y,//r2
	                sdfDerivatives_xy_yz_zx.z, sdfDerivatives_xy_yz_zx.y, sdfDerivatives_xx_yy_zz.z};
	sdfHessian.setValues(vals);
};

//without color
template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobianAndHessian(const CONSTPTR(Vector3i)& originalPosition,
                                                        const CONSTPTR(Vector3f)& originalWarp_t,
                                                        const CONSTPTR(TVoxel)* canonicalVoxels,
                                                        const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                                        THREADPTR(TCache)& canonicalCache,
                                                        const CONSTPTR(ITMVoxelAux)* liveVoxels,
                                                        const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                        THREADPTR(TCache)& liveCache,
                                                        THREADPTR(float)& warpedSdf,
                                                        THREADPTR(Vector3f)& sdfJacobean,
                                                        THREADPTR(Matrix3f)& sdfHessian) {
	//position projected with the current warp
	Vector3f currentProjectedPosition = originalPosition.toFloat() + originalWarp_t;

	//=== shifted warped sdf locations, shift vector, alternative projected position
	warpedSdf = interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition, liveCache);

	//=========== LOOKUP WITH ALTERNATIVE WARPS ========================================================================
	// === forward by 1 in each direction
	Vector3f warpedSdfForward(
			interpolateTrilinearly(liveVoxels, liveHashTable,currentProjectedPosition + Vector3f(1, 0, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable,currentProjectedPosition + Vector3f(0, 1, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable,currentProjectedPosition + Vector3f(0, 0, 1), liveCache));
	// === back by 1 in each direction
	Vector3f warpedSdfBackward(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(-1, 0, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, -1, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 0, -1), liveCache));
	// === x-y, y-z, and x-z plane forward corners for 2nd derivatives
	Vector3f warpedSdfCorners(
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 1, 0), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(0, 1, 1), liveCache),
			interpolateTrilinearly(liveVoxels, liveHashTable, currentProjectedPosition + Vector3f(1, 0, 1), liveCache));
	//=========== COMPUTE JACOBEAN =====================================================================================
	sdfJacobean = warpedSdfForward - Vector3f(warpedSdf);

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
	Vector3f sdfDerivatives_xy_yz_zx = cornerSdfDerivatives - sdfJacobean;

	float vals[] = {sdfDerivatives_xx_yy_zz.x, sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xy_yz_zx.z,//r1
	                sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xx_yy_zz.y, sdfDerivatives_xy_yz_zx.y,//r2
	                sdfDerivatives_xy_yz_zx.z, sdfDerivatives_xy_yz_zx.y, sdfDerivatives_xx_yy_zz.z};
	sdfHessian.setValues(vals);
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPoint2ndDerivativeNeighborhoodWarp(THREADPTR(Vector3f)* warp_tData, //x8
                                                   const CONSTPTR(Vector3i)& voxelPosition,
                                                   const CONSTPTR(TVoxel)* voxelData,
                                                   const CONSTPTR(ITMHashEntry)* hashTable,
                                                   THREADPTR(TCache)& cache) {
	int vmIndex;
	Vector3i localBlockLocation;

	Vector3i(0, 0, 0);
	TVoxel voxel;
#define PROCESS_VOXEL(location, index)\
    localBlockLocation = voxelPosition + (location);\
    voxel = readVoxel(voxelData, hashTable, localBlockLocation, vmIndex, cache);\
    warp_tData[index] = voxel.warp_t;\

	//necessary for 2nd derivatives in same direction, i.e. xx and zz
	PROCESS_VOXEL(Vector3i(-1, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, -1, 0), 1);
	PROCESS_VOXEL(Vector3i(0, 0, -1), 2);

	//necessary for 1st derivatives
	PROCESS_VOXEL(Vector3i(0, 0, 1), 3);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 4);
	PROCESS_VOXEL(Vector3i(1, 0, 0), 5);

	//necessary for 2nd derivatives in varying directions, i.e. xy and yx
	PROCESS_VOXEL(Vector3i(1, 1, 0), 6);
	PROCESS_VOXEL(Vector3i(0, 1, 1), 7);//yz corner
	PROCESS_VOXEL(Vector3i(1, 0, 1), 8);


#undef PROCESS_VOXEL
}

template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpJacobianAndHessian(const CONSTPTR(Vector3f)& originalWarp_t,
                                                  const CONSTPTR(Vector3i)& originalPosition,
                                                  const CONSTPTR(TVoxel)* voxels,
                                                  const CONSTPTR(ITMHashEntry)* hashTable,
                                                  THREADPTR(TCache)& cache,
                                                  THREADPTR(Matrix3f)& jacobean,
                                                  THREADPTR(Matrix3f)* hessian //x3

) {
	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	Vector3f warp_tNeighbors[9];
	findPoint2ndDerivativeNeighborhoodWarp(warp_tNeighbors, //x8
	                                       originalPosition,
	                                       voxels,
	                                       hashTable,
	                                       cache);

	// |u_x, u_y, u_z|       |m00, m10, m20|
	// |v_x, v_y, v_z|       |m01, m11, m21|
	// |w_x, w_y, w_z|       |m02, m12, m22|
	jacobean.setColumn(0, warp_tNeighbors[3] - originalWarp_t);//1st derivative in x
	jacobean.setColumn(1, warp_tNeighbors[4] - originalWarp_t);//1st derivative in y
	jacobean.setColumn(2, warp_tNeighbors[5] - originalWarp_t);//1st derivative in z

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
	Matrix3f dd_XX_YY_ZZ = jacobean - backwardDifferences;

	Matrix3f neighborDifferences;
	neighborDifferences.setColumn(0, warp_tNeighbors[6] - warp_tNeighbors[4]);//(0,1,0)->(1,1,0)
	neighborDifferences.setColumn(1, warp_tNeighbors[7] - warp_tNeighbors[5]);//(0,0,1)->(0,1,1)
	neighborDifferences.setColumn(2, warp_tNeighbors[8] - warp_tNeighbors[3]);//(1,0,0)->(1,0,1)

#ifdef EXTRA_DEBUG //TODO: remove after optimizer starts working correctly
	Matrix3f neighborDifferences2;
	neighborDifferences2.setColumn(0, warp_tNeighbors[8] - warp_tNeighbors[5]);//(0,0,1)->(1,0,1)
	neighborDifferences2.setColumn(1, warp_tNeighbors[6] - warp_tNeighbors[3]);//(1,0,0)->(1,1,0)
	neighborDifferences2.setColumn(2, warp_tNeighbors[7] - warp_tNeighbors[4]);//(0,1,0)->(0,1,1)

#endif

	//second derivatives in different directions
	// |u_xy, u_yz, u_zx|      |m00, m10, m20|
	// |v_xy, v_yz, v_zx|      |m01, m11, m21|
	// |w_xy, w_yz, w_zx|      |m02, m12, m22|
	Matrix3f dd_XY_YZ_ZX = neighborDifferences - jacobean;

#ifdef EXTRA_DEBUG
	// |u_xz, u_yx, u_zy|      |m00, m10, m20|
	// |v_xz, v_yx, v_zy|      |m01, m11, m21|
	// |w_xz, w_yx, w_zy|      |m02, m12, m22|
	Matrix3f dd_XZ_YX_ZY = neighborDifferences2 - jacobean;

	Matrix3f dd_comp;
	dd_comp.setColumn(0, dd_XZ_YX_ZY.getColumn(1));
	dd_comp.setColumn(1, dd_XZ_YX_ZY.getColumn(2));
	dd_comp.setColumn(2, dd_XZ_YX_ZY.getColumn(0));

	if(dd_XY_YZ_ZX.getColumn(0) != dd_XZ_YX_ZY.getColumn(1) ||
			dd_XY_YZ_ZX.getColumn(1) != dd_XZ_YX_ZY.getColumn(2) ||
				dd_XY_YZ_ZX.getColumn(2) != dd_XZ_YX_ZY.getColumn(0)){
		std::cout << std::endl << dd_XY_YZ_ZX << std::endl << dd_comp << std::endl <<std::endl;
	}
#endif

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
