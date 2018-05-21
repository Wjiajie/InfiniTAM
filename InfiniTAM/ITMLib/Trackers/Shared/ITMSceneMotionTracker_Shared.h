//  ================================================================
//  Created by Gregory Kramida on 4/26/18.
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

#include "../../Utils/ITMVoxelFlags.h"
#include "../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../ORUtils/PlatformIndependence.h"
#include "../../Utils/ITMPrintHelpers.h"


// region =================================EXPLORATION OF NEIGHBORHOOD AROUND CANONICAL VOXEL===========================



/**
 * \brief Finds neighbor voxel's warps in the order specified below.
 *     0        1        2          3         4         5           6         7         8
 *	(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
 * \tparam TVoxel
 * \tparam TCache
 * \param[out] neighborWarps
 * \param[out] neighborKnown - current behavior is:
 * 1) record unallocated voxels as non-found
 * 2) truncated voxels marked unknown or known as found
 * 3) everything else (non-truncated), of course, as found
 * \param[in] voxelPosition exact position of voxel in the scene.
 * \param[in] voxels
 * \param[in] hashEntries
 * \param[in] cache
 */
template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPoint2ndDerivativeNeighborhoodWarp(THREADPTR(Vector3f)* neighborWarps, //x9, out
                                                   THREADPTR(bool)* neighborKnown, //x9, out
                                                   THREADPTR(bool)* neighborTruncated, //x9, out
                                                   THREADPTR(bool)* neighborAllocated, //x9, out
                                                   const CONSTPTR(Vector3i)& voxelPosition,
                                                   const CONSTPTR(TVoxel)* voxels,
                                                   const CONSTPTR(ITMHashEntry)* hashEntries,
                                                   THREADPTR(TCache)& cache) {
	int vmIndex;

	TVoxel voxel;
	//TODO: define inline function instead of macro
#define PROCESS_VOXEL(location, index)\
    voxel = readVoxel(voxels, hashEntries, voxelPosition + (location), vmIndex, cache);\
    neighborWarps[index] = voxel.warp;\
    neighborAllocated[index] = vmIndex != 0;\
    neighborKnown[index] = voxel.flags != ITMLib::VOXEL_UNKNOWN;\
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

//endregion




//region ================================= SDF JACOBIAN ================================================================

template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobianForwardDifferences(Vector3f& jacobian,
                                           const Vector3i& position,
                                           const float& sdfAtPosition,
                                           const TVoxel* voxels,
                                           const ITMHashEntry* hashEntries,
                                           TCache cache) {
	int vmIndex;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, hashEntries, position + (offset), vmIndex, cache).sdf))
	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));
#undef sdf_at

	jacobian[0] = sdfAtXplusOne - sdfAtPosition;
	jacobian[1] = sdfAtYplusOne - sdfAtPosition;
	jacobian[2] = sdfAtZplusOne - sdfAtPosition;
};

template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_ForwardDifferences(Vector3f& jacobian,
                                            const Vector3i& position,
                                            const TVoxel* voxels,
                                            const ITMHashEntry* hashEntries,
                                            TCache cache) {
	int vmIndex;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, hashEntries, position + (offset), vmIndex, cache).sdf))
	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));

	float sdfAtPosition = sdf_at(Vector3i(0, 0, 0));
#undef sdf_at
	jacobian[0] = sdfAtXplusOne - sdfAtPosition;
	jacobian[1] = sdfAtYplusOne - sdfAtPosition;
	jacobian[2] = sdfAtZplusOne - sdfAtPosition;
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences(Vector3f& jacobian,
                                            const Vector3i& voxelPosition,
                                            const TVoxel* voxels,
                                            const ITMHashEntry* hashEntries,
                                            TCache cache) {
	int vmIndex;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, hashEntries, voxelPosition + (offset), vmIndex, cache).sdf))

	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));
	float sdfAtXminusOne = sdf_at(Vector3i(-1, 0, 0));
	float sdfAtYminusOne = sdf_at(Vector3i(0, -1, 0));
	float sdfAtZminusOne = sdf_at(Vector3i(0, 0, -1));

#undef sdf_at
	jacobian[0] = 0.5f * (sdfAtXplusOne - sdfAtXminusOne);
	jacobian[1] = 0.5f * (sdfAtYplusOne - sdfAtYminusOne);
	jacobian[2] = 0.5f * (sdfAtZplusOne - sdfAtZminusOne);
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_IndexedFields(
		Vector3f& jacobian,
		const Vector3i& voxelPosition,
		const TVoxel* voxels,
		const ITMHashEntry* hashEntries,
		THREADPTR(TCache) cache,
		int fieldIndex) {
	int vmIndex;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, hashEntries, voxelPosition + (offset), vmIndex, cache).sdf_values[fieldIndex]))

	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));
	float sdfAtXminusOne = sdf_at(Vector3i(-1, 0, 0));
	float sdfAtYminusOne = sdf_at(Vector3i(0, -1, 0));
	float sdfAtZminusOne = sdf_at(Vector3i(0, 0, -1));

#undef sdf_at
	jacobian[0] = 0.5f * (sdfAtXplusOne - sdfAtXminusOne);
	jacobian[1] = 0.5f * (sdfAtYplusOne - sdfAtYminusOne);
	jacobian[2] = 0.5f * (sdfAtZplusOne - sdfAtZminusOne);
};
// endregion

// region ================================= SDF HESSIAN ================================================================

template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeSdfHessian(THREADPTR(Matrix3f)& hessian,
                              const CONSTPTR(Vector3i &) position,
                              const CONSTPTR(float)& sdfAtPosition,
		//const CONSTPTR(Vector3f&) jacobianAtPosition,
		                      const CONSTPTR(TVoxel)* voxels,
		                      const CONSTPTR(ITMHashEntry)* hashEntries,
		                      THREADPTR(TCache) cache) {
	int vmIndex;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, hashEntries, position + (offset), vmIndex, cache).sdf))
	//for xx, yy, zz
	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));
	float sdfAtXminusOne = sdf_at(Vector3i(-1, 0, 0));
	float sdfAtYminusOne = sdf_at(Vector3i(0, -1, 0));
	float sdfAtZminusOne = sdf_at(Vector3i(0, 0, -1));

	//for xy, xz, yz
	float sdfAtXplusOneYplusOne = sdf_at(Vector3i(1, 1, 0));
	float sdfAtXminusOneYminusOne = sdf_at(Vector3i(-1, -1, 0));
	float sdfAtYplusOneZplusOne = sdf_at(Vector3i(0, 1, 1));
	float sdfAtYminusOneZminusOne = sdf_at(Vector3i(0, -1, -1));
	float sdfAtXplusOneZplusOne = sdf_at(Vector3i(1, 0, 1));
	float sdfAtXminusOneZminusOne = sdf_at(Vector3i(-1, 0, -1));
#undef sdf_at
	float delta_xx = sdfAtXplusOne - 2 * sdfAtPosition + sdfAtXminusOne;
	float delta_yy = sdfAtYplusOne - 2 * sdfAtPosition + sdfAtYminusOne;
	float delta_zz = sdfAtZplusOne - 2 * sdfAtPosition + sdfAtZminusOne;

	float delta_xy = 0.5f * (sdfAtXplusOneYplusOne - sdfAtXplusOne - sdfAtYplusOne
	                         + 2 * sdfAtPosition
	                         - sdfAtXminusOne - sdfAtYminusOne + sdfAtXminusOneYminusOne);

	float delta_yz = 0.5f * (sdfAtYplusOneZplusOne - sdfAtYplusOne - sdfAtZplusOne
	                         + 2 * sdfAtPosition
	                         - sdfAtYminusOne - sdfAtZminusOne + sdfAtYminusOneZminusOne);

	float delta_xz = 0.5f * (sdfAtXplusOneZplusOne - sdfAtXplusOne - sdfAtZplusOne
	                         + 2 * sdfAtPosition
	                         - sdfAtXminusOne - sdfAtZminusOne + sdfAtXminusOneZminusOne);

	float vals[9] = {delta_xx, delta_xy, delta_xz,
	                 delta_xy, delta_yy, delta_yz,
	                 delta_yz, delta_yz, delta_zz};

	hessian.setValues(vals);
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeSdfHessian_IndexedFields(THREADPTR(Matrix3f)& hessian,
                              const CONSTPTR(Vector3i &) position,
                              const CONSTPTR(float)& sdfAtPosition,
		//const CONSTPTR(Vector3f&) jacobianAtPosition,
		                      const CONSTPTR(TVoxel)* voxels,
		                      const CONSTPTR(ITMHashEntry)* hashEntries,
		                      THREADPTR(TCache) cache,
		                                    int fieldIndex) {
	int vmIndex;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, hashEntries, position + (offset), vmIndex, cache).sdf_values[fieldIndex]))
	//for xx, yy, zz
	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));
	float sdfAtXminusOne = sdf_at(Vector3i(-1, 0, 0));
	float sdfAtYminusOne = sdf_at(Vector3i(0, -1, 0));
	float sdfAtZminusOne = sdf_at(Vector3i(0, 0, -1));

	//for xy, xz, yz
	float sdfAtXplusOneYplusOne = sdf_at(Vector3i(1, 1, 0));
	float sdfAtXminusOneYminusOne = sdf_at(Vector3i(-1, -1, 0));
	float sdfAtYplusOneZplusOne = sdf_at(Vector3i(0, 1, 1));
	float sdfAtYminusOneZminusOne = sdf_at(Vector3i(0, -1, -1));
	float sdfAtXplusOneZplusOne = sdf_at(Vector3i(1, 0, 1));
	float sdfAtXminusOneZminusOne = sdf_at(Vector3i(-1, 0, -1));
#undef sdf_at
	float delta_xx = sdfAtXplusOne - 2 * sdfAtPosition + sdfAtXminusOne;
	float delta_yy = sdfAtYplusOne - 2 * sdfAtPosition + sdfAtYminusOne;
	float delta_zz = sdfAtZplusOne - 2 * sdfAtPosition + sdfAtZminusOne;

	float delta_xy = 0.5f * (sdfAtXplusOneYplusOne - sdfAtXplusOne - sdfAtYplusOne
	                         + 2 * sdfAtPosition
	                         - sdfAtXminusOne - sdfAtYminusOne + sdfAtXminusOneYminusOne);

	float delta_yz = 0.5f * (sdfAtYplusOneZplusOne - sdfAtYplusOne - sdfAtZplusOne
	                         + 2 * sdfAtPosition
	                         - sdfAtYminusOne - sdfAtZminusOne + sdfAtYminusOneZminusOne);

	float delta_xz = 0.5f * (sdfAtXplusOneZplusOne - sdfAtXplusOne - sdfAtZplusOne
	                         + 2 * sdfAtPosition
	                         - sdfAtXminusOne - sdfAtZminusOne + sdfAtXminusOneZminusOne);

	float vals[9] = {delta_xx, delta_xy, delta_xz,
	                 delta_xy, delta_yy, delta_yz,
	                 delta_yz, delta_yz, delta_zz};

	hessian.setValues(vals);
};


//endregion

// region ================================ WARP LAPLACIAN (SMOOTHING/TIKHONOV TERM) ====================================

inline void ComputeWarpLaplacian(THREADPTR(Vector3f)& laplacian,
                                 const CONSTPTR(Vector3f)& voxelWarp,
                                 const CONSTPTR(Vector3f*) neighborWarps) {//in, x6-9
	//    0        1        2          3         4         5
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)
	laplacian = Vector3f(0.0f);
	//3D discrete Laplacian filter based on https://en.wikipedia.org/wiki/Discrete_Laplace_operator
	for (int iSixConnectedNeighbor = 0; iSixConnectedNeighbor < 6; iSixConnectedNeighbor++) {
		laplacian += neighborWarps[iSixConnectedNeighbor];
	}
	laplacian -= 6 * voxelWarp;
}

inline void ComputeWarpLaplacianAndJacobian(THREADPTR(Vector3f)& laplacian,
                                            THREADPTR(Matrix3f)& jacobian,
                                            const CONSTPTR(Vector3f)& voxelWarp,
                                            const CONSTPTR(Vector3f*) neighborWarps) {//in, x6-9
	//    0        1        2          3         4         5
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)
	laplacian = Vector3f(0.0f);
	//3D discrete Laplacian filter based on https://en.wikipedia.org/wiki/Discrete_Laplace_operator
	for (int iSixConnectedNeighbor = 0; iSixConnectedNeighbor < 6; iSixConnectedNeighbor++) {
		laplacian += neighborWarps[iSixConnectedNeighbor];
	}
	laplacian -= 6 * voxelWarp;
	//use first-order differences for jacobian
	jacobian.setColumn(0, 0.5 * (neighborWarps[3] - neighborWarps[0]));//1st derivative in x
	jacobian.setColumn(1, 0.5 * (neighborWarps[4] - neighborWarps[1]));//1st derivative in y
	jacobian.setColumn(2, 0.5 * (neighborWarps[5] - neighborWarps[2]));//1st derivative in z
}

// endregion

// region =================================== WARP JACOBIAN AND HESSIAN (SMOOTHING/KILLING TERM) =======================
//Computes the jacobian and hessian approximation for the warp vectors themselves in a given neighborhood
_CPU_AND_GPU_CODE_
inline void ComputePerVoxelWarpJacobianAndHessian(const CONSTPTR(Vector3f)& voxelWarp,
                                                  const CONSTPTR(Vector3f*) neighborWarps, //in, x9
                                                  THREADPTR(Matrix3f)& jacobian, //out
                                                  THREADPTR(Matrix3f)* hessian //out, x3
) {
	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)

	// |u_x, u_y, u_z|       |m00, m10, m20|
	// |v_x, v_y, v_z|       |m01, m11, m21|
	// |w_x, w_y, w_z|       |m02, m12, m22|
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
// endregion
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void GetVoxel(THREADPTR(TVoxel)*& voxel, THREADPTR(TVoxel) *voxelData,
const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) *voxelIndex, const THREADPTR(Vector3i) & point, THREADPTR(ITMLib::ITMVoxelBlockHash::IndexCache) & cache){
	Vector3i blockPos;
	int linearIdx = pointToVoxelBlockPos(point, blockPos);
	if IS_EQUAL3(blockPos, cache.blockPos)
	{
		voxel = &voxelData[cache.blockPtr + linearIdx];
		return;
	}

	int hashIdx = hashIndex(blockPos);

	while (true)
	{
		ITMHashEntry hashEntry = voxelIndex[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0)
		{
			cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;

			voxel = &voxelData[cache.blockPtr + linearIdx];
			return;
		}

		if (hashEntry.offset < 1) break;
		hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
	}
	voxel = nullptr;
}

