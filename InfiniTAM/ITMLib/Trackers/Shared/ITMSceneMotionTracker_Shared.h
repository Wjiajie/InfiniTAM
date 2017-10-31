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
inline float ComputeWarpedTrilinear(const CONSTPTR(Vector3i)& originalPosition,
                                    const CONSTPTR(Vector3f)& projectedPosition,
                                    const CONSTPTR(Vector3i)& positionShift,
                                    const CONSTPTR(TVoxel)* canonicalVoxels,
                                    const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                    THREADPTR(TCache)& cacheCanonical,
                                    const CONSTPTR(TVoxel)* liveVoxels,
                                    const CONSTPTR(ITMHashEntry)* liveHashTable,
                                    THREADPTR(TCache)& cacheLive,
                                    THREADPTR(Vector3f)& deltaEColorValue
) {
	int foundVoxel;
	Vector3i shiftedPosition = originalPosition + positionShift;
	Vector3f shiftedWarp =
			readVoxel(canonicalVoxels, canonicalHashTable, shiftedPosition, foundVoxel, cacheCanonical).warp_t;
	Vector3f shiftedProjectedPosition = Vector3f(
			positionShift.x ? shiftedPosition.x + shiftedWarp.x : projectedPosition.x,
			positionShift.y ? shiftedPosition.y + shiftedWarp.y : projectedPosition.y,
			positionShift.z ? shiftedPosition.z + shiftedWarp.z : projectedPosition.z);
	return interpolateTrilinearly(liveVoxels, liveHashTable, shiftedProjectedPosition, cacheLive,
	                              deltaEColorValue);
}


template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline float ComputeWarpedTrilinear(const CONSTPTR(Vector3i)& originalPosition,
                                    const CONSTPTR(Vector3f)& projectedPosition,
                                    const CONSTPTR(Vector3i)& positionShift,
                                    const CONSTPTR(TVoxel)* canonicalVoxels,
                                    const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                    THREADPTR(TCache)& canonicalCache,
                                    const CONSTPTR(TVoxel)* liveVoxels,
                                    const CONSTPTR(ITMHashEntry)* liveHashTable,
                                    THREADPTR(TCache)& liveCache
) {
	int foundVoxel;
	Vector3i shiftedPosition = originalPosition + positionShift;
	Vector3f shiftedWarp =
			readVoxel(canonicalVoxels, canonicalHashTable, shiftedPosition, foundVoxel, canonicalCache).warp_t;
	Vector3f shiftedProjectedPosition = Vector3f(
			positionShift.x ? shiftedPosition.x + shiftedWarp.x : projectedPosition.x,
			positionShift.y ? shiftedPosition.y + shiftedWarp.y : projectedPosition.y,
			positionShift.z ? shiftedPosition.z + shiftedWarp.z : projectedPosition.z);
	return interpolateTrilinearly(liveVoxels, liveHashTable, shiftedProjectedPosition, liveCache);
}

template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline float ComputeWarpedTrilinear(const CONSTPTR(Vector3i)& originalPosition,
                                    const CONSTPTR(Vector3f)& projectedPosition,
                                    const CONSTPTR(int)& direction,
                                    const CONSTPTR(int)& stepSize,
                                    const CONSTPTR(TVoxel)* canonicalVoxels,
                                    const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                    THREADPTR(TCache)& cacheCanonical,
                                    const CONSTPTR(TVoxel)* liveVoxels,
                                    const CONSTPTR(ITMHashEntry)* liveHashTable,
                                    THREADPTR(TCache)& cacheLive,
                                    THREADPTR(Vector3f)& deltaEColorValue
) {
	int foundVoxel;
	Vector3i shiftedPosition = Vector3i(originalPosition);
	shiftedPosition[direction] += stepSize;
	Vector3f shiftedWarp =
			readVoxel(canonicalVoxels, canonicalHashTable, shiftedPosition, foundVoxel, cacheCanonical).warp_t;
	Vector3f shiftedProjectedPosition(projectedPosition);
	shiftedProjectedPosition[direction] = shiftedPosition[direction] + shiftedWarp[direction];
	return interpolateTrilinearly(liveVoxels, liveHashTable, shiftedProjectedPosition, cacheLive,
	                              deltaEColorValue);
}

template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline float ComputeWarpedTrilinear(const CONSTPTR(Vector3i)& originalPosition,
                                    const CONSTPTR(Vector3f)& projectedPosition,
                                    const CONSTPTR(int)& direction,
                                    const CONSTPTR(int)& stepSize,
                                    const CONSTPTR(TVoxel)* canonicalVoxels,
                                    const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                    THREADPTR(TCache)& cacheCanonical,
                                    const CONSTPTR(TVoxel)* liveVoxels,
                                    const CONSTPTR(ITMHashEntry)* liveHashTable,
                                    THREADPTR(TCache)& cacheLive
) {
	int foundVoxel;
	Vector3i shiftedPosition = Vector3i(originalPosition);
	shiftedPosition[direction] += stepSize;
	Vector3f shiftedWarp =
			readVoxel(canonicalVoxels, canonicalHashTable, shiftedPosition, foundVoxel, cacheCanonical).warp_t;
	Vector3f shiftedProjectedPosition(projectedPosition);
	shiftedProjectedPosition[direction] = shiftedPosition[direction] + shiftedWarp[direction];
	return interpolateTrilinearly(liveVoxels, liveHashTable, shiftedProjectedPosition, cacheLive);
}

_CPU_AND_GPU_CODE_
inline float squareDistance(const CONSTPTR(Vector3f)& vec1,
                            const CONSTPTR(Vector3f)& vec2) {
	Vector3f difference = vec1 - vec2;
	return dot(difference, difference);
}

_CPU_AND_GPU_CODE_
inline Vector3f squareFiniteCentralDifferenceColor(const CONSTPTR(Vector3f)* forward, const CONSTPTR(Vector3f)& central,
                                                   const CONSTPTR(Vector3f)* backward) {
	Vector3f centralTimes2 = 2.0f * central;
	Vector3f difference0 = forward[0] - centralTimes2 + backward[0];
	Vector3f difference1 = forward[1] - centralTimes2 + backward[1];
	Vector3f difference2 = forward[2] - centralTimes2 + backward[2];
	return Vector3f(dot(difference0, difference0), dot(difference1, difference1), dot(difference2, difference2));
}

_CPU_AND_GPU_CODE_
inline Vector3f
squareFiniteForward2DifferenceColor(const CONSTPTR(Vector3f)& central, const CONSTPTR(Vector3f)* forward1,
                                    const CONSTPTR(Vector3f)* forward2) {
	Vector3f difference0 = forward2[0] - 2.0 * forward1[0] + central;
	Vector3f difference1 = forward2[1] - 2.0 * forward1[1] + central;
	Vector3f difference2 = forward2[2] - 2.0 * forward1[2] + central;
	return Vector3f(dot(difference0, difference0), dot(difference1, difference1), dot(difference2, difference2));
}

template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline Vector3f ComputeWarpedTrilinearStep(const CONSTPTR(Vector3i)& originalPosition,
                                           const CONSTPTR(Vector3f)& projectedPosition,
                                           const CONSTPTR(int)& stepSize,
                                           const CONSTPTR(TVoxel)* canonicalVoxels,
                                           const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                           THREADPTR(TCache)& cacheCanonical,
                                           const CONSTPTR(TVoxel)* liveVoxels,
                                           const CONSTPTR(ITMHashEntry)* liveHashTable,
                                           THREADPTR(TCache)& cacheLive,
                                           THREADPTR(Vector3f)* warpedColorStep
) {
	Vector3f warpedSdfStep;

	//used to compute finite difference in the x direction, i.e. delta_x(applyWarp(LiveSDF,Warp))
	warpedSdfStep.x = ComputeWarpedTrilinear<TVoxel, TIndex, TCache>(originalPosition, projectedPosition, 0, stepSize,
	                                                                 canonicalVoxels, canonicalHashTable,
	                                                                 cacheCanonical,
	                                                                 liveVoxels, liveHashTable, cacheLive,
	                                                                 warpedColorStep[0]);
	//used to compute finite difference y direction, i.e. for later computing delta_y(applyWarp(LiveSDF,Warp))
	warpedSdfStep.y = ComputeWarpedTrilinear<TVoxel, TIndex, TCache>(originalPosition, projectedPosition, 1, stepSize,
	                                                                 canonicalVoxels, canonicalHashTable,
	                                                                 cacheCanonical,
	                                                                 liveVoxels, liveHashTable, cacheLive,
	                                                                 warpedColorStep[1]);
	//compute gradient factor in the z direction, i.e. delta_z(applyWarp(LiveSDF,Warp))
	warpedSdfStep.z = ComputeWarpedTrilinear<TVoxel, TIndex, TCache>(originalPosition, projectedPosition, 2, stepSize,
	                                                                 canonicalVoxels, canonicalHashTable,
	                                                                 cacheCanonical,
	                                                                 liveVoxels, liveHashTable, cacheLive,
	                                                                 warpedColorStep[2]);
	return warpedSdfStep;
}

template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline Vector3f ComputeWarpedTrilinearStep(const CONSTPTR(Vector3i)& originalPosition,
                                           const CONSTPTR(Vector3f)& projectedPosition,
                                           const CONSTPTR(int)& stepSize,
                                           const CONSTPTR(TVoxel)* canonicalVoxels,
                                           const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                           THREADPTR(TCache)& cacheCanonical,
                                           const CONSTPTR(TVoxel)* liveVoxels,
                                           const CONSTPTR(ITMHashEntry)* liveHashTable,
                                           THREADPTR(TCache)& cacheLive
) {
	Vector3f warpedSdfStep;

	//used to compute finite difference in the x direction, i.e. delta_x(applyWarp(LiveSDF,Warp))
	warpedSdfStep.x = ComputeWarpedTrilinear<TVoxel, TIndex, TCache>(originalPosition, projectedPosition, 0, stepSize,
	                                                                 canonicalVoxels, canonicalHashTable,
	                                                                 cacheCanonical,
	                                                                 liveVoxels, liveHashTable, cacheLive);
	//used to compute finite difference y direction, i.e. for later computing delta_y(applyWarp(LiveSDF,Warp))
	warpedSdfStep.y = ComputeWarpedTrilinear<TVoxel, TIndex, TCache>(originalPosition, projectedPosition, 1, stepSize,
	                                                                 canonicalVoxels, canonicalHashTable,
	                                                                 cacheCanonical,
	                                                                 liveVoxels, liveHashTable, cacheLive);
	//compute gradient factor in the z direction, i.e. delta_z(applyWarp(LiveSDF,Warp))
	warpedSdfStep.z = ComputeWarpedTrilinear<TVoxel, TIndex, TCache>(originalPosition, projectedPosition, 2, stepSize,
	                                                                 canonicalVoxels, canonicalHashTable,
	                                                                 cacheCanonical,
	                                                                 liveVoxels, liveHashTable, cacheLive);
	return warpedSdfStep;
}

//_DEBUG
// this is the correct function
template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobianAlt2(const CONSTPTR(Vector3i)& originalPosition,
                                                  const CONSTPTR(Vector3f)& originalWarp_t,
                                                  const CONSTPTR(TVoxel)* canonicalVoxels,
                                                  const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                                  THREADPTR(TCache)& canonicalCache,
                                                  const CONSTPTR(TVoxel)* liveVoxels,
                                                  const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                  THREADPTR(TCache)& liveCache,
                                                  THREADPTR(float)& warpedSdf,
                                                  THREADPTR(Vector3f)& sdfJacobean){
	Vector3f projectedPosition = originalPosition.toFloat() + originalWarp_t;
	warpedSdf = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, liveCache);

	Vector3f warpedSdfForward;
	Vector3f positionShift = Vector3f(1, 0, 0);
	projectedPosition = originalPosition.toFloat() + originalWarp_t + positionShift;
	warpedSdfForward.x = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, liveCache);

	positionShift = Vector3f(0, 1, 0);
	projectedPosition = originalPosition.toFloat() + originalWarp_t + positionShift;
	warpedSdfForward.y = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, liveCache);

	positionShift = Vector3f(0, 0, 1);
	projectedPosition = originalPosition.toFloat() + originalWarp_t + positionShift;
	warpedSdfForward.z = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, liveCache);

	sdfJacobean = warpedSdfForward - Vector3f(warpedSdf);
};

//with color
template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputePerPointWarpedLiveJacobianAndHessian(const CONSTPTR(Vector3i)& originalPosition,
                                                            const CONSTPTR(Vector3f)& originalWarp_t,
                                                            const CONSTPTR(TVoxel)* canonicalVoxels,
                                                            const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                                            THREADPTR(TCache)& canonicalCache,
                                                            const CONSTPTR(TVoxel)* liveVoxels,
                                                            const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                            THREADPTR(TCache)& liveCache,
                                                            THREADPTR(float)& warpedSdf,
                                                            THREADPTR(Vector3f)& warpedColor,
                                                            THREADPTR(Vector3f)& sdfJacobean,
                                                            THREADPTR(Vector3f)& colorJacobean,
                                                            THREADPTR(Matrix3f)& sdfHessian) {

	//find value at direct location of voxel projected with the warp vector
	Vector3f projectedPosition = originalPosition.toFloat() + originalWarp_t;
	warpedSdf = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, liveCache,
	                                   warpedColor);

	//=== shifted warped sdf locations
	Vector3f warpedSdfForward;
	Vector3f warpedColorForward1[3];

	warpedSdfForward = ComputeWarpedTrilinearStep<TVoxel, TIndex, typename TIndex::IndexCache>(
			originalPosition, projectedPosition, 1,
			canonicalVoxels, canonicalHashTable, canonicalCache,
			liveVoxels, liveHashTable, liveCache,
			warpedColorForward1);
#define USE_CENTRAL_DIFFERENCE
#ifdef USE_CENTRAL_DIFFERENCE
	Vector3f warpedSdfBackward1;
	Vector3f warpedColorBackward1[3];

	warpedSdfBackward1 = ComputeWarpedTrilinearStep<TVoxel, TIndex, typename TIndex::IndexCache>(
			originalPosition, projectedPosition, -1, canonicalVoxels, canonicalHashTable,
			canonicalCache, liveVoxels, liveHashTable, liveCache, warpedColorBackward1);
#else
	Vector3f warpedSdfForward2;
						Vector3f warpedColorForward2[3];

						warpedSdfForward = ComputeWarpedTrilinearStep<TVoxel, TIndex, typename TIndex::IndexCache>(
								originalPosition, projectedPosition, 2,
								canonicalVoxels, canonicalHashTable, cacheCanonical,
								liveVoxels, liveHashTable, liveCahce,
								warpedColorForward2);
#endif

	Vector3f warpedSdfCorners;
	Vector3i positionShift;

	positionShift = Vector3i(1, 1, 0); //(x+1, y+1, z) corner
	warpedSdfCorners.x =
			ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(
					originalPosition, projectedPosition, positionShift,
					canonicalVoxels, canonicalHashTable, canonicalCache,
					liveVoxels, liveHashTable, liveCache);

	positionShift = Vector3i(0, 1, 1); //(x, y+1, z+1) corner
	warpedSdfCorners.y =
			ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(
					originalPosition, projectedPosition, positionShift,
					canonicalVoxels, canonicalHashTable, canonicalCache,
					liveVoxels, liveHashTable, liveCache);

	positionShift = Vector3i(1, 0, 1); //(x+1, y, z+1) corner
	warpedSdfCorners.z =
			ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(
					originalPosition, projectedPosition, positionShift,
					canonicalVoxels, canonicalHashTable, canonicalCache,
					liveVoxels, liveHashTable, liveCache);

	//===
	//=== approximation for gradients of warped live scene,
	//=== i.e. partial 1-st order derivatives in 3 directions
	//===
	sdfJacobean = warpedSdfForward - Vector3f(warpedSdf);
	colorJacobean = Vector3f(
			squareDistance(warpedColorForward1[0], warpedColor),
			squareDistance(warpedColorForward1[1], warpedColor),
			squareDistance(warpedColorForward1[2], warpedColor));

	//===
	//=== Approximation for Hessian of warped live scene, i.e. partial 2-nd order derivatives in 3 directions
	//===
	//let warped live scene be denoted as phi(psi)
	//first, compute [delta_xx(phi(psi)), delta_yy(phi(psi)), delta_zz(phi(psi))]^T
#ifdef USE_CENTRAL_DIFFERENCE
	Vector3f sdfDerivatives_xx_yy_zz =
			warpedSdfForward - Vector3f(2.0f * warpedSdf) + warpedSdfBackward1;
#else
	Vector3f sdfDerivatives_xx_yy_zz =
			warpedSdfForward2 - 2 * warpedSdfForward + Vector3f(warpedSdf);
	Vector3f clrDerivatives_xx_yy_zz = squareFiniteForward2DifferenceColor(
			warpedColor, warpedColorForward1, warpedColorForward2);
#endif
	//=== corner-voxel auxiliary derivatives for later 2nd derivative approximations
	//Along the x axis, case 1: (0,1,0)->(1,1,0)
	//Along the y axis, case 1: (0,0,1)->(0,1,1)
	//Along the z axis, case 1: (1,0,0)->(1,0,1)
	Vector3f cornerSdfDerivatives = Vector3f(
			warpedSdfCorners.x - warpedSdfForward.y,
			warpedSdfCorners.y - warpedSdfForward.z,
			warpedSdfCorners.z - warpedSdfForward.x);
	//Along the x axis, case 2: (0,0,1)->(1,0,1)
	//Along the y axis, case 2: (1,0,0)->(1,1,0)
	//Along the z axis, case 2: (0,1,0)->(0,1,1)
	Vector3f cornerSdfDerivatives2 = Vector3f(
			warpedSdfCorners.z - warpedSdfForward.x,
			warpedSdfCorners.x - warpedSdfForward.z,
			warpedSdfCorners.y - warpedSdfForward.y);


	//===Compute the 2nd partial derivatives for different direction sequences
	Vector3f sdfDerivatives_xy_yz_zx = cornerSdfDerivatives - sdfJacobean;
	Vector3f sdfDerivatives_xz_yx_zy = cornerSdfDerivatives2 - sdfJacobean;


	float vals[] = {sdfDerivatives_xx_yy_zz.x, sdfDerivatives_xz_yx_zy.y, sdfDerivatives_xy_yz_zx.z,//r1
	                sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xx_yy_zz.y, sdfDerivatives_xz_yx_zy.z,//r2
	                sdfDerivatives_xz_yx_zy.x, sdfDerivatives_xy_yz_zx.y, sdfDerivatives_xx_yy_zz.z};//

//	float vals[] = {sdfDerivatives_xx_yy_zz.x, sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xy_yz_zx.z,//r1
//	                sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xx_yy_zz.y, sdfDerivatives_xy_yz_zx.y,//r2
//	                sdfDerivatives_xy_yz_zx.z, sdfDerivatives_xy_yz_zx.y, sdfDerivatives_xx_yy_zz.z};
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
                                                            const CONSTPTR(TVoxel)* liveVoxels,
                                                            const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                            THREADPTR(TCache)& liveCache,
                                                            THREADPTR(float)& warpedSdf,
                                                            THREADPTR(Vector3f)& sdfJacobean,
                                                            THREADPTR(Matrix3f)& sdfHessian) {
	//find value at direct location of voxel projected with the warp vector
	Vector3f projectedPosition = originalPosition.toFloat() + originalWarp_t;
	warpedSdf = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, liveCache);

	//=== shifted warped sdf locations
	Vector3f warpedSdfForward;

	warpedSdfForward = ComputeWarpedTrilinearStep<TVoxel, TIndex, typename TIndex::IndexCache>(
			originalPosition, projectedPosition, 1,
			canonicalVoxels, canonicalHashTable, canonicalCache,
			liveVoxels, liveHashTable, liveCache);
#define USE_CENTRAL_DIFFERENCE
#ifdef USE_CENTRAL_DIFFERENCE
	Vector3f warpedSdfBackward1;

	warpedSdfBackward1 = ComputeWarpedTrilinearStep<TVoxel, TIndex, typename TIndex::IndexCache>(
			originalPosition, projectedPosition, -1, canonicalVoxels, canonicalHashTable,
			canonicalCache, liveVoxels, liveHashTable, liveCache);
#else
	Vector3f warpedSdfForward2;

	warpedSdfForward = ComputeWarpedTrilinearStep<TVoxel, TIndex, typename TIndex::IndexCache>(
			originalPosition, projectedPosition, 2,
			canonicalVoxels, canonicalHashTable, cacheCanonical,
			liveVoxels, liveHashTable, liveCahce);
#endif

	Vector3f warpedSdfCorners;
	Vector3i positionShift;

	positionShift = Vector3i(1, 1, 0); //(x+1, y+1, z) corner
	warpedSdfCorners.x =
			ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(
					originalPosition, projectedPosition, positionShift,
					canonicalVoxels, canonicalHashTable, canonicalCache,
					liveVoxels, liveHashTable, liveCache);

	positionShift = Vector3i(0, 1, 1); //(x, y+1, z+1) corner
	warpedSdfCorners.y =
			ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(
					originalPosition, projectedPosition, positionShift,
					canonicalVoxels, canonicalHashTable, canonicalCache,
					liveVoxels, liveHashTable, liveCache);

	positionShift = Vector3i(1, 0, 1); //(x+1, y, z+1) corner
	warpedSdfCorners.z =
			ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(
					originalPosition, projectedPosition, positionShift,
					canonicalVoxels, canonicalHashTable, canonicalCache,
					liveVoxels, liveHashTable, liveCache);

	//===
	//=== approximation for gradients of warped live scene,
	//=== i.e. partial 1-st order derivatives in 3 directions
	//===
	sdfJacobean = warpedSdfForward - Vector3f(warpedSdf);

	//===
	//=== Approximation for Hessian of warped live scene, i.e. partial 2-nd order derivatives in 3 directions
	//===
	//let warped live scene be denoted as phi(psi)
	//first, compute [delta_xx(phi(psi)), delta_yy(phi(psi)), delta_zz(phi(psi))]^T
#ifdef USE_CENTRAL_DIFFERENCE
	Vector3f sdfDerivatives_xx_yy_zz = warpedSdfForward - Vector3f(2.0f * warpedSdf) + warpedSdfBackward1;
#else
	Vector3f sdfDerivatives_xx_yy_zz = warpedSdfForward2 - 2 * warpedSdfForward + Vector3f(warpedSdf);
#endif
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
