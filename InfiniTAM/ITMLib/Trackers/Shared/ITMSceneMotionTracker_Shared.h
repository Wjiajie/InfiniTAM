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


template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float interpolateTrilinearly(const CONSTPTR(TVoxel)* voxelData,
                                    const CONSTPTR(ITMHashEntry)* hashIndex,
                                    const THREADPTR(Vector3f)& point,
                                    THREADPTR(TCache)& cache,
                                    THREADPTR(Vector3f)& color) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	Vector3f colorRes1, colorRes2, colorV1, colorV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);
	{
		const TVoxel& v = readVoxel(voxelData, hashIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
		sdfV1 = v.sdf;
		colorV1 = v.clr.toFloat();
	}
	{
		const TVoxel& v = readVoxel(voxelData, hashIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
		sdfV2 = v.sdf;
		colorV2 = v.clr.toFloat();
	}
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes1 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;

	{
		const TVoxel& v = readVoxel(voxelData, hashIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
		sdfV1 = v.sdf;
		colorV1 = v.clr.toFloat();
	}
	{
		const TVoxel& v = readVoxel(voxelData, hashIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
		sdfV2 = v.sdf;
		colorV2 = v.clr.toFloat();
	}
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes1 = (1.0f - coeff.y) * colorRes1 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);

	{
		const TVoxel& v = readVoxel(voxelData, hashIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
		sdfV1 = v.sdf;
		colorV1 = v.clr.toFloat();
	}
	{
		const TVoxel& v = readVoxel(voxelData, hashIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
		sdfV2 = v.sdf;
		colorV2 = v.clr.toFloat();
	}
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes2 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;

	{
		const TVoxel& v = readVoxel(voxelData, hashIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
		sdfV1 = v.sdf;
		colorV1 = v.clr.toFloat();
	}
	{
		const TVoxel& v = readVoxel(voxelData, hashIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
		sdfV2 = v.sdf;
		colorV2 = v.clr.toFloat();
	}
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes2 = (1.0f - coeff.y) * colorRes2 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	color = (1.0f - coeff.z) * colorRes1 + coeff.z * colorRes2;
	return TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
}

template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float interpolateTrilinearly(const CONSTPTR(TVoxel)* voxelData,
                                    const CONSTPTR(ITMHashEntry)* voxelHash,
                                    const THREADPTR(Vector3f)& point,
                                    THREADPTR(TCache)& cache) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);
	{
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + Vector3i(0, 0, 0), vmIndex, cache);
		sdfV1 = v.sdf;
	}
	{
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + Vector3i(1, 0, 0), vmIndex, cache);
		sdfV2 = v.sdf;
	}
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;

	{
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + Vector3i(0, 1, 0), vmIndex, cache);
		sdfV1 = v.sdf;
	}
	{
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + Vector3i(1, 1, 0), vmIndex, cache);
		sdfV2 = v.sdf;
	}
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);

	{
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + Vector3i(0, 0, 1), vmIndex, cache);
		sdfV1 = v.sdf;
	}
	{
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + Vector3i(1, 0, 1), vmIndex, cache);
		sdfV2 = v.sdf;
	}
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;

	{
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + Vector3i(0, 1, 1), vmIndex, cache);
		sdfV1 = v.sdf;
	}
	{
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + Vector3i(1, 1, 1), vmIndex, cache);
		sdfV2 = v.sdf;
	}
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	return TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
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
                                    THREADPTR(TCache)& cacheCanonical,
                                    const CONSTPTR(TVoxel)* liveVoxels,
                                    const CONSTPTR(ITMHashEntry)* liveHashTable,
                                    THREADPTR(TCache)& cacheLive
) {
	int foundVoxel;
	Vector3i shiftedPosition = originalPosition + positionShift;
	Vector3f shiftedWarp =
			readVoxel(canonicalVoxels, canonicalHashTable, shiftedPosition, foundVoxel, cacheCanonical).warp_t;
	Vector3f shiftedProjectedPosition = Vector3f(
			positionShift.x ? shiftedPosition.x + shiftedWarp.x : projectedPosition.x,
			positionShift.y ? shiftedPosition.y + shiftedWarp.y : projectedPosition.y,
			positionShift.z ? shiftedPosition.z + shiftedWarp.z : projectedPosition.z);
	return interpolateTrilinearly(liveVoxels, liveHashTable, shiftedProjectedPosition, cacheLive);
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
inline Vector3f squareFiniteForward2DifferenceColor(const CONSTPTR(Vector3f)& central, const CONSTPTR(Vector3f)* forward1,
                                                   const CONSTPTR(Vector3f)* forward2) {
	Vector3f difference0 = forward2[0] - 2.0*forward1[0] + central;
	Vector3f difference1 = forward2[1] - 2.0*forward1[1] + central;
	Vector3f difference2 = forward2[2] - 2.0*forward1[2] + central;
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
	warpedSdfStep.x = ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(originalPosition,
	                                                                                      projectedPosition,
	                                                                                      0,
	                                                                                      stepSize,
	                                                                                      canonicalVoxels,
	                                                                                      canonicalHashTable,
	                                                                                      cacheCanonical,
	                                                                                      liveVoxels, liveHashTable,
	                                                                                      cacheLive,
	                                                                                      warpedColorStep[0]);
	//used to compute finite difference y direction, i.e. for later computing delta_y(applyWarp(LiveSDF,Warp))
	warpedSdfStep.y = ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(originalPosition,
	                                                                                      projectedPosition,
	                                                                                      1,
	                                                                                      stepSize,
	                                                                                      canonicalVoxels,
	                                                                                      canonicalHashTable,
	                                                                                      cacheCanonical,
	                                                                                      liveVoxels, liveHashTable,
	                                                                                      cacheLive,
	                                                                                      warpedColorStep[1]);
	//compute gradient factor in the z direction, i.e. delta_z(applyWarp(LiveSDF,Warp))
	warpedSdfStep.z = ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(originalPosition,
	                                                                                      projectedPosition,
	                                                                                      2,
	                                                                                      stepSize,
	                                                                                      canonicalVoxels,
	                                                                                      canonicalHashTable,
	                                                                                      cacheCanonical,
	                                                                                      liveVoxels, liveHashTable,
	                                                                                      cacheLive,
	                                                                                      warpedColorStep[2]);
	return warpedSdfStep;
}

template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline bool ComputeWarpedTrilinear27Neighborhood(const CONSTPTR(Vector3i)& originalPosition,
                                                 const CONSTPTR(TVoxel)* canonicalVoxels,
                                                 const CONSTPTR(ITMHashEntry)* canonicalHashTable,
                                                 THREADPTR(TCache)& cacheCanonical,
                                                 const CONSTPTR(TVoxel)* liveVoxels,
                                                 const CONSTPTR(ITMHashEntry)* liveHashTable,
                                                 THREADPTR(TCache)& cacheLive,
                                                 THREADPTR(float)* warpedSdfValues, //x27
                                                 THREADPTR(Vector3f)* warpedColorValues //x27
) {
	int foundVoxel;
	TVoxel canonicalVoxel =
			readVoxel(canonicalVoxels, canonicalHashTable, originalPosition, foundVoxel,
			          cacheCanonical);
	if (!foundVoxel) {
		return false;
	}
	//find value at direct location of voxel projected with the warp vector
	Vector3f projectedPosition = originalPosition.toFloat() + canonicalVoxel.warp_t;

	//assume cubic kernel
	const int kernelRowSize = 3;
	const int kernelPlaneSize = kernelRowSize * kernelRowSize;
	const int kernelCenterShift = (kernelRowSize - 1) / 2;
	const int kernelStop = kernelCenterShift + 1;

	//find values at kernel locations
	for (int z = -kernelCenterShift, zIx = 0; z < kernelStop; z++, zIx += kernelPlaneSize) {
		for (int y = -kernelCenterShift, yIx = 0; y < kernelStop; y++, yIx += kernelRowSize) {
			for (int x = -kernelCenterShift; x < kernelStop; x++) {
				Vector3i positionShift(x, y, z);
				int linearIndex = zIx + yIx + x;
				warpedSdfValues[linearIndex] = ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(
						originalPosition, projectedPosition, positionShift,
						canonicalVoxels, canonicalHashTable, cacheCanonical,
						liveVoxels, liveHashTable, cacheLive,
						warpedColorValues[linearIndex]);
			}
		}
	}
	return true;
};

template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline void Compute3DGradient(
		THREADPTR(float)* warpedSdfValues, //x27
		THREADPTR(Vector3f)* warpedColorValues, //x27
		THREADPTR(Vector3f)& warpedSdfGradient,
		THREADPTR(Vector3f)* warpedColorGradient //x3
) {
	//apply 3D Sobel-Feldman operator

	//N.B. commented values left for clarity of code, please DO NOT ERASE
// @formatter:off
	float warpedSdfGradXandGradYShared = 2.0f * (warpedSdfValues[9] +
	                                             warpedSdfValues[11] +
	                                             warpedSdfValues[17] +
	                                             warpedSdfValues[15]) +

	                                     warpedSdfValues[0] +
	                                   /*warpedSdfValues[9]  * 2.0f +*/
	                                     warpedSdfValues[18] +

	                                     warpedSdfValues[2] +
	                                   /*warpedSdfValues[11] * 2.0f +*/
	                                     warpedSdfValues[20] +

	                                     warpedSdfValues[8] +
	                                   /*warpedSdfValues[17] * 2.0f +*/
	                                     warpedSdfValues[26] +

	                                     warpedSdfValues[6] +
	                                   /*warpedSdfValues[15] * 2.0f +*/
	                                     warpedSdfValues[24];

	float warpedSdfGradXandGradZShared = 2.0f * (warpedSdfValues[3] + warpedSdfValues[5] +
	                                             warpedSdfValues[21] + warpedSdfValues[23]);
	float warpedSdfGradYandGradZShared = 2.0f * (warpedSdfValues[1] + warpedSdfValues[19] +
	                                             warpedSdfValues[7] + warpedSdfValues[25]);

	warpedSdfGradient.x =
			  warpedSdfGradXandGradYShared + warpedSdfGradXandGradZShared +
			/*warpedSdfValues[0]          + warpedSdfValues[3] *2.0f    + warpedSdfGradient[6] +*/
			/*warpedSdfValues[9] *2.0f*/  + warpedSdfValues[12] * 4.0f/*+ warpedSdfGradient[15]*2.0f +*/
			/*warpedSdfValues[18]         + warpedSdfValues[21]*2.0f    + warpedSdfGradient[24] +*/

			/*warpedSdfValues[2]          + warpedSdfValues[5] *2.0f    + warpedSdfGradient[8] +*/
			/*warpedSdfValues[11]*2.0f*/  + warpedSdfValues[14] * 4.0f/*+ warpedSdfGradient[17]*2.0f +*/
		    /*warpedSdfValues[20]         + warpedSdfValues[23]*2.0f    + warpedSdfGradient[26]*/;

	warpedSdfGradient.y =
			  warpedSdfGradXandGradYShared + warpedSdfGradYandGradZShared +
			/*warpedSdfValues[0]          + warpedSdfValues[1] *2.0f    + warpedSdfValues[2]*/
			/*warpedSdfValues[9] *2.0f*/  + warpedSdfValues[10] * 4.0f/*+ warpedSdfValues[11]*2.0f*/
			/*warpedSdfValues[18]         + warpedSdfValues[19]*2.0f    + warpedSdfValues[20]*/

			/*warpedSdfGradient[6]        + warpedSdfValues[7] *2.0f    + warpedSdfGradient[8] +*/
			/*warpedSdfGradient[15]*2.0f*/+ warpedSdfValues[16] * 4.0f/*+ warpedSdfGradient[17]*2.0f +*/
		    /*warpedSdfGradient[24]       + warpedSdfValues[25]*2.0f    + warpedSdfGradient[26]*/;
	warpedSdfGradient.z =
			  warpedSdfGradXandGradZShared + warpedSdfGradYandGradZShared +
			  warpedSdfValues[0]         /*+ warpedSdfValues[1] *2.0f*/  + warpedSdfGradient[2] +
			/*warpedSdfValues[3] *2.0f  */ + warpedSdfValues[4] * 4.0f  +/*warpedSdfGradient[5]*2.0f +*/
			  warpedSdfValues[6]         /*+ warpedSdfValues[7] *2.0f*/  + warpedSdfGradient[8] +

			  warpedSdfValues[18]        /*+ warpedSdfValues[19]*2.0f*/  + warpedSdfGradient[20]
			/*warpedSdfValues[21]*2.0f  +*/+ warpedSdfValues[22] * 4.0f +/*warpedSdfGradient[23]*2.0f +*/
			  warpedSdfValues[24]        /*+ warpedSdfValues[25]*2.0f*/  + warpedSdfGradient[26];

// @formatter:on
};