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

//pick maximum weights, get confidence
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float interpolateTrilinearly(const CONSTPTR(TVoxel)* voxelData,
                                    const CONSTPTR(ITMHashEntry)* hashIndex,
                                    const THREADPTR(Vector3f)& point,
                                    THREADPTR(TCache)& cache,
                                    THREADPTR(Vector3f)& color,
                                    THREADPTR(int)& wDepth,
                                    THREADPTR(int)& wColor,
                                    THREADPTR(float)& confidence) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	Vector3f colorRes1, colorRes2, colorV1, colorV2;
	float confRes1, confRes2, confV1, confV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);

#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, hashIndex, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.sdf;\
        colorV##suffix = TO_FLOAT3(v.clr);\
		confV##suffix = v.confidence;\
		if (v.w_depth > wDepth) wDepth = v.w_depth;\
		if (v.w_color > wColor) wColor = v.w_color;\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes1 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	confRes1 = (1.0f - coeff.x) * confV1 + coeff.x * confV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes1 = (1.0f - coeff.y) * colorRes1 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	confRes1 = (1.0f - coeff.y) * confRes1 + coeff.y * ((1.0f - coeff.x) * confV1 + coeff.x * confV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes2 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	confRes2 = (1.0f - coeff.x) * confV1 + coeff.x * confV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes2 = (1.0f - coeff.y) * colorRes2 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	confRes2 = (1.0f - coeff.y) * confRes2 + coeff.y * ((1.0f - coeff.x) * confV1 + coeff.x * confV2);
	color = ((1.0f - coeff.z) * colorRes1 + coeff.z * colorRes2) / 255.0f;
	confidence = (1.0f - coeff.z) * confRes1 + coeff.z * confRes2;
#undef PROCESS_VOXEL
	return TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
}

//pick maximum weights, get confidence
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float interpolateTrilinearly(const CONSTPTR(TVoxel)* voxelData,
                                    const CONSTPTR(ITMHashEntry)* hashIndex,
                                    const THREADPTR(Vector3f)& point,
                                    THREADPTR(TCache)& cache,
                                    THREADPTR(Vector3f)& color,
                                    THREADPTR(int)& wDepth,
                                    THREADPTR(int)& wColor) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	Vector3f colorRes1, colorRes2, colorV1, colorV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);

#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, hashIndex, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.sdf;\
        colorV##suffix = TO_FLOAT3(v.clr);\
		if (v.w_depth > wDepth) wDepth = v.w_depth;\
		if (v.w_color > wColor) wColor = v.w_color;\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes1 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes1 = (1.0f - coeff.y) * colorRes1 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes2 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes2 = (1.0f - coeff.y) * colorRes2 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	color = ((1.0f - coeff.z) * colorRes1 + coeff.z * colorRes2) / 255.0f;

#undef PROCESS_VOXEL
	return TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
}

//sdf, color, and interpolated weights
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float interpolateTrilinearly(const CONSTPTR(TVoxel)* voxelData,
                                    const CONSTPTR(ITMHashEntry)* hashIndex,
                                    const THREADPTR(Vector3f)& point,
                                    THREADPTR(TCache)& cache,
                                    THREADPTR(Vector3f)& color,
									THREADPTR(float)& wDepth,
									THREADPTR(float)& wColor) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	float wDepthRes1, wDepthRes2, wDepthV1, wDepthV2;
	float wColorRes1, wColorRes2, wColorV1, wColorV2;
	Vector3f colorRes1, colorRes2, colorV1, colorV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);

#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, hashIndex, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.sdf;\
        colorV##suffix = TO_FLOAT3(v.clr);\
		wDepthV##suffix = v.w_depth;\
		wColorV##suffix = v.w_color;\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes1 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	wDepthRes1 = (1.0f - coeff.x) * wDepthV1 + coeff.x * wDepthV2;
	wColorRes1 = (1.0f - coeff.x) * wColorV1 + coeff.x * wColorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes1 = (1.0f - coeff.y) * colorRes1 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	wDepthRes1 = (1.0f - coeff.y) * wDepthRes1 + coeff.y * ((1.0f - coeff.x) * wDepthV1 + coeff.x * wDepthV2);
	wColorRes1 = (1.0f - coeff.y) * wColorRes1 + coeff.y * ((1.0f - coeff.x) * wColorV1 + coeff.x * wColorV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes2 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	wDepthRes2 = (1.0f - coeff.x) * wDepthV1 + coeff.x * wDepthV2;
	wColorRes2 = (1.0f - coeff.x) * wColorV1 + coeff.x * wColorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes2 = (1.0f - coeff.y) * colorRes2 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	wDepthRes2 = (1.0f - coeff.y) * wDepthRes2 + coeff.y * ((1.0f - coeff.x) * wDepthV1 + coeff.x * wDepthV2);
	wColorRes2 = (1.0f - coeff.y) * wColorRes2 + coeff.y * ((1.0f - coeff.x) * wColorV1 + coeff.x * wColorV2);
	
	color = ((1.0f - coeff.z) * colorRes1 + coeff.z * colorRes2) / 255.0f;
	wDepth = (1.0f - coeff.z) * wDepthRes1 + coeff.z * wDepthRes2;
	wColor = (1.0f - coeff.z) * wColorRes1 + coeff.z * wColorRes2;
#undef PROCESS_VOXEL
	return TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
}

//sdf and color
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
#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, hashIndex, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.sdf;\
        colorV##suffix = TO_FLOAT3(v.clr);\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes1 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes1 = (1.0f - coeff.y) * colorRes1 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes2 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))

	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes2 = (1.0f - coeff.y) * colorRes2 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	color = ((1.0f - coeff.z) * colorRes1 + coeff.z * colorRes2) / 255.0f;
#undef PROCESS_VOXEL
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
#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.sdf;\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))

	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
#undef PROCESS_VOXEL
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

//with color
template<typename TVoxel, typename TIndex, typename TCache>
_CPU_AND_GPU_CODE_
inline Vector3f ComputePerPointWarpedLiveJacobianAndHessian(const CONSTPTR(Vector3i)& originalPosition,
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
inline Vector3f ComputePerPointWarpedLiveJacobianAndHessian(const CONSTPTR(Vector3i)& originalPosition,
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
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	Vector3f warp_tNeighbors[8];
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
	backwardDifferences.setColumn(0, warp_tNeighbors[0] - originalWarp_t);//1st derivative in x
	backwardDifferences.setColumn(0, warp_tNeighbors[1] - originalWarp_t);//1st derivative in y
	backwardDifferences.setColumn(0, warp_tNeighbors[2] - originalWarp_t);//1st derivative in z

	//second derivatives in same direction
	// |u_xx, u_yy, u_zz|       |m00, m10, m20|
	// |v_xx, v_yy, v_zz|       |m01, m11, m21|
	// |w_xx, w_yy, w_zz|       |m02, m12, m22|
	Matrix3f dd_XX_YY_ZZ = jacobean - backwardDifferences;

	Matrix3f neighborDifferences;
	neighborDifferences.setColumn(0, warp_tNeighbors[6] - warp_tNeighbors[4]);//(0,1,0)->(1,1,0)
	neighborDifferences.setColumn(0, warp_tNeighbors[7] - warp_tNeighbors[5]);//(0,0,1)->(0,1,1)
	neighborDifferences.setColumn(0, warp_tNeighbors[8] - warp_tNeighbors[3]);//(1,0,0)->(1,0,1)

	//second derivatives in different directions
	// |u_xy, u_yz, u_zx|      |m00, m10, m20|
	// |v_xy, v_yz, v_zx|      |m01, m11, m21|
	// |w_xy, w_yz, w_zx|      |m02, m12, m22|
	Matrix3f dd_XY_YZ_ZX = neighborDifferences - jacobean;

	// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_zx|
	// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
	// |2, 5, 8|     |m02, m12, m22|      |u_zx, u_yz, u_zz|
	float valsU[] = {dd_XX_YY_ZZ.m00, dd_XY_YZ_ZX.m00, dd_XY_YZ_ZX.m20,
	                 dd_XY_YZ_ZX.m00, dd_XX_YY_ZZ.m10, dd_XY_YZ_ZX.m10,
	                 dd_XY_YZ_ZX.m20, dd_XY_YZ_ZX.m10, dd_XX_YY_ZZ.m20};
	hessian[0].setValues(valsU);

	// |0, 3, 6|     |m00, m10, m20|      |v_xx, v_xy, v_zx|
	// |1, 4, 7|     |m01, m11, m21|      |v_xy, v_yy, v_yz|
	// |2, 5, 8|     |m02, m12, m22|      |v_zx, v_yz, v_zz|
	float valsV[] = {dd_XX_YY_ZZ.m01, dd_XY_YZ_ZX.m01, dd_XY_YZ_ZX.m21,
	                 dd_XY_YZ_ZX.m01, dd_XX_YY_ZZ.m11, dd_XY_YZ_ZX.m11,
	                 dd_XY_YZ_ZX.m21, dd_XY_YZ_ZX.m11, dd_XX_YY_ZZ.m21};
	hessian[1].setValues(valsV);

	// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_zx|
	// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
	// |2, 5, 8|     |m02, m12, m22|      |u_zx, u_yz, u_zz|
	float valsW[] = {dd_XX_YY_ZZ.m02, dd_XY_YZ_ZX.m02, dd_XY_YZ_ZX.m22,
	                 dd_XY_YZ_ZX.m02, dd_XX_YY_ZZ.m12, dd_XY_YZ_ZX.m12,
	                 dd_XY_YZ_ZX.m22, dd_XY_YZ_ZX.m12, dd_XX_YY_ZZ.m22};
	hessian[2].setValues(valsW);

};
