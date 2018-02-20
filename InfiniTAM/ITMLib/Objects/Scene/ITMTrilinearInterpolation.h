//  ================================================================
//  Created by Gregory Kramida on 10/27/17.
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

//_DEBUG -- special treatment of truncated values, with semantic information only!
//sdf, color, pick maximum weights, get confidence
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float interpolateTrilinearly_Corrected2(const CONSTPTR(TVoxel)* voxelData,
                                               const CONSTPTR(ITMHashEntry)* voxelHash,
                                               const THREADPTR(Vector3f)& point,
                                               THREADPTR(TCache)& cache,
                                               THREADPTR(Vector3f)& color,
                                               THREADPTR(float)& confidence,
                                               THREADPTR(bool)& struckNarrowBand) {
	float sdfRes1, sdfRes2;
	float confRes1, confRes2;
	Vector3f clrRes1, clrRes2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;

	TO_INT_FLOOR3(pos, coeff, point);
	const int neighborCount = 8;
	float sdfs[neighborCount];
	Vector3f colors[neighborCount];
	float confs[neighborCount];
	bool found[neighborCount] = {0};
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float sumFound = 0.0f;
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		sdfs[iNeighbor] = TVoxel::valueToFloat(v.sdf);
		confs[iNeighbor] = v.confidence;
		colors[iNeighbor] = TO_FLOAT3(v.clr);
		bool foundCur = vmIndex != 0 && v.flags != ITMLib::VOXEL_TRUNCATED;
		struckNarrowBand |= foundCur;
		found[iNeighbor] = foundCur;
		sumFound += foundCur * v.sdf;
	}
	if (!struckNarrowBand) {
		return 0.0f;
	}

	float truncated = std::copysign(1.0f, sumFound);
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		if (!found[iNeighbor]) {
			sdfs[iNeighbor] = truncated;
		}
	}


	float oneMinusCoeffX = 1.0f - coeff.x;
	float oneMinusCoeffY = 1.0f - coeff.y;

	sdfRes1 = oneMinusCoeffY * (oneMinusCoeffX * sdfs[0] + coeff.x * sdfs[1])
	          + coeff.y * (oneMinusCoeffX * sdfs[2] + coeff.x * sdfs[3]);
	sdfRes2 = oneMinusCoeffY * (oneMinusCoeffX * sdfs[4] + coeff.x * sdfs[5])
	          + coeff.y * (oneMinusCoeffX * sdfs[6] + coeff.x * sdfs[7]);
	confRes1 = oneMinusCoeffY * (oneMinusCoeffX * confs[0] + coeff.x * confs[1])
	           + coeff.y * (oneMinusCoeffX * confs[2] + coeff.x * confs[3]);
	confRes2 = oneMinusCoeffY * (oneMinusCoeffX * confs[4] + coeff.x * confs[5])
	           + coeff.y * (oneMinusCoeffX * confs[6] + coeff.x * confs[7]);
	clrRes1 = oneMinusCoeffY * (oneMinusCoeffX * colors[0] + coeff.x * colors[1])
	          + coeff.y * (oneMinusCoeffX * colors[2] + coeff.x * colors[3]);
	clrRes2 = oneMinusCoeffY * (oneMinusCoeffX * colors[4] + coeff.x * colors[5])
	          + coeff.y * (oneMinusCoeffX * colors[6] + coeff.x * colors[7]);

	float sdf = (1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2;
	confidence = (1.0f - coeff.z) * confRes1 + coeff.z * confRes2;
	color = (1.0f - coeff.z) * clrRes1 + coeff.z * clrRes2;
	return sdf;
}


//_DEBUG -- special treatment of truncated values
//sdf, color, pick maximum weights, get confidence
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float interpolateTrilinearly_Corrected(const CONSTPTR(TVoxel)* voxelData,
                                              const CONSTPTR(ITMHashEntry)* voxelHash,
                                              const THREADPTR(Vector3f)& point,
                                              THREADPTR(TCache)& cache,
                                              THREADPTR(Vector3f)& color,
                                              THREADPTR(float)& confidence,
                                              THREADPTR(bool)& struckNarrowBand) {
	float sdfRes1, sdfRes2;
	float confRes1, confRes2;
	Vector3f clrRes1, clrRes2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;

	TO_INT_FLOOR3(pos, coeff, point);
	const int neighborCount = 8;
	float sdfs[neighborCount];
	Vector3f colors[neighborCount];
	float confs[neighborCount];
	bool found[neighborCount] = {0};
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float sumFound = 0.0f;
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		sdfs[iNeighbor] = TVoxel::valueToFloat(v.sdf);
		confs[iNeighbor] = v.confidence;
		colors[iNeighbor] = TO_FLOAT3(v.clr);
		bool foundCur = vmIndex != 0;
		struckNarrowBand |= foundCur;
		found[iNeighbor] = foundCur;
		sumFound += foundCur * v.sdf;
	}
	if (!struckNarrowBand) {
		return 0.0f;
	}

	float truncated = std::copysign(1.0f, sumFound);
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		if (!found[iNeighbor]) {
			sdfs[iNeighbor] = truncated;
		}
	}

	float oneMinusCoeffX = 1.0f - coeff.x;
	float oneMinusCoeffY = 1.0f - coeff.y;

	sdfRes1 = oneMinusCoeffY * (oneMinusCoeffX * sdfs[0] + coeff.x * sdfs[1])
	          + coeff.y * (oneMinusCoeffX * sdfs[2] + coeff.x * sdfs[3]);
	sdfRes2 = oneMinusCoeffY * (oneMinusCoeffX * sdfs[4] + coeff.x * sdfs[5])
	          + coeff.y * (oneMinusCoeffX * sdfs[6] + coeff.x * sdfs[7]);
	confRes1 = oneMinusCoeffY * (oneMinusCoeffX * confs[0] + coeff.x * confs[1])
	           + coeff.y * (oneMinusCoeffX * confs[2] + coeff.x * confs[3]);
	confRes2 = oneMinusCoeffY * (oneMinusCoeffX * confs[4] + coeff.x * confs[5])
	           + coeff.y * (oneMinusCoeffX * confs[6] + coeff.x * confs[7]);
	clrRes1 = oneMinusCoeffY * (oneMinusCoeffX * colors[0] + coeff.x * colors[1])
	          + coeff.y * (oneMinusCoeffX * colors[2] + coeff.x * colors[3]);
	clrRes2 = oneMinusCoeffY * (oneMinusCoeffX * colors[4] + coeff.x * colors[5])
	          + coeff.y * (oneMinusCoeffX * colors[6] + coeff.x * colors[7]);

	float sdf = (1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2;
	confidence = (1.0f - coeff.z) * confRes1 + coeff.z * confRes2;
	color = (1.0f - coeff.z) * clrRes1 + coeff.z * clrRes2;
	return sdf;
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

//_DEBUG -- special treatment of truncated values
//sdf without color, found/not
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float interpolateTrilinearly_Corrected(const CONSTPTR(TVoxel)* voxelData,
                                              const CONSTPTR(ITMHashEntry)* voxelHash,
                                              const THREADPTR(Vector3f)& point,
                                              THREADPTR(TCache)& cache) {
	float sdfRes1, sdfRes2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;

	TO_INT_FLOOR3(pos, coeff, point);
	const int neighborCount = 8;
	float sdfs[neighborCount];
	bool found[neighborCount] = {0};
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float sumFound = 0.0f;
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		sdfs[iNeighbor] = v.sdf;
		bool foundCur = vmIndex != 0;
		found[iNeighbor] = foundCur;
		sumFound += foundCur * v.sdf;
	}
	float truncated = std::copysign(1.0f, sumFound);
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		if (!found[iNeighbor]) {
			sdfs[iNeighbor] = truncated;
		}
	}


	float oneMinusCoeffX = 1.0f - coeff.x;
	float oneMinusCoeffY = 1.0f - coeff.y;

	sdfRes1 = (oneMinusCoeffY) * ((oneMinusCoeffX) * sdfs[0] + coeff.x * sdfs[1])
	          + coeff.y * ((oneMinusCoeffX) * sdfs[2] + coeff.x * sdfs[3]);
	sdfRes2 = (oneMinusCoeffY) * ((oneMinusCoeffX) * sdfs[4] + coeff.x * sdfs[5])
	          + coeff.y * ((oneMinusCoeffX) * sdfs[6] + coeff.x * sdfs[7]);

	float sdf = TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
	return sdf;
}

//sdf without color, found/not
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float interpolateTrilinearly(const CONSTPTR(TVoxel)* voxelData,
                                    const CONSTPTR(ITMHashEntry)* voxelHash,
                                    const THREADPTR(Vector3f)& point,
                                    THREADPTR(TCache)& cache,
                                    THREADPTR(bool)& found) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	found = false;
	TO_INT_FLOOR3(pos, coeff, point);
#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.sdf;\
        found |= (vmIndex != 0 && v.flags != ITMLib::VoxelFlags::VOXEL_TRUNCATED);\
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
	float sdf = TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);

	return sdf;
}

//sdf without color
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
	float sdf = TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);

	return sdf;
}


template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPointNeighbors(THREADPTR(Vector3f)* p,
                               THREADPTR(float)* sdf,
                               const CONSTPTR(Vector3i) blockLocation,
                               const CONSTPTR(TVoxel)* localVBA,
                               const CONSTPTR(ITMHashEntry)* hashTable,
                               THREADPTR(TCache)& cache) {
	int vmIndex;
	Vector3i localBlockLocation;

	Vector3i(0, 0, 0);
	TVoxel voxel;
#define PROCESS_VOXEL(location, index)\
    localBlockLocation = blockLocation + (location);\
    p[index] = localBlockLocation.toFloat();\
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex, cache);\
    sdf[index] = TVoxel::valueToFloat(voxel.sdf);

	PROCESS_VOXEL(Vector3i(0, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 1);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 2);
	PROCESS_VOXEL(Vector3i(0, 1, 1), 3);
	PROCESS_VOXEL(Vector3i(1, 0, 0), 4);
	PROCESS_VOXEL(Vector3i(1, 0, 1), 5);
	PROCESS_VOXEL(Vector3i(1, 1, 0), 6);
	PROCESS_VOXEL(Vector3i(1, 1, 1), 7);
#undef PROCESS_VOXEL
}

//_DEBUG
//sdf without color
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float interpolateTrilinearlyAlt(const CONSTPTR(TVoxel)* voxelData,
                                       const CONSTPTR(ITMHashEntry)* voxelHash,
                                       const THREADPTR(Vector3f)& point,
                                       THREADPTR(TCache)& cache) {
	const int neighborCount = 8;
	Vector3f points[neighborCount];
	float sdfVals[neighborCount];
	findPointNeighbors(points, sdfVals, point.toIntFloor(), voxelData, voxelHash, cache);
	float sdf;

	Vector3f ratios = (point - points[0]) / (points[7] - points[0]);
	Vector3f invRatios = Vector3f(1.f) - ratios;
	Vector3f coeff;
	point.toIntFloor(coeff);

//#define INTERPOLATE_TRILINEAR(type, prefix, output, array, ratios, invRatios)\
//                    type prefix##_00 = (array)[0]*(invRatios).x + (array)[4]*(ratios).x;\
//                    type prefix##_01 = (array)[1]*(invRatios).x + (array)[5]*(ratios).x;\
//                    type prefix##_10 = (array)[2]*(invRatios).x + (array)[6]*(ratios).x;\
//                    type prefix##_11 = (array)[3]*(invRatios).x + (array)[7]*(ratios).x;\
//                    type prefix##_0 = prefix##_00*(invRatios).y + prefix##_10*(ratios).y;\
//                    type prefix##_1 = prefix##_01*(invRatios).y + prefix##_11*(ratios).y;\
//                    (output) = prefix##_0*(invRatios).z + prefix##_1 * (ratios).z;
//	INTERPOLATE_TRILINEAR(float, sdf, sdf, sdfVals, ratios, invRatios);
//#undef INTERPOLATE_TRILINEAR
	float sdf_00 = sdfVals[0] * (invRatios).x + sdfVals[4] * (ratios).x;
	float sdf_01 = sdfVals[1] * (invRatios).x + sdfVals[5] * (ratios).x;
	float sdf_10 = sdfVals[2] * (invRatios).x + sdfVals[6] * (ratios).x;
	float sdf_11 = sdfVals[3] * (invRatios).x + sdfVals[7] * (ratios).x;
	float sdf_0 = sdf_00 * (invRatios).y + sdf_10 * (ratios).y;
	float sdf_1 = sdf_01 * (invRatios).y + sdf_11 * (ratios).y;
	sdf = sdf_0 * (invRatios).z + sdf_1 * (ratios).z;
	//_DEBUG
//	if (1.0f - std::abs(sdf) < 10e-10){
//
//	}
	return sdf;
}