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
	return TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
}