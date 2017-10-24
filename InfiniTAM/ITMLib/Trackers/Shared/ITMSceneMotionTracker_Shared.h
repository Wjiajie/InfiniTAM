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


//TODO: assume we do have color for now. Make provisions later to account for TVoxel with no color. -Greg (GitHub:Algomorph)
/**
 * \brief find neighboring SDF values and point locations.
 * This is different from findPointNeighbors because it returns 1.0 for truncated voxels and voxels beyond the scene boundary.
 * \tparam TVoxel the voxel type
 * \param p [out] pointer to memory where to store 8 values Vector3i for all the neighbors
 * \param sdf [out] pointer to memory where to store 8 SDF values for all the neighbors
 * \param blockLocation the actual block location
 * \param localVBA the voxel grid
 * \param hashTable the hash table index of the voxel entries
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_
inline void findPointNeighbor_PositionsSdfColor(THREADPTR(Vector3f)* p,
                                                THREADPTR(float)* sdf,
                                                THREADPTR(Vector3u)* colorVals,
                                                Vector3i blockLocation,
                                                const CONSTPTR(TVoxel)* voxelData,
                                                const CONSTPTR(ITMHashEntry)* hashTable) {
	int vmIndex;
	Vector3i localBlockLocation;

	Vector3i(0, 0, 0);
	TVoxel voxel;
#define PROCESS_VOXEL(location, index)\
    localBlockLocation = blockLocation + (location);\
    p[index] = localBlockLocation.toFloat();\
    voxel = readVoxel(voxelData, hashTable, localBlockLocation, vmIndex);\
    sdf[index] = TVoxel::valueToFloat(voxel.sdf);\
    colorVals[index] = voxel.clr;

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

template<class TVoxel>
_CPU_AND_GPU_CODE_
inline void findPointNeighborWarp_t(THREADPTR(Vector3f)* warpVals,
                                    Vector3i blockLocation,
                                    const CONSTPTR(TVoxel)* voxelData,
                                    const CONSTPTR(ITMHashEntry)* hashTable) {
	int vmIndex;
	Vector3i localBlockLocation;

	Vector3i(0, 0, 0);
	TVoxel voxel;
#define PROCESS_VOXEL(location, index)\
    localBlockLocation = blockLocation + (location);\
    voxel = readVoxel(voxelData, hashTable, localBlockLocation, vmIndex);\
    warpVals[index] = voxel.warp_t;

	PROCESS_VOXEL(Vector3i(0, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 0);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 0);
	PROCESS_VOXEL(Vector3i(0, 1, 1), 0);
	PROCESS_VOXEL(Vector3i(1, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(1, 0, 1), 0);
	PROCESS_VOXEL(Vector3i(1, 1, 0), 0);
	PROCESS_VOXEL(Vector3i(1, 1, 1), 0);
#undef PROCESS_VOXEL
}


//TODO: assume we do have color for now. Make provisions later to account for TVoxel with no color. -Greg (GitHub:Algomorph)
/**
 * \brief find neighboring voxels and write their pointers to array
 * This is different from findPointNeighbors because it returns 1.0 for truncated voxels and voxels beyond the scene boundary.
 * \tparam TVoxel the voxel type
 * \param voxelPositions [out] pointer to memory where to store 8 values Vector3i for all the neighbors
 * \param sdf [out] pointer to memory where to store 8 SDF values for all the neighbors
 * \param voxelLocation the actual voxel location
 * \param localVBA the voxel grid
 * \param hashTable the hash table index of the voxel entries
 */
_CPU_AND_GPU_CODE_
inline void findPointNeighbors2(THREADPTR(Vector3f) *voxelPositions, THREADPTR(int) voxelIndexes[8],
                                const CONSTPTR(Vector3i)& baseVoxelPosition,
                                const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) *voxelIndex,
                                THREADPTR(ITMLib::ITMVoxelBlockHash::IndexCache) & cache) {
	int foundPoint;
	Vector3i location;

#define PROCESS_VOXEL(localPos, index) \
    location = baseVoxelPosition + (localPos);\
    voxelPositions[index] = (location).toFloat();\
    voxelIndexes[index] = findVoxel(voxelIndex, location, foundPoint, cache);\

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

template<class TVoxel, class TIndex>
inline void interpolateTrilinearly(const CONSTPTR(TVoxel) *voxelData,
                                   const CONSTPTR(TIndex) *voxelIndex,
                                   const THREADPTR(Vector3f) & point,
                                   THREADPTR(ITMLib::ITMVoxelBlockHash::IndexCache) & cache,
                                   THREADPTR(float)& sdf, THREADPTR(Vector3f)& color) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	Vector3f colorRes1, colorRes2, colorV1, colorV2;
	int vmIndex = false;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
		sdfV1 = v.sdf;
		colorV1 = v.clr.toFloat();
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
		sdfV2 = v.sdf;
		colorV2 = v.clr.toFloat();
	}
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes1 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
		sdfV1 = v.sdf;
		colorV1 = v.clr.toFloat();
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
		sdfV2 = v.sdf;
		colorV2 = v.clr.toFloat();
	}
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes1 = (1.0f - coeff.y) * colorRes1 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
		sdfV1 = v.sdf;
		colorV1 = v.clr.toFloat();
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
		sdfV2 = v.sdf;
		colorV2 = v.clr.toFloat();
	}
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes2 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
		sdfV1 = v.sdf;
		colorV1 = v.clr.toFloat();
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
		sdfV2 = v.sdf;
		colorV2 = v.clr.toFloat();
	}
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes2 = (1.0f - coeff.y) * colorRes2 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	sdf = TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
	color = (1.0f - coeff.z) * colorRes1 + coeff.z * colorRes2;
}

inline void interpolateTrilinearly_sdfAndColor(float& sdf, Vector3u& color, float* sdfVals, Vector3u* colorVals,
                                               Vector3f* points,
                                               Vector3f pointPosition) {
	const int neighborCount = 8;
	Vector3f colorValsF[neighborCount];
	for (int iVoxel = 0; iVoxel < neighborCount; iVoxel++) {
		colorValsF[iVoxel] = colorVals[iVoxel].toFloat();
	}
	Vector3f ratios = (pointPosition - points[0]) / (points[7] - points[0]);
	Vector3f invRatios = Vector3f(1.f) - ratios;
	Vector3f colorF;
#define INTERPOLATE_TRILINEAR(type, prefix, output, array, ratios, invRatios)\
                    type prefix##_00 = (array)[0]*(invRatios).x + (array)[4]*(ratios).x;\
                    type prefix##_01 = (array)[1]*(invRatios).x + (array)[5]*(ratios).x;\
                    type prefix##_10 = (array)[2]*(invRatios).x + (array)[6]*(ratios).x;\
                    type prefix##_11 = (array)[3]*(invRatios).x + (array)[7]*(ratios).x;\
                    type prefix##_0 = prefix##_00*(invRatios).y + prefix##_10*(ratios).y;\
                    type prefix##_1 = prefix##_01*(invRatios).y + prefix##_11*(ratios).y;\
                    (output) = prefix##_0*(invRatios).z + prefix##_1 * (ratios).z;
	INTERPOLATE_TRILINEAR(float, sdf, sdf, sdfVals, ratios, invRatios);
	INTERPOLATE_TRILINEAR(Vector3f, color, colorF, colorValsF, ratios, invRatios);
#undef INTERPOLATE_TRILINEAR
	color = colorF.toUChar();
}