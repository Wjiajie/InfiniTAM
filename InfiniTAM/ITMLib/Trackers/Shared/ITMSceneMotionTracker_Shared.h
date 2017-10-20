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
 * \brief find neighboring voxels and write their pointers to array
 * This is different from findPointNeighbors because it returns 1.0 for truncated voxels and voxels beyond the scene boundary.
 * \tparam TVoxel the voxel type
 * \param p [out] pointer to memory where to store 8 values Vector3i for all the neighbors
 * \param sdf [out] pointer to memory where to store 8 SDF values for all the neighbors
 * \param blockLocation the actual block location
 * \param localVBA
 * \param hashTable
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_
inline void findPointNeighbors_PositionsAndPointers(THREADPTR(Vector3f)* p,
                                                    THREADPTR(TVoxel)** voxels,
                                                    Vector3i blockLocation,
                                                    const CONSTPTR(TVoxel)* localVBA,
                                                    const CONSTPTR(ITMHashEntry)* hashTable) {
	int vmIndex;
	Vector3i localBlockLocation;

	Vector3i(0, 0, 0);
	TVoxel voxel;
#define PROCESS_VOXEL(location, index)\
    localBlockLocation = blockLocation + (location);\
    p[index] = localBlockLocation.toFloat();\
    voxels[index] = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);

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
 * \brief find neighboring SDF values and point locations.
 * This is different from findPointNeighbors because it returns 1.0 for truncated voxels and voxels beyond the scene boundary.
 * \tparam TVoxel the voxel type
 * \param p [out] pointer to memory where to store 8 values Vector3i for all the neighbors
 * \param sdf [out] pointer to memory where to store 8 SDF values for all the neighbors
 * \param blockLocation the actual block location
 * \param localVBA
 * \param hashTable
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_
inline void findPointNeighbor_PositionsSdfColor(THREADPTR(Vector3f)* p,
                                                THREADPTR(float)* sdf,
                                                THREADPTR(Vector3u)* colorVals,
                                                Vector3i blockLocation,
                                                const CONSTPTR(TVoxel)* localVBA,
                                                const CONSTPTR(ITMHashEntry)* hashTable) {
	int vmIndex;
	Vector3i localBlockLocation;

	Vector3i(0, 0, 0);
	TVoxel voxel;
#define PROCESS_VOXEL(location, index)\
    localBlockLocation = blockLocation + (location);\
    p[index] = localBlockLocation.toFloat();\
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);\
    sdf[index] = TVoxel::valueToFloat(voxel.sdf);\
    colorVals[index] = voxel.clr;

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

template<class TVoxel>
_CPU_AND_GPU_CODE_
inline void findPointNeighborWarp_t(THREADPTR(Vector3f)* warpVals,
                                    Vector3i blockLocation,
                                    const CONSTPTR(TVoxel)* localVBA,
                                    const CONSTPTR(ITMHashEntry)* hashTable) {
	int vmIndex;
	Vector3i localBlockLocation;

	Vector3i(0, 0, 0);
	TVoxel voxel;
#define PROCESS_VOXEL(location, index)\
    localBlockLocation = blockLocation + (location);\
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);\
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

inline
<class TVoxel>
inline void distributeTrilinearly(float& sdf, Vector3u& color, Vector3f& pointPosition) {
	const int neighborCount = 8;
	Vector3i neighbor0BlockLocation = pointPosition.toIntRound() - Vector3i(1);

	TVoxel* neighbors[neighborCount];
	Vector3f neighborPositions[neighborCount];

	Vector3f colorF = color.toFloat();
	Vector3f ratios = (pointPosition - neighborPositions[0]) / (neighborPositions[7] - neighborPositions[0]);
	Vector3f invRatios = Vector3f(1.f) - ratios;

#define DISTRIBUTE_TRILINEAR(type, prefix, input, array, field, modifier, ratios, invRatios)\
                    type prefix##_0 = (input)*(invRatios).z;\
					type prefix##_1 = (input)*(ratios).z;\
					type prefix##_00 = prefix##_0*(invRatios).y;\
                    type prefix##_01 = prefix##_1*(invRatios).y;\
                    type prefix##_10 = prefix##_0*(ratios).y;\
					type prefix##_11 = prefix##_1*(ratios).y;\
					(array)[0]##field = (prefix##_00*(invRatios).x)##modifier;\
					(array)[1]##field = (prefix##_01*(invRatios).x)##modifier;\
					(array)[2]##field = (prefix##_10*(invRatios).x)##modifier;\
                    (array)[3]##field = (prefix##_11*(invRatios).x)##modifier;\
					(array)[4]##field = (prefix##_00*(ratios).x)##modifier;\
					(array)[5]##field = (prefix##_01*(ratios).x)##modifier;\
					(array)[6]##field = (prefix##_10*(ratios).x)##modifier;\
					(array)[7]##field = (prefix##_11*(ratios).x)##modifier;
	DISTRIBUTE_TRILINEAR(float, sdf, sdf, neighbors, ->sdf, ,ratios, invRatios);
	//DISTRIBUTE_TRILINEAR(Vector3f, color, colorF, colorValsF, ratios, invRatios);
#undef DISTRIBUTE_TRILINEAR
	//discretize float colors
//	for (int iVoxel = 0; iVoxel < neighborCount; iVoxel++) {
//		colorVals[iVoxel] = colorValsF[iVoxel].toUChar();
//	}
}

inline void interpolateTrilinearly(float& sdf, Vector3u& color, float* sdfVals, Vector3u* colorVals, Vector3f* points,
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