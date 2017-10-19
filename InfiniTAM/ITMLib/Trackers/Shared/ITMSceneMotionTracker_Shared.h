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
 * \param localVBA
 * \param hashTable
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_
inline void findPointNeighborsGeneric(THREADPTR(Vector3f)* p,
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

