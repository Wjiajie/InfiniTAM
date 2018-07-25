//  ================================================================
//  Created by Gregory Kramida on 7/25/18.
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

#include "../../../ORUtils/PlatformIndependence.h"
#include "../../Utils/ITMMath.h"
#include "ITMRepresentationAccess.h"

template<typename TVoxel, typename TCache, typename TIndexData>
_CPU_AND_GPU_CODE_
inline float ProcessTrilinearNeighbor(const CONSTPTR(TVoxel)* voxelData,
                                      const CONSTPTR(TIndexData)* voxelIndex,
                                      const CONSTPTR(int)& sdfIndex,
                                      const CONSTPTR(Vector3i)& position,
                                      const CONSTPTR(float*) coefficients,
                                      THREADPTR(TCache)& cache,
                                      THREADPTR(bool)& struckKnownVoxels,
                                      int iNeighbor,
                                      THREADPTR(float)& sdf,
                                      THREADPTR(float)& cumulativeWeight){
	int vmIndex;

	const TVoxel& v = readVoxel(voxelData, voxelIndex, position, vmIndex, cache);
	bool curKnown = v.flag_values[sdfIndex] != ITMLib::VOXEL_UNKNOWN;
	//_DEBUG
	//trilinear unknown filter: ON
	//float weight = coefficients[iNeighbor] * curKnown;
	//trilinear unknown filter: OFF
	float weight = coefficients[iNeighbor];

	sdf += weight * TVoxel::valueToFloat(v.sdf_values[sdfIndex]);
	struckKnownVoxels |= curKnown;
	cumulativeWeight += weight;
	return weight;
};

_CPU_AND_GPU_CODE_
inline void ComputeTrilinearCoefficents(const CONSTPTR(Vector3f)& point, THREADPTR(Vector3i)& pos,
THREADPTR(float*) coefficients /*x8*/){
Vector3f ratios;
Vector3f inverseRatios(1.0f);

TO_INT_FLOOR3(pos, ratios, point);
inverseRatios -= ratios;

//@formatter:off
	coefficients[0] = inverseRatios.x * inverseRatios.y * inverseRatios.z; //000
	coefficients[1] = ratios.x *        inverseRatios.y * inverseRatios.z; //100
	coefficients[2] = inverseRatios.x * ratios.y *        inverseRatios.z; //010
	coefficients[3] = ratios.x *        ratios.y *        inverseRatios.z; //110
	coefficients[4] = inverseRatios.x * inverseRatios.y * ratios.z;        //001
	coefficients[5] = ratios.x *        inverseRatios.y * ratios.z;        //101
	coefficients[6] = inverseRatios.x * ratios.y *        ratios.z;        //011
	coefficients[7] = ratios.x *        ratios.y *        ratios.z;        //111
}
