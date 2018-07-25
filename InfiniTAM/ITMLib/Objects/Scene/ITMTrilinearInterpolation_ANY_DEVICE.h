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

//stdlib
#include <cfloat>

//local
#include "../../../ORUtils/PlatformIndependence.h"
#include "../../Utils/ITMMath.h"
#include "ITMTrilinearInterpolation_Shared.h"

/**
 * \brief Given an arbitrary (float-valued) point, use trilinear interpolation to get the signed distance function value
 * at this point.
 * \details The sdf_values field will be sampled using sdfIndex, i.e. sdf_values[sdfIndex]. In the meantime,
 * also determines whether any of the voxels in the sampling space (2x2x2 voxels) has a known (established) value,
 * to discriminate it from the newly-initialized voxels set to the default sdf value.
 * \tparam TVoxel
 * \tparam TCache
 * \tparam TIndexData
 * \param voxelData
 * \param voxelIndex
 * \param point
 * \param sdfIndex
 * \param cache
 * \param struckKnownVoxels
 * \return
 */

template<typename TVoxel, typename TCache, typename TIndexData>
_CPU_AND_GPU_CODE_
inline float _DEBUG_InterpolateMultiSdfTrilinearly_StruckKnown(const CONSTPTR(TVoxel)* voxelData,
                                                               const CONSTPTR(TIndexData)* voxelIndex,
                                                               const CONSTPTR(Vector3f)& point,
                                                               int sdfIndex,
                                                               THREADPTR(TCache)& cache,
                                                               THREADPTR(bool)& struckKnownVoxels,
                                                               bool verbose = false) {

	const int neighborCount = 8;
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float coefficients[neighborCount];
	Vector3i pos;
	ComputeTrilinearCoefficents(point, pos, coefficients);

	struckKnownVoxels = false;
	float cumulativeWeight = 0.0f;
	float sdf = 0.0f;

	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		float weight = ProcessTrilinearNeighbor(voxelData, voxelIndex, sdfIndex, pos + positions[iNeighbor],
		                                        coefficients, cache,
		                                        struckKnownVoxels, iNeighbor, sdf, cumulativeWeight);

	}
	if (cumulativeWeight > FLT_EPSILON) {
		sdf /= cumulativeWeight;
	} else {
		sdf = TVoxel::SDF_initialValue();
	}
	return sdf;
}