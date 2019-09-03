//  ================================================================
//  Created by Gregory Kramida on 8/28/19.
//  Copyright (c) 2019 Gregory Kramida
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
#include "../../Objects/Scene/ITMVoxelVolume.h"

namespace ITMLib{
/**
 * \brief Determine if every pair of corresponding voxels within the two voxel volumes is within the provided tolerance
 * of each other.
 * \details Voxel correspondence between the two volumes is determined by coinciding spatial location within both volumes.
 * The two scenes may use different indices.
 * \tparam TVoxel voxel type
 * \tparam TIndexA type of index for the first volume
 * \tparam TIndexB type of index for the second volume
 * \tparam ToleranceType type of the tolerance metric
 * \param a the first voxel volume
 * \param b the second voxel volume
 * \param tolerance (absolute) difference bound for each quantitative value in each voxel
 * \return true if scene content matches (to within specified tolerance), false otherwise
 */
template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool contentAlmostEqual_CPU(ITMVoxelVolume<TVoxel,TIndexA>* a, ITMVoxelVolume<TVoxel,TIndexB>* b, ToleranceType tolerance);

} // namespace ITMLib

