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
#include "../Objects/Scene/ITMVoxelVolume.h"

namespace ITMLib{
/**
 * \brief Determine if every pair of corresponding voxels within the two voxel volumes is within the provided tolerance
 * of each other.
 * \details Correspondence is determined by coinciding spatial location. The two scenes may use different indices.
 * \tparam TVoxel voxel type
 * \tparam TIndexA
 * \tparam TIndexB
 * \tparam ToleranceType
 * \param a
 * \param b
 * \param tolerance
 * \return
 */
template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool contentAlmostEqual(ITMVoxelVolume<TVoxel,TIndexA> a, ITMVoxelVolume<TVoxel,TIndexB> b, ToleranceType tolerance);
} // namespace ITMLib

