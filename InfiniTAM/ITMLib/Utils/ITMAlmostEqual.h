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

//local
#include "../Objects/Scene/ITMVoxelTypes.h"
#include "ITMMath.h"

namespace ITMLib{


bool almostEqual(float a, float b);
bool almostEqual(double a, double b);
/**
 * \brief Determine whether the two values are within a given tolerance of each-other
 * \details The comparison is done in an absolute way, i.e. the relative value magnitudes don't matter. This is useful
 * for situations where there is a predetermined upper bound on the values, i.e. values are in range [0.0,1.0], and
 * small values don't really matter all that much.
 * \param a the first value
 * \param b the second value
 * \return true if the two values are within the provided tolerance, false otherwise.
 */
bool almostEqual(float a, float b, float tolerance);
bool almostEqual(float a, float b, double tolerance);
bool almostEqual(double a, double b, double tolerance);

template<typename ElementType, typename ToleranceType>
bool almostEqual(ORUtils::Vector2<ElementType> a, ORUtils::Vector2<ElementType> b, ToleranceType tolerance);
template<typename ElementType, typename ToleranceType>
bool almostEqual(ORUtils::Vector3<ElementType> a, ORUtils::Vector3<ElementType> b, ToleranceType tolerance);
template<typename ElementType, typename ToleranceType>
bool almostEqual(ORUtils::Matrix3<ElementType> a, ORUtils::Matrix3<ElementType> b, ToleranceType tolerance);
template<typename ElementType, typename ToleranceType>
bool almostEqual(ORUtils::Matrix4<ElementType> a, ORUtils::Matrix4<ElementType> b, ToleranceType tolerance);

template<typename TVoxel, typename ToleranceType>
bool almostEqual(TVoxel& a, TVoxel& b, ToleranceType tolerance);

/**
 * \brief Tries to determine whether the voxel been altered from default
 * \tparam TVoxel voxel type
 * \param voxel the voxel to evaluate
 * \return true if the voxel has been altered for certain, false if not (or voxel seems to have default value)
 */
template<typename TVoxel>
bool isAltered(TVoxel& voxel);



} // namespace ITMLib
