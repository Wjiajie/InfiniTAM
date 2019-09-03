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
//stdlib
#include <cmath>
#include <string>
#include <limits>

//local
#include "../ITMVoxelVolumeComparison_CPU.h"
#include "../../Objects/Scene/ITMVoxelTypes.h"
#include "../ITMMath.h"
#include "../../../ORUtils/PlatformIndependence.h"

namespace ITMLib {

// region ============================================== DECLARATIONS ==================================================

_CPU_AND_GPU_CODE_ bool almostEqual(float a, float b);
_CPU_AND_GPU_CODE_ bool almostEqual(double a, double b);
/**
 * \brief Determine whether the two values are within a given tolerance of each-other
 * \details The comparison is done in an absolute way, i.e. the relative value magnitudes don't matter. This is useful
 * for situations where there is a predetermined upper bound on the values, i.e. values are in range [0.0,1.0], and
 * small values don't really matter all that much.
 * \param a the first value
 * \param b the second value
 * \return true if the two values are within the provided tolerance, false otherwise.
 */
_CPU_AND_GPU_CODE_ bool almostEqual(float a, float b, float tolerance);
_CPU_AND_GPU_CODE_ bool almostEqual(float a, float b, double tolerance);
_CPU_AND_GPU_CODE_ bool almostEqual(double a, double b, double tolerance);


template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
bool almostEqual(ORUtils::Vector2<ElementType> a, ORUtils::Vector2<ElementType> b, ToleranceType tolerance);

template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
bool almostEqual(ORUtils::Vector3<ElementType> a, ORUtils::Vector3<ElementType> b, ToleranceType tolerance);

template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
bool almostEqual(ORUtils::Matrix3<ElementType> a, ORUtils::Matrix3<ElementType> b, ToleranceType tolerance);
_CPU_AND_GPU_CODE_
template<typename ElementType, typename ToleranceType>
bool almostEqual(ORUtils::Matrix4<ElementType> a, ORUtils::Matrix4<ElementType> b, ToleranceType tolerance);
_CPU_AND_GPU_CODE_
template<typename TVoxel, typename ToleranceType>
bool almostEqual(TVoxel& a, TVoxel& b, ToleranceType tolerance);

//TODO: below function probs should be in a different header than "AlmostEqual"
/**
 * \brief Tries to determine whether the voxel been altered from default
 * \tparam TVoxel voxel type
 * \param voxel the voxel to evaluate
 * \return true if the voxel has been altered for certain, false if not (or voxel seems to have default value)
 **/
_CPU_AND_GPU_CODE_
template<typename TVoxel>
bool isAltered(TVoxel& voxel);

// endregion

// region ==================== ABSOLUTE / RELATIVE REAL TYPE COMPARISONS ===============================================
_CPU_AND_GPU_CODE_
template<typename TReal>
inline bool almostEqualRelative(TReal a, TReal b, TReal epsilon = 3e-6) {
	const TReal absA = std::abs(a);
	const TReal absB = std::abs(b);
	const TReal diff = std::abs(a - b);

	if (a == b) { // shortcut, handles infinities
		return true;
	} else if (a == 0 || b == 0 || diff < std::numeric_limits<TReal>::denorm_min()) {
		// a or b is zero or both are extremely close to it
		// relative error is less meaningful here
		return diff < (epsilon * std::numeric_limits<TReal>::denorm_min());
	} else { // use relative error
		return diff / std::min((absA + absB), std::numeric_limits<TReal>::max()) < epsilon;
	}
}

_CPU_AND_GPU_CODE_
template<typename TReal>
inline bool almostEqualAbsolute(TReal a, TReal b, TReal epsilon = 3e-6) {
	return std::abs(a - b) < epsilon;
}
// endregion


_CPU_AND_GPU_CODE_
bool almostEqual(float a, float b) {
	return almostEqualAbsolute(a, b);
}

_CPU_AND_GPU_CODE_
bool almostEqual(double a, double b) {
	return almostEqualAbsolute(a, b);
}

_CPU_AND_GPU_CODE_
bool almostEqual(float a, float b, float tolerance) {
	return almostEqualAbsolute(a, b, tolerance);
}

_CPU_AND_GPU_CODE_
bool almostEqual(float a, float b, double tolerance) {
	return almostEqualAbsolute(a, b, static_cast<float>(tolerance));
}

_CPU_AND_GPU_CODE_
bool almostEqual(double a, double b, double tolerance) {
	return almostEqualAbsolute(a, b, tolerance);
}


//region ==================== SPECIFIC REAL TYPE COMPARISONS ===========================================================

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<float, float>(Vector2f a, Vector2f b, float tolerance) {
	return almostEqualAbsolute(a.x, b.x, tolerance) && almostEqualAbsolute(a.y, b.y, tolerance);
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<float, float>(Vector3f a, Vector3f b, float tolerance) {
	return almostEqualAbsolute(a.x, b.x, tolerance)
	       && almostEqualAbsolute(a.y, b.y, tolerance)
	       && almostEqualAbsolute(a.z, b.z, tolerance);
}

_CPU_AND_GPU_CODE_
//color comparison
template<>
bool almostEqual<uchar, float>(Vector3u a, Vector3u b, float tolerance) {
	auto a_float_normalized = TO_FLOAT3(a) / 255.0f;
	auto b_float_normalized = TO_FLOAT3(b) / 255.0f;
	return almostEqual<float, float>(a_float_normalized, b_float_normalized, tolerance);
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<float, float>(Matrix3f a, Matrix3f b, float tolerance) {
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance)) return false;
	}
	return true;
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<float, float>(Matrix4f a, Matrix4f b, float tolerance) {
	for (int i_entry = 0; i_entry < 16; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance)) return false;
	}
	return true;
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<float, double>(Vector2f a, Vector2f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	return almostEqualAbsolute(a.x, b.x, tolerance_float) && almostEqualAbsolute(a.y, b.y, tolerance_float);
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<float, double>(Vector3f a, Vector3f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	return almostEqualAbsolute(a.x, b.x, tolerance_float)
	       && almostEqualAbsolute(a.y, b.y, tolerance_float)
	       && almostEqualAbsolute(a.z, b.z, tolerance_float);
}

_CPU_AND_GPU_CODE_
//color comparison
template<>
bool almostEqual<uchar, double>(Vector3u a, Vector3u b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	auto a_float_normalized = TO_FLOAT3(a) / 255.0f;
	auto b_float_normalized = TO_FLOAT3(b) / 255.0f;
	return almostEqual<float, float>(a_float_normalized, b_float_normalized, tolerance_float);
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<float, double>(Matrix3f a, Matrix3f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance_float)) return false;
	}
	return true;
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<float, double>(Matrix4f a, Matrix4f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance_float)) return false;
	}
	return true;
}
//endregion

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<ITMVoxel_f, float>(ITMVoxel_f& a, ITMVoxel_f& b, float tolerance) {
	return almostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth;
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<ITMVoxel_s, float>(ITMVoxel_s& a, ITMVoxel_s& b, float tolerance) {
	return almostEqual(ITMVoxel_s_rgb::valueToFloat(a.sdf), ITMVoxel_s_rgb::valueToFloat(b.sdf), tolerance) &&
	       a.w_depth == b.w_depth;
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<ITMVoxel_f_rgb, float>(ITMVoxel_f_rgb& a, ITMVoxel_f_rgb& b, float tolerance) {
	return almostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth &&
	       almostEqual(a.clr, b.clr, tolerance) &&
	       a.w_color == b.w_color;
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<ITMVoxel_s_rgb, float>(ITMVoxel_s_rgb& a, ITMVoxel_s_rgb& b, float tolerance) {
	return almostEqual(ITMVoxel_s_rgb::valueToFloat(a.sdf), ITMVoxel_s_rgb::valueToFloat(b.sdf), tolerance) &&
	       a.w_depth == b.w_depth &&
	       almostEqual(a.clr, b.clr, tolerance) &&
	       a.w_color == b.w_color;
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<ITMVoxel_f_conf, float>(ITMVoxel_f_conf& a, ITMVoxel_f_conf& b, float tolerance) {
	return almostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth &&
	       almostEqual(a.confidence, b.confidence, tolerance);
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<ITMVoxel_s_rgb_conf, float>(ITMVoxel_s_rgb_conf& a, ITMVoxel_s_rgb_conf& b, float tolerance) {
	return almostEqual(ITMVoxel_s_rgb::valueToFloat(a.sdf), ITMVoxel_s_rgb::valueToFloat(b.sdf), tolerance) &&
	       a.w_depth == b.w_depth &&
	       a.clr == b.clr &&
	       almostEqual(a.clr, b.clr, tolerance) &&
	       almostEqual(a.confidence, b.confidence, tolerance);
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<ITMVoxel_f_warp, float>(ITMVoxel_f_warp& a, ITMVoxel_f_warp& b, float tolerance) {
	return almostEqual(a.flow_warp, b.flow_warp, tolerance) &&
	       almostEqual(a.gradient0, b.gradient1, tolerance) &&
	       almostEqual(a.gradient1, b.gradient1, tolerance);
}

_CPU_AND_GPU_CODE_
template<>
bool almostEqual<ITMVoxel_f_flags, float>(ITMVoxel_f_flags& a, ITMVoxel_f_flags& b, float tolerance) {
	return almostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth &&
	       a.flags == b.flags;
}
// endregion

} // namespace ITMLib
