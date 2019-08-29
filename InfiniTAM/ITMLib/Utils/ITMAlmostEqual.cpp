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
#include "ITMAlmostEqual.tpp"

namespace ITMLib {
bool almostEqual(float a, float b) {
	return almostEqualAbsolute(a, b);
}

bool almostEqual(double a, double b) {
	return almostEqualAbsolute(a, b);
}

bool almostEqual(float a, float b, float tolerance) {
	return almostEqualAbsolute(a, b, tolerance);
}

bool almostEqual(float a, float b, double tolerance) {
	return almostEqualAbsolute(a, b, static_cast<float>(tolerance));
}

bool almostEqual(double a, double b, double tolerance) {
	return almostEqualAbsolute(a, b, tolerance);
}


//region ==================== SPECIFIC REAL TYPE COMPARISONS ===========================================================
template<>
bool almostEqual<float, float>(Vector2f a, Vector2f b, float tolerance) {
	return almostEqualAbsolute(a.x, b.x, tolerance) && almostEqualAbsolute(a.y, b.y, tolerance);
}

template<>
bool almostEqual<float, float>(Vector3f a, Vector3f b, float tolerance) {
	return almostEqualAbsolute(a.x, b.x, tolerance)
	       && almostEqualAbsolute(a.y, b.y, tolerance)
	       && almostEqualAbsolute(a.z, b.z, tolerance);
}

//color comparison
template<>
bool almostEqual<uchar, float>(Vector3u a, Vector3u b, float tolerance) {
	auto a_float_normalized = TO_FLOAT3(a) / 255.0f;
	auto b_float_normalized = TO_FLOAT3(b) / 255.0f;
	return almostEqual<float, float>(a_float_normalized, b_float_normalized, tolerance);
}

template<>
bool almostEqual<float, float>(Matrix3f a, Matrix3f b, float tolerance) {
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance)) return false;
	}
	return true;
}

template<>
bool almostEqual<float, float>(Matrix4f a, Matrix4f b, float tolerance) {
	for (int i_entry = 0; i_entry < 16; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance)) return false;
	}
	return true;
}

template<>
bool almostEqual<float, double>(Vector2f a, Vector2f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	return almostEqualAbsolute(a.x, b.x, tolerance_float) && almostEqualAbsolute(a.y, b.y, tolerance_float);
}

template<>
bool almostEqual<float, double>(Vector3f a, Vector3f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	return almostEqualAbsolute(a.x, b.x, tolerance_float)
	       && almostEqualAbsolute(a.y, b.y, tolerance_float)
	       && almostEqualAbsolute(a.z, b.z, tolerance_float);
}

//color comparison
template<>
bool almostEqual<uchar, double>(Vector3u a, Vector3u b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	auto a_float_normalized = TO_FLOAT3(a) / 255.0f;
	auto b_float_normalized = TO_FLOAT3(b) / 255.0f;
	return almostEqual<float, float>(a_float_normalized, b_float_normalized, tolerance_float);
}

template<>
bool almostEqual<float, double>(Matrix3f a, Matrix3f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance_float)) return false;
	}
	return true;
}

template<>
bool almostEqual<float, double>(Matrix4f a, Matrix4f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance_float)) return false;
	}
	return true;
}
//endregion

template<>
bool almostEqual<ITMVoxel_f, float>(ITMVoxel_f& a, ITMVoxel_f& b, float tolerance) {
	return almostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth;
}

template<>
bool almostEqual<ITMVoxel_s, float>(ITMVoxel_s& a, ITMVoxel_s& b, float tolerance) {
	return almostEqual(ITMVoxel_s_rgb::valueToFloat(a.sdf), ITMVoxel_s_rgb::valueToFloat(b.sdf), tolerance) &&
	       a.w_depth == b.w_depth;
}

template<>
bool almostEqual<ITMVoxel_f_rgb, float>(ITMVoxel_f_rgb& a, ITMVoxel_f_rgb& b, float tolerance) {
	return almostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth &&
	       almostEqual(a.clr, b.clr, tolerance) &&
	       a.w_color == b.w_color;
}

template<>
bool almostEqual<ITMVoxel_s_rgb, float>(ITMVoxel_s_rgb& a, ITMVoxel_s_rgb& b, float tolerance) {
	return almostEqual(ITMVoxel_s_rgb::valueToFloat(a.sdf), ITMVoxel_s_rgb::valueToFloat(b.sdf), tolerance) &&
	       a.w_depth == b.w_depth &&
	       almostEqual(a.clr, b.clr, tolerance) &&
	       a.w_color == b.w_color;
}

template<>
bool almostEqual<ITMVoxel_f_conf, float>(ITMVoxel_f_conf& a, ITMVoxel_f_conf& b, float tolerance) {
	return almostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth &&
	       almostEqual(a.confidence, b.confidence, tolerance);
}

template<>
bool almostEqual<ITMVoxel_s_rgb_conf, float>(ITMVoxel_s_rgb_conf& a, ITMVoxel_s_rgb_conf& b, float tolerance) {
	return almostEqual(ITMVoxel_s_rgb::valueToFloat(a.sdf), ITMVoxel_s_rgb::valueToFloat(b.sdf), tolerance) &&
	       a.w_depth == b.w_depth &&
	       a.clr == b.clr &&
	       almostEqual(a.clr, b.clr, tolerance) &&
	       almostEqual(a.confidence, b.confidence, tolerance);
}

template<>
bool almostEqual<ITMVoxel_f_warp, float>(ITMVoxel_f_warp& a, ITMVoxel_f_warp& b, float tolerance) {
	return almostEqual(a.flow_warp, b.flow_warp, tolerance) &&
	       almostEqual(a.gradient0, b.gradient1, tolerance) &&
	       almostEqual(a.gradient1, b.gradient1, tolerance);
}

template<>
bool almostEqual<ITMVoxel_f_flags, float>(ITMVoxel_f_flags& a, ITMVoxel_f_flags& b, float tolerance) {
	return almostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth &&
	       a.flags == b.flags;
}

} // namespace ITMLib
