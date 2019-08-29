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
#include "ITMAlmostEqual.h"

namespace ITMLib {

// region ==================== ABSOLUTE / RELATIVE REAL TYPE COMPARISONS ===============================================

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

template<typename TReal>
inline bool almostEqualAbsolute(TReal a, TReal b, TReal epsilon = 3e-6) {
	return std::abs(a - b) < epsilon;
}
// endregion
// region =========================== FUNCTIONS TO DETERMINE WHETHER A VOXEL HAS BEEN ALTERED FROM DEFAULT =============

template<bool hasSDFInformation, bool hasSemanticInformation, bool hasFlowWarpInformation, bool hasWarpUpdateInformation, typename TVoxel>
struct IsAlteredFunctor;

template<typename TVoxel>
struct IsAlteredFunctor<true, false, false, false, TVoxel> {
	inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.w_depth != 0;
	}
};

template<typename TVoxel>
struct IsAlteredFunctor<true, true, false, false, TVoxel> {
	inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.flags != ITMLib::VOXEL_UNKNOWN;
	}
};

template<typename TVoxel>
struct IsAlteredFunctor<false, true, false, false, TVoxel> {
	inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.flags != ITMLib::VOXEL_UNKNOWN;
	}
};

template<typename TVoxel>
struct IsAlteredFunctor<false, false, true, true, TVoxel> {
	inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.flow_warp != Vector3f(0.0f) || voxel.warp_update != Vector3f(0.0f);
	}
};

template<typename TVoxel>
bool isAltered(TVoxel& voxel) {
	return IsAlteredFunctor<TVoxel::hasSDFInformation, TVoxel::hasSemanticInformation, TVoxel::hasFlowWarpInformation,
			TVoxel::hasWarpUpdateInformation, TVoxel>::evaluate(voxel);
}
// endregion



} // namespace ITMLib
