//  ================================================================
//  Created by Gregory Kramida on 9/3/19.
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

#include "../../../ORUtils/PlatformIndependence.h"
#include "../ITMVoxelFlags.h"
#include "../ITMMath.h"

namespace ITMLib {


// region =========================== FUNCTIONS TO DETERMINE WHETHER A VOXEL HAS BEEN ALTERED FROM DEFAULT =============

template<bool hasSDFInformation, bool hasSemanticInformation, bool hasFlowWarpInformation, bool hasWarpUpdateInformation, typename TVoxel>
struct IsAlteredFunctor;


template<typename TVoxel>
struct IsAlteredFunctor<true, false, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.w_depth != 0;
	}
};

template<typename TVoxel>
struct IsAlteredFunctor<true, true, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.flags != ITMLib::VOXEL_UNKNOWN;
	}
};

template<typename TVoxel>
struct IsAlteredFunctor<false, true, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.flags != ITMLib::VOXEL_UNKNOWN;
	}
};

template<typename TVoxel>
struct IsAlteredFunctor<false, false, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.flow_warp != Vector3f(0.0f) || voxel.warp_update != Vector3f(0.0f);
	}
};

_CPU_AND_GPU_CODE_
template<typename TVoxel>
bool isAltered(TVoxel& voxel) {
	return IsAlteredFunctor<TVoxel::hasSDFInformation, TVoxel::hasSemanticInformation, TVoxel::hasFlowWarpInformation,
			TVoxel::hasWarpUpdateInformation, TVoxel>::evaluate(voxel);
}
// endregion

} // namespace ITMLib

