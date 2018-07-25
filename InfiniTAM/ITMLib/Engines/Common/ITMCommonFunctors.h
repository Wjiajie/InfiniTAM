//  ================================================================
//  Created by Gregory Kramida on 7/24/18.
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

// region ===================================== VOXEL LOOKUPS ==========================================================
template <typename TVoxel>
struct LookupBasedOnWarpStaticFunctor{
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TVoxel& voxel,const Vector3i& position){
		return position.toFloat() + voxel.warp;
	}
};


template <typename TVoxel>
struct LookupBasedOnWarpUpdateStaticFunctor{
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TVoxel& voxel,const Vector3i& position){
		return position.toFloat() + voxel.warp_update;
	}
};
// endregion ===========================================================================================================


