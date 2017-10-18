//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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
#include <limits>
#include "ITMSceneMotionTracker.h"

using namespace ITMLib;

template<class TVoxel, class TWarpField, class TIndex>
void ITMSceneMotionTracker<TVoxel, TWarpField, TIndex>::UpdateWarpField(ITMScene<TVoxel, TIndex>* canonical_scene,
                                                                        ITMScene<TVoxel, TIndex>* live_scene,
                                                                        ITMScene<TWarpField, TIndex>* warp_field) {

	float error = std::numeric_limits<float>::infinity();
	const int max_iteration_count = 100;
	const float error_threshold = 0.1;
	for(int iteration = 0; error < error_threshold || iteration < max_iteration_count; iteration++){
		error = this->PerformUpdateIteration(canonical_scene,live_scene,warp_field);
	}
	DIEWITHEXCEPTION("Warp field updates not yet fully implemented");
}
