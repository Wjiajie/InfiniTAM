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

template<class TVoxel, class TIndex>
void ITMSceneMotionTracker<TVoxel, TIndex>::ProcessFrame(ITMScene<TVoxel, TIndex>* canonicalScene,
                                                                     ITMScene<TVoxel, TIndex>* liveScene) {

	float max_vector_update = std::numeric_limits<float>::infinity();
	const int max_iteration_count = 100; //TODO -- make parameter
	const float max_vector_update_threshold = 0.1; //TODO -- make parameter
	const float update_factor = 0.1; //TODO -- make parameter
	const float killing_term_damping_factor = 0.1;
	const float weight_killing_term = 0.1;
	const float weight_level_set_term = 0.1;

	for(int iteration = 1; max_vector_update < max_vector_update_threshold || iteration < max_iteration_count; iteration++){
		max_vector_update = UpdateWarpField(canonicalScene,liveScene);
	}


	DIEWITHEXCEPTION("Warp field updates not yet fully implemented");
}
