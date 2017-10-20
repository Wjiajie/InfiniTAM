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
void ITMSceneMotionTracker<TVoxel, TWarpField, TIndex>::ProcessFrame(ITMScene<TVoxel, TIndex>* canonicalScene,
                                                                     ITMScene<TVoxel, TIndex>* liveScene,
                                                                     ITMScene<TWarpField, TIndex>* warpField) {

	float energy = std::numeric_limits<float>::infinity();
	const int max_iteration_count = 100; //TODO -- make parameter
	const float energy_threshold = 0.1; //TODO -- make parameter



	for(int iteration = 1; energy < energy_threshold || iteration < max_iteration_count; iteration++){
		ITMScene<TVoxel, TIndex>* deformedLiveScene = new ITMScene<TVoxel,TIndex>(liveScene->sceneParams, liveScene->globalCache == NULL
				, liveScene->index.getMemoryType());
		ITMScene<TWarpField, TIndex>* warpFieldDelta = new ITMScene<TWarpField,TIndex>(warpField->sceneParams, warpField->globalCache == NULL
				, warpField->index.getMemoryType());
		delete liveScene;
		liveScene = deformedLiveScene;
		DeformScene(liveScene,deformedLiveScene,warpField);
	}


	DIEWITHEXCEPTION("Warp field updates not yet fully implemented");
}
