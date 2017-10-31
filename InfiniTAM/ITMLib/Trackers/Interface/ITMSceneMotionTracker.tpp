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

	float maxVectorUpdate = std::numeric_limits<float>::infinity();

	for(int iteration = 0; maxVectorUpdate > maxVectorUpdateThreshold && iteration < maxIterationCount; iteration++){
		std::cout << "Iteration: " << iteration << std::endl;
		maxVectorUpdate = UpdateWarpField(canonicalScene,liveScene);
		std::cout << " Max vector update: " << maxVectorUpdate << std::endl;
	}

	this->FuseFrame(canonicalScene, liveScene);
}
