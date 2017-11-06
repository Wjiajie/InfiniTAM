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
#include "../../ITMLibDefines.h"

using namespace ITMLib;

template<class TVoxel, class TIndex>
void ITMSceneMotionTracker<TVoxel, TIndex>::ProcessFrame(ITMScene<TVoxel, TIndex>* canonicalScene,
                                                                     ITMScene<ITMVoxelAux, TIndex>* liveScene) {

	float maxVectorUpdate = std::numeric_limits<float>::infinity();

	std::cout << "Desired warp update (voxels) below " << maxVectorUpdateThresholdVoxels << std::endl;
	//_DEBUG //TODO: remove temporary update limit
	for(int iteration = 0; maxVectorUpdate > maxVectorUpdateThresholdVoxels && iteration < maxIterationCount; iteration++){
		const std::string red("\033[0;31m");
		const std::string reset("\033[0m");
		std::cout << red << "Iteration: " << iteration << reset;// << std::endl;
		maxVectorUpdate = UpdateWarpField(canonicalScene, liveScene);
		std::cout << " Max update: " << maxVectorUpdate << std::endl;
	}

	this->FuseFrame(canonicalScene, liveScene);
}
template<class TVoxel, class TIndex>
ITMSceneMotionTracker<TVoxel, TIndex>::ITMSceneMotionTracker(const ITMSceneParams& params) : maxVectorUpdateThresholdVoxels(maxVectorUpdateThresholdMeters / params.voxelSize) {}
