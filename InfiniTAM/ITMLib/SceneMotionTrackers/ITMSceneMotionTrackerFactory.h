//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
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

#include "Interface/ITMSceneMotionTracker.h"
#include "CPU/ITMSceneMotionTracker_CPU.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMSceneMotionTracker_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#error "NOT CURRENTLY SUPPORTED"
#endif

namespace ITMLib{
class ITMSceneMotionTrackerFactory {
public:
/**
* \brief Makes a scene motion tracker.
*
* \param settings  settings to use
*/
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
static ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>*
MakeSceneMotionTracker() {
	ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>* sceneRecoEngine = nullptr;
	auto& settings = ITMLibSettings::Instance();
	switch (settings.deviceType) {
		case ITMLibSettings::DEVICE_CPU:
			sceneRecoEngine = new ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>();
			break;
		case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			sceneRecoEngine = new ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, TIndex>();
#endif
			break;
		case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
			//TODO
				DIEWITHEXCEPTION("Motion Scene Tracking not yet implemented on Metal")
#endif
			break;
	}

	return sceneRecoEngine;
}
};
}//namespace ITMLib