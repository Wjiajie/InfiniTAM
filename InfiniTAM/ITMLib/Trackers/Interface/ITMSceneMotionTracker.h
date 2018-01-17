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
#pragma once

#define _DEBUG
#ifdef _DEBUG

//*** LOGGING FOR 3D VISUAL DEBUGGING***
#define _LOGGER
#ifdef _LOGGER

#include "../../Utils/ITMSceneLogger.h"
#define SAVE_FRAME
//#define LOAD_FRAME
#define LOG_HIGHLIGHTS
//#define LOG_HIGHLIGHT_REGIONS //TODO

#endif //ifdef _LOGGER

#include <opencv2/core/mat.hpp>

//*** 2D RASTERIZATION FOR VISUAL DEBUGGING ***
//#define RASTERIZE_CANONICAL_SCENE
//#define RASTERIZE_LIVE_SCENE
//#define DRAW_IMAGE
#if defined(DRAW_IMAGE) || defined(RASTERIZE_CANONICAL_SCENE) || defined(RASTERIZE_LIVE_SCENE)
#include "../../Utils/ITMSceneSliceRasterizer.h"
#endif

//*** DEBUG OUTPUT MESSAGES FOR UPDATE WARP ON CPU ***
//#define PRINT_TIME_STATS //-- needs rearranging of TICs and TOCs
//#define PRINT_SINGLE_VOXEL_RESULT //Caution: very verbose!
#define PRINT_MAX_WARP_AND_UPDATE
#define PRINT_ENERGY_STATS
#define PRINT_ADDITIONAL_STATS
#define PRINT_DEBUG_HISTOGRAM
//#define OPENMP_WARP_UPDATE_COMPUTE_DISABLE
//***

#endif //ifdef _DEBUG

//local
#include "../../Objects/Scene/ITMScene.h"


namespace ITMLib {
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneMotionTracker {
protected:

	virtual float UpdateWarpField(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                              ITMScene <TVoxelLive, TIndex>* liveScene) = 0;

	virtual void AllocateNewCanonicalHashBlocks(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                                            ITMScene <TVoxelLive, TIndex>* liveScene) = 0;


	virtual void
	FuseFrame(ITMScene <TVoxelCanonical, TIndex>* canonicalScene, ITMScene <TVoxelLive, TIndex>* liveScene) = 0;

	//TODO -- make all of these parameters
	const int maxIterationCount = 200;
	const float maxVectorUpdateThresholdMeters = 0.0001f;//m
	const float gradientDescentLearningRate = 0.1f;
	const float rigidityEnforcementFactor = 0.1f;
	const float weightKillingTerm = 0.5f;
	const float weightLevelSetTerm = 0.2f;
	const float weightColorDataTerm = 0.0f;
	//_DEBUG
	const float colorSdfThreshold = -1.00f;
	//const float colorSdfThreshold = 0.25f;
	const float epsilon = FLT_EPSILON;

	float maxVectorUpdateThresholdVoxels;

#ifdef _LOGGER
	int currentFrameIx = 0;
	int iteration;
	const int frameOfInterest = 1;
	ITMSceneLogger<TVoxelCanonical,TVoxelLive,TIndex> sceneLogger;
#endif

public:
	explicit ITMSceneMotionTracker(const ITMSceneParams& params);

	void ProcessFrame(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                  ITMScene <TVoxelLive, TIndex>* liveScene);
};


}//namespace ITMLib



