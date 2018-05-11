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

#include <opencv2/core/mat.hpp>
#include "../Interface/ITMSceneMotionTracker.h"
#include "../../Utils/ITMHashBlockProperties.h"

namespace ITMLib {
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneMotionTracker_CPU :
		public ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex> {
public:

	ITMSceneMotionTracker_CPU(const ITMSceneParams& params,
	                          std::string scenePath,

	                          bool enableDataTerm,
	                          bool enableLevelSetTerm,
	                          bool enableSmoothingTerm,
	                          bool enableKillingTerm,
	                          bool enableGradientSmoothing,

	                          unsigned int maxIterationCount = 200,
	                          float maxVectorUpdateThresholdMeters = 0.0001f,
	                          float gradientDescentLearningRate = 0.0f,
	                          float rigidityEnforcementFactor = 0.1f,
	                          float weightSmoothnessTerm = 0.2f,
	                          float weightLevelSetTerm = 0.2f,
	                          float epsilon = FLT_EPSILON);

	ITMSceneMotionTracker_CPU(const ITMSceneParams& params,
	                          std::string scenePath,
	                          Vector3i focusCoordinates,

	                          bool enableDataTerm,
	                          bool enableLevelSetTerm,
	                          bool enableSmoothingTerm,
	                          bool enableKillingTerm,
	                          bool enableGradientSmoothing,

	                          unsigned int maxIterationCount = 200,
	                          float maxVectorUpdateThresholdMeters = 0.0001f,
	                          float gradientDescentLearningRate = 0.0f,
	                          float rigidityEnforcementFactor = 0.1f,
	                          float weightSmoothnessTerm = 0.2f,
	                          float weightLevelSetTerm = 0.2f,
	                          float epsilon = FLT_EPSILON);
	virtual ~ITMSceneMotionTracker_CPU();
	void FuseFrame(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) override;
	void WarpCanonicalToLive(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                         ITMScene<TVoxelLive, TIndex>* liveScene) override;

protected:

	const bool enableDataTerm;
	const bool enableLevelSetTerm;
	const bool enableSmoothingTerm;
	const bool enableKillingTerm;
	const bool enableGradientSmoothing;



	void ApplyWarpFieldToLive(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                          ITMScene<TVoxelLive, TIndex>* liveScene) override;
	void ApplySmoothingToGradient(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) override;
	float ApplyWarpUpdateToWarp(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) override;
	void ApplyWarpUpdateToLive(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                           ITMScene<TVoxelLive, TIndex>* liveScene) override;


	float CalculateWarpUpdate(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                          ITMScene<TVoxelLive, TIndex>* liveScene) override;


	void AllocateNewCanonicalHashBlocks(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                    ITMScene<TVoxelLive, TIndex>* liveScene) override;


private:
	void AllocateHashBlocksAtWarpedLocations(
			ITMScene <TVoxelCanonical, TIndex>* warpSourceScene, ITMScene <TVoxelLive, TIndex>* sdfScene, int flagIndex);

	float CalculateWarpUpdate_SingleThreadedVerbose(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                                ITMScene<TVoxelLive, TIndex>* liveScene);
	float CalculateWarpUpdate_MultiThreaded(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                        ITMScene<TVoxelLive, TIndex>* liveScene);

	float ApplyWarpUpdateToWarp_SingleThreadedVerbose(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene);

	float ApplyWarpUpdateToWarp_MultiThreaded(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene);


	void InitializeHelper(const ITMLib::ITMSceneParams& sceneParams);
	void PrintSettings();
	ORUtils::MemoryBlock<unsigned char>* hashEntryAllocationTypes;
	ORUtils::MemoryBlock<unsigned char>* canonicalEntryAllocationTypes;
	ORUtils::MemoryBlock<Vector3s>* allocationBlockCoordinates;

};


}//namespace ITMLib


