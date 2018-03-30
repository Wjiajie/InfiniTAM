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

#include "../../TempDebugDefines.h"

//local
#include "../../Objects/Scene/ITMScene.h"


namespace ITMLib {
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneMotionTracker {
public:
//============================= CONSTRUCTORS / DESTRUCTORS =============================================================

	explicit ITMSceneMotionTracker(const ITMSceneParams& params, std::string scenePath);
	virtual ~ITMSceneMotionTracker();

//============================= MEMBER FUNCTIONS =======================================================================
	/**
	 * \brief Fuses the live scene into the canonical scene based on the motion warp of the canonical scene
	 * \details Typically called after TrackMotion is called
	 * \param canonicalScene the canonical voxel grid, representing the state at the beginning of the sequence
	 * \param liveScene the live voxel grid, a TSDF generated from a single recent depth image
	 */
	virtual void
	FuseFrame(ITMScene <TVoxelCanonical, TIndex>* canonicalScene, ITMScene <TVoxelLive, TIndex>* liveScene) = 0;
	virtual void
	ApplyWarp(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) = 0;

	void TrackMotion(ITMScene <TVoxelCanonical, TIndex>* canonicalScene, ITMScene <TVoxelLive, TIndex>* liveScene,
	                 bool recordWarpUpdates = false);


	std::string GenerateCurrentFrameOutputPath() const;
	int GetFrameIndex() const { return currentFrameIx; }
protected:

//============================= MEMBER FUNCTIONS =======================================================================
	virtual float UpdateWarpField(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                              ITMScene <TVoxelLive, TIndex>* liveScene) = 0;

	virtual void AllocateNewCanonicalHashBlocks(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                                            ITMScene <TVoxelLive, TIndex>* liveScene) = 0;


//============================= MEMBER VARIABLES =======================================================================
	//TODO -- make all of these parameters
	const int maxIterationCount = 200;
	const float maxVectorUpdateThresholdMeters = 0.0001f;//m //original
	//const float maxVectorUpdateThresholdMeters = 0.00005f;//m //_DEBUG
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

	unsigned int iteration = 0;
	unsigned int currentFrameIx = 0;

	ITMSceneLogger<TVoxelCanonical,TVoxelLive,TIndex>* sceneLogger;
	std::string baseOutputDirectory;

#ifdef RECORD_CONTINOUS_HIGHLIGHTS
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> previouslyRecordedAnomalies;
#endif

#ifdef WRITE_ENERGY_STATS_TO_FILE
	std::ofstream energy_stat_file;
#endif

private:
};


}//namespace ITMLib



