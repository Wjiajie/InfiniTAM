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
#include "../../Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.h"

namespace ITMLib {

template<typename TVoxelCanonical1, typename TVoxelLive1, typename TIndex1>
class ITMDenseDynamicMapper;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneMotionTracker {
	template<typename TVoxelCanonical1, typename TVoxelLive1, typename TIndex1>
	friend class ITMDenseDynamicMapper;
public:
//============================= CONSTRUCTORS / DESTRUCTORS =============================================================

	explicit ITMSceneMotionTracker(const ITMSceneParams& params, std::string scenePath);
	explicit ITMSceneMotionTracker(const ITMSceneParams& params, std::string scenePath, Vector3i focusCoordinates);
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

	/**
	 * \brief Warp canonical back to live
	 * \param canonicalScene
	 * \param liveScene
	 */
	virtual void
	WarpCanonicalToLive(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) = 0;

	void TrackMotion(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& sourceLiveScene,
			bool recordWarpUpdates,
			ITMSceneReconstructionEngine <TVoxelLive, TIndex>* liveSceneReconstructor);


	std::string GenerateCurrentFrameOutputPath() const;
	int GetFrameIndex() const { return currentFrameIx; }

protected:

	virtual float CalculateWarpUpdate(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                  ITMScene<TVoxelLive, TIndex>* liveScene) = 0;//TODO: refactor to "CalculateWarpGradient"

	virtual void ApplySmoothingToGradient(
			ITMScene <TVoxelCanonical, TIndex>* canonicalScene, ITMScene <TVoxelLive, TIndex>* liveScene) = 0;
	virtual float ApplyWarpUpdateToWarp(
			ITMScene <TVoxelCanonical, TIndex>* canonicalScene, ITMScene <TVoxelLive, TIndex>* liveScene) = 0;

	virtual void ApplyWarpFieldToLive(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
		                                  ITMScene <TVoxelLive, TIndex>* sourceLiveScene)= 0;
	virtual void ApplyWarpUpdateToLive(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
		                                   ITMScene <TVoxelLive, TIndex>* sourceLiveScene) = 0;



	virtual void AllocateNewCanonicalHashBlocks(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                                            ITMScene <TVoxelLive, TIndex>* liveScene) = 0;


//============================= MEMBER VARIABLES =======================================================================
	//TODO -- make all of these parameters
	const int maxIterationCount = 200;
	const float maxVectorUpdateThresholdMeters = 0.0001f;//m //original
	const float gradientDescentLearningRate = 0.1f;
	const float rigidityEnforcementFactor = 0.1f;
	const float weightSmoothnessTerm = 0.2f;
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
	std::ofstream energy_stat_file;

	// variables for extra logging/analysis
	bool hasFocusCoordinates = false;
	Vector3i focusCoordinates;

	bool rasterizeLive = false;
	bool rasterizeCanonical = false;
	bool rasterizeUpdates = false;

private:

	void InitializeUpdate2DImageLogging(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene,
			cv::Mat& blank, cv::Mat& liveImgTemplate);
	void LogWarpUpdateAs2DImage(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                            ITMScene <TVoxelLive, TIndex>* sourceLiveScene, const cv::Mat& blank,
	                            const cv::Mat& liveImgTemplate);
};


}//namespace ITMLib



