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

//local
#include "../../Objects/Scene/ITMScene.h"
#include "../../Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.h"
#include "../../Utils/ITMSceneSliceRasterizer.h"
#include "../../Utils/ITMLibSettings.h"
#include "../../Utils/FileIO/ITMSceneLogger.h"

namespace ITMLib {

template<typename TVoxelCanonical1, typename TVoxelLive1, typename TIndex1>
class ITMDenseDynamicMapper;

//TODO: write documentation block -Greg (Github: Algomorph)
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneMotionTracker {
	template<typename TVoxelCanonical1, typename TVoxelLive1, typename TIndex1>
	friend
	class ITMDenseDynamicMapper;
public:
//============================= CONSTRUCTORS / DESTRUCTORS =============================================================
//TODO: write documentation block -Greg (Github: Algomorph)
//TODO: simplify constructor to just accept the ITMLibSettings object and set the parameters from it

	explicit ITMSceneMotionTracker(const ITMLibSettings* settings);

	virtual ~ITMSceneMotionTracker();

//============================= MEMBER FUNCTIONS =======================================================================
	/**
	 * \brief Fuses the live scene into the canonical scene based on the motion warp of the canonical scene
	 * \details Typically called after TrackMotion is called
	 * \param canonicalScene the canonical voxel grid, representing the state at the beginning of the sequence
	 * \param liveScene the live voxel grid, a TSDF generated from a single recent depth image
	 */
	virtual void
	FuseFrame(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) = 0;

	/**
	 * \brief Warp canonical back to live
	 * \param canonicalScene
	 * \param liveScene
	 */
	virtual void
	WarpCanonicalToLive(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) = 0;

	void TrackMotion(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene,
			bool recordWarpUpdates);

	void SetUpStepByStepTracking(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& sourceLiveScene);
	bool UpdateTrackingSingleStep(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& sourceLiveScene);


	std::string GenerateCurrentFrameOutputPath() const;

	int GetTrackedFrameCount() const { return trackedFrameCount; }
	bool GetInSimpleSceneExperimentMode() const { return simpleSceneExperimentModeEnabled;}

	struct Parameters {
		const unsigned int maxIterationCount;// = 200;
		const float maxVectorUpdateThresholdMeters;// = 0.0001f;//m //original for KillingFusion
		const float gradientDescentLearningRate;// = 0.1f;
		const float rigidityEnforcementFactor;// = 0.1f;
		const float weightDataTerm;
		const float weightSmoothnessTerm;// = 0.2f; //0.2 is default for SobolevFusion, 0.5 is default for KillingFusion
		const float weightLevelSetTerm;// = 0.2f;
		const float epsilon;// = 1e-5f;
		const float unity; // voxelSize / mu, i.e. voxelSize / [narrow-band half-width]
	};

protected:

	virtual void CalculateWarpGradient(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                   ITMScene<TVoxelLive, TIndex>* liveScene) = 0;//TODO: refactor to "CalculateWarpGradient"

	virtual void ApplySmoothingToGradient(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) = 0;
	virtual float ApplyWarpUpdateToWarp(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) = 0;

	virtual void ApplyWarpFieldToLive(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                  ITMScene<TVoxelLive, TIndex>* liveScene)= 0;
	virtual void ApplyWarpUpdateToLive(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                   ITMScene<TVoxelLive, TIndex>* liveScene) = 0;


	virtual void AllocateNewCanonicalHashBlocks(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                            ITMScene<TVoxelLive, TIndex>* liveScene) = 0;

	virtual void ClearOutWarps(ITMScene<TVoxelCanonical, TIndex>* canonicalScene) = 0;


//============================= MEMBER VARIABLES =======================================================================
	Parameters parameters;

	float maxVectorUpdateThresholdVoxels;

	unsigned int iteration = 0;
	unsigned int trackedFrameCount = 0;
	const int startTrackingAfterFrame = 0;


	ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>* sceneLogger;
	std::string baseOutputDirectory;
	std::ofstream energy_stat_file;

	// variables for extra logging/analysis
	bool hasFocusCoordinates = false;
	Vector3i focusCoordinates = Vector3i(0);

	//_DEBUG
	bool restrictZtrackingForDebugging = false;
	bool simpleSceneExperimentModeEnabled = false;
	//*** 2D visual debugging
	//TODO: these should be CLI parameters -Greg (GitHub:Algomorph)
	//TODO: recording & recording frame index should be CLI parameters -Greg (GitHub:Algomorph)
	bool rasterizeLive = false;
	bool rasterizeCanonical = false;
	bool rasterizeWarps = false;
	unsigned int rasterizationFrame = 1;
	const int focus_slice_radius = 3;

	ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex> rasterizer;
	cv::Mat blank;
	cv::Mat liveImgTemplate;

private:

	void InitializeTracking(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene,
	                        bool recordWarpUpdates);

	void PerformSingleOptimizationStep(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                   ITMScene<TVoxelLive, TIndex>*& liveScene,
	                                   bool recordWarpUpdates);
	void FinalizeTracking(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene,
	                      bool recordWarpUpdates);

	void InitializeUpdate2DImageLogging(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene, cv::Mat& blank,
			cv::Mat& liveImgTemplate, ITMLib::ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>& rasterizer);
	void LogWarpUpdateAs2DImage(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene,
			const cv::Mat& blank, const cv::Mat& liveImgTemplate,
			ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>& rasterizer);
	float maxVectorUpdate;
	bool inStepByStepProcessingMode = false;
	void PrintLiveSceneStatistics(ITMScene<TVoxelLive, TIndex>* scene, const char* desc);


};


}//namespace ITMLib



