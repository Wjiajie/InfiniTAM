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
#include "../../Objects/Scene/ITMVoxelVolume.h"
#include "../../Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.h"
#include "../../Utils/Visualization/ITMSceneSliceVisualizer2D.h"
#include "../../Utils/ITMLibSettings.h"
#include "../../Utils/FileIO/ITMSceneLogger.h"
#include "../../Utils/ITMPrintHelpers.h"
#include "../../Utils/ITMVoxelFlags.h"
#include "../Shared/ITMSceneMotionOptimizationParameters.h"

namespace ITMLib {

//TODO: rename to "ITMSurfaceTracker" or "SurfaceTracker"
/**
 * \brief Class responsible for tracking motion of rigid or dynamic surfaces within the scene
 * \tparam TVoxel TSDF voxel type
 * \tparam TWarp Warp vector voxel type
 * \tparam TIndex Indexing structure type used for voxel volumes
 */
template<typename TVoxel, typename TWarp, typename TIndex>
class ITMSceneMotionTracker {

public:

//============================= CONSTRUCTORS / DESTRUCTORS =============================================================
//TODO: write documentation block -Greg (Github: Algomorph)


//TODO: reorder argument lists in both classes for consistency with reconstruction engine: warp field should come first,
//  canonical (as the "target") should come last
	explicit ITMSceneMotionTracker() :
			parameters{
					ITMLibSettings::Instance().sceneTrackingGradientDescentLearningRate,
					ITMLibSettings::Instance().sceneTrackingRigidityEnforcementFactor,
					ITMLibSettings::Instance().sceneTrackingWeightDataTerm,
					ITMLibSettings::Instance().sceneTrackingWeightSmoothingTerm,
					ITMLibSettings::Instance().sceneTrackingWeightLevelSetTerm,
					ITMLibSettings::Instance().sceneTrackingLevelSetTermEpsilon,
					ITMLibSettings::Instance().sceneParams.voxelSize / ITMLibSettings::Instance().sceneParams.mu
			},
			switches{
					ITMLibSettings::Instance().enableDataTerm,
					ITMLibSettings::Instance().enableLevelSetTerm,
					ITMLibSettings::Instance().enableSmoothingTerm,
					ITMLibSettings::Instance().enableKillingTerm,
					ITMLibSettings::Instance().enableGradientSmoothing
			} {
		PrintSettings();
	}

	virtual ~ITMSceneMotionTracker() = default;
//============================= MEMBER FUNCTIONS =======================================================================
	virtual void
	CalculateWarpGradient(ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
	                      ITMVoxelVolume<TVoxel, TIndex>* liveScene,
	                      ITMVoxelVolume<TWarp, TIndex>* warpField,
	                      bool restrictZTrackingForDebugging) = 0;
	virtual void SmoothWarpGradient(
			ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
			ITMVoxelVolume<TVoxel, TIndex>* liveScene,
			ITMVoxelVolume<TWarp, TIndex>* warpField) = 0;
	virtual float UpdateWarps(ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
	                          ITMVoxelVolume<TVoxel, TIndex>* liveScene,
	                          ITMVoxelVolume<TWarp, TIndex>* warpField) = 0;
	virtual void ClearOutFlowWarp(ITMVoxelVolume<TWarp, TIndex>* warpField) = 0;
	virtual void AddFlowWarpToWarp(
			ITMVoxelVolume<TWarp, TIndex>* warpField, bool clearFlowWarp) = 0;
	virtual void ResetWarps(ITMVoxelVolume<TWarp, TIndex>* warpField) = 0;
//============================= MEMBER VARIABLES =======================================================================

	const ITMSceneMotionOptimizationParameters parameters;
	const ITMSceneMotionOptimizationSwitches switches;
protected:
	void PrintSettings() {
		std::cout << bright_cyan << "*** Scene Motion Tracker Settings: ***" << reset << std::endl;
		std::cout << "Data term enabled: " << printBool(this->switches.enableDataTerm) << std::endl;
		std::cout << "Smoothing term enabled: " << printBool(this->switches.enableSmoothingTerm) << std::endl;
		std::cout << "Level Set term enabled: " << printBool(this->switches.enableLevelSetTerm) << std::endl;
		std::cout << "Killing term enabled: " << printBool(this->switches.enableKillingTerm) << std::endl;
		std::cout << "Gradient smoothing enabled: " << printBool(this->switches.enableGradientSmoothing) << std::endl
		          << std::endl;

		std::cout << "Gradient descent learning rate: " << this->parameters.gradientDescentLearningRate << std::endl;
		std::cout << "Rigidity enforcement factor: " << this->parameters.rigidityEnforcementFactor << std::endl;
		std::cout << "Weight of the data term: " << this->parameters.weightDataTerm << std::endl;
		std::cout << "Weight of the smoothness term: " << this->parameters.weightSmoothingTerm << std::endl;
		std::cout << "Weight of the level set term: " << this->parameters.weightLevelSetTerm << std::endl;
		std::cout << "Epsilon for the level set term: " << this->parameters.epsilon << std::endl;
		std::cout << "Unity scaling factor: " << this->parameters.unity << std::endl;
		std::cout << bright_cyan << "*** *********************************** ***" << reset << std::endl;
	}

};


}//namespace ITMLib



