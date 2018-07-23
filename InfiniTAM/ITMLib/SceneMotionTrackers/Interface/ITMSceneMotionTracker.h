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
#include "../../Utils/FileIO/ITMDynamicFusionLogger.h"
#include "../../Objects/Scene/ITMScene.h"
#include "../../Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.h"
#include "../../Utils/Visualization/ITMSceneSliceVisualizer2D.h"
#include "../../Utils/ITMLibSettings.h"
#include "../../Utils/FileIO/ITMSceneLogger.h"
#include "../../Utils/ITMPrintHelpers.h"
#include "../Shared/ITMSceneMotionTracker_Debug.h"

namespace ITMLib {


template<typename TVoxelCanonical, typename TVoxelLive>
inline
bool VoxelIsConsideredForTracking(TVoxelCanonical& voxelCanonical, TVoxelLive voxelLive, int sourceFieldIndex) {
	return voxelCanonical.flags == VOXEL_NONTRUNCATED || voxelLive.flag_values[sourceFieldIndex] == VOXEL_NONTRUNCATED;
};

//_DEBUG
// this seems to give more noise in the results (visually)
//template<typename TVoxelCanonical, typename TVoxelLive>
//inline
//bool VoxelIsConsideredForTracking(TVoxelCanonical& voxelCanonical, TVoxelLive voxelLive, int sourceFieldIndex){
//	return voxelLive.flag_values[sourceFieldIndex] == VOXEL_NONTRUNCATED;
//};

//TODO: write documentation block -Greg (Github: Algomorph)
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneMotionTracker {

public:
//============================= INNER DATA STRUCTURES ==================================================================
	struct Parameters {
		const float gradientDescentLearningRate;// = 0.1f;
		const float rigidityEnforcementFactor;// = 0.1f;
		const float weightDataTerm;
		const float weightSmoothnessTerm;// = 0.2f; //0.2 is default for SobolevFusion, 0.5 is default for KillingFusion
		const float weightLevelSetTerm;// = 0.2f;
		const float epsilon;// = 1e-5f;
		const float unity; // voxelSize / mu, i.e. voxelSize / [narrow-band half-width]
	};

	struct Switches {
		const bool enableDataTerm;
		const bool enableLevelSetTerm;
		const bool enableSmoothingTerm;
		const bool enableKillingTerm;
		const bool enableGradientSmoothing;
		const bool usePreviousUpdateVectorsForSmoothing;
	};
//============================= CONSTRUCTORS / DESTRUCTORS =============================================================
//TODO: write documentation block -Greg (Github: Algomorph)

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
					ITMLibSettings::Instance().enableGradientSmoothing,
					ITMLibSettings::Instance().usePreviousUpdateVectorsForSmoothing
			} {
		PrintSettings();
	}

	virtual ~ITMSceneMotionTracker() = default;
//============================= MEMBER FUNCTIONS =======================================================================
	virtual void
	CalculateWarpGradient(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		                      ITMScene<TVoxelLive, TIndex>* liveScene, int sourceFieldIndex,
		                      bool restrictZTrackingForDebugging) = 0;
	virtual void SmoothWarpGradient(
			ITMScene<TVoxelLive, TIndex>* liveScene,
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, int sourceFieldIndex) = 0;
	virtual float UpdateWarps(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                          ITMScene<TVoxelLive, TIndex>* liveScene, int sourceSdfIndex) = 0;
	virtual void ClearOutFramewiseWarp(ITMScene<TVoxelCanonical, TIndex>* canonicalScene) = 0;
	virtual void AddFramewiseWarpToWarp(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, bool clearFramewiseWarp) = 0;
	virtual void ResetWarps(ITMScene<TVoxelCanonical, TIndex>* canonicalScene) = 0;
//============================= MEMBER VARIABLES =======================================================================

	const Parameters parameters;
	const Switches switches;
protected:
	void PrintSettings() {
		std::cout << bright_cyan << "*** ITMSceneMotionTracker_CPU Settings: ***" << reset << std::endl;
		std::cout << "Data term enabled: " << printBool(this->switches.enableDataTerm) << std::endl;
		std::cout << "Smoothing term enabled: " << printBool(this->switches.enableSmoothingTerm) << std::endl;
		std::cout << "Level Set term enabled: " << printBool(this->switches.enableLevelSetTerm) << std::endl;
		std::cout << "Killing term enabled: " << printBool(this->switches.enableKillingTerm) << std::endl;
		std::cout << "Gradient smoothing enabled: " << printBool(this->switches.enableGradientSmoothing) << std::endl
		          << std::endl;

		std::cout << "Gradient descent learning rate: " << this->parameters.gradientDescentLearningRate << std::endl;
		std::cout << "Rigidity enforcement factor: " << this->parameters.rigidityEnforcementFactor << std::endl;
		std::cout << "Weight of the data term: " << this->parameters.weightDataTerm << std::endl;
		std::cout << "Weight of the smoothness term: " << this->parameters.weightSmoothnessTerm << std::endl;
		std::cout << "Weight of the level set term: " << this->parameters.weightLevelSetTerm << std::endl;
		std::cout << "Epsilon for the level set term: " << this->parameters.epsilon << std::endl;
		std::cout << "Unity scaling factor: " << this->parameters.unity << std::endl;
		std::cout << bright_cyan << "*** *********************************** ***" << reset << std::endl;
	}

};


}//namespace ITMLib



