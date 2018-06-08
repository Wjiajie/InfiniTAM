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
#include "../../Utils/Visualization/ITMScene2DSliceVisualizer.h"
#include "../../Utils/ITMLibSettings.h"
#include "../../Utils/FileIO/ITMSceneLogger.h"
#include "../../Utils/ITMPrintHelpers.h"
#include "../Shared/ITMSceneMotionTracker_Debug.h"

namespace ITMLib {


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
	};
//============================= CONSTRUCTORS / DESTRUCTORS =============================================================
//TODO: write documentation block -Greg (Github: Algomorph)

	explicit ITMSceneMotionTracker(const ITMLibSettings* settings, ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive,TIndex>& logger) :
			parameters{
					settings->sceneTrackingGradientDescentLearningRate,
					settings->sceneTrackingRigidityEnforcementFactor,
					settings->sceneTrackingWeightDataTerm,
					settings->sceneTrackingWeightSmoothingTerm,
					settings->sceneTrackingWeightLevelSetTerm,
					settings->sceneTrackingLevelSetTermEpsilon,
					settings->sceneParams.voxelSize / settings->sceneParams.mu
			},
			switches{
					settings->enableDataTerm,
					settings->enableLevelSetTerm,
					settings->enableSmoothingTerm,
					settings->enableKillingTerm,
					settings->enableGradientSmoothing
			} {
		PrintSettings();
	}

	virtual ~ITMSceneMotionTracker() = default;
//============================= MEMBER FUNCTIONS =======================================================================
	virtual void CalculateWarpGradient(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene,
	                                   bool hasFocusCoordinates, const Vector3i& focusCoordinates, int sourceFieldIndex,
	                                   bool restrictZTrackingForDebugging) = 0;
	virtual void SmoothWarpGradient(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene) = 0;
	virtual float UpdateWarps(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) = 0;
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



