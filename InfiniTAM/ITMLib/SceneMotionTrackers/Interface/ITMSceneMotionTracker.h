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
#include "../../Utils/FileIO/ITMScene2DSliceLogger.h"
#include "../../Utils/ITMLibSettings.h"
#include "../../Utils/FileIO/ITMSceneLogger.h"

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
//TODO: simplify constructor to just accept the ITMLibSettings object and set the parameters from it

	explicit ITMSceneMotionTracker(const ITMLibSettings* settings) :
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
			} {}

	virtual ~ITMSceneMotionTracker() = default;

//============================= MEMBER FUNCTIONS =======================================================================


	/**
	 * \brief Warp canonical back to live
	 * \param canonicalScene
	 * \param liveScene
	 */
	virtual void
	WarpCanonicalToLive(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) = 0;
	virtual void CalculateWarpGradient(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                   ITMScene<TVoxelLive, TIndex>* liveScene) = 0;
	virtual void SmoothWarpGradient(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) = 0;
	virtual float ApplyWarpUpdateToWarp(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) = 0;
	virtual void ApplyWarpUpdateToLive(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                   ITMScene<TVoxelLive, TIndex>* liveScene) = 0;

	virtual void ClearOutWarps(ITMScene<TVoxelCanonical, TIndex>* canonicalScene) = 0;

//============================= MEMBER VARIABLES =======================================================================

	const Parameters parameters;
	const Switches switches;
};


}//namespace ITMLib



