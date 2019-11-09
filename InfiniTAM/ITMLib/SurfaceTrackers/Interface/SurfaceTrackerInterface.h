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
#include "../../Utils/FileIO/ITMSceneLogger.h"
#include "../../Utils/ITMPrintHelpers.h"
#include "../../Utils/ITMVoxelFlags.h"
#include "../Shared/ITMSceneMotionOptimizationParameters.h"

//boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;

namespace ITMLib {

//TODO: move to separate header
class SlavchevaSurfaceTracker {
public:

	enum ConfigurationMode{
		KILLING_FUSION,
		SOBOLEV_FUSION
	};

	struct Parameters {
		Parameters();
		explicit Parameters(ConfigurationMode mode, float unity);
		explicit Parameters(const po::variables_map& vm);
		const float gradientDescentLearningRate;// = 0.1f;
		const float rigidityEnforcementFactor;// = 0.1f;
		const float weightDataTerm;// = 1.0f
		const float weightSmoothingTerm;// = 0.2f; //0.2 is default for SobolevFusion, 0.5 is default for KillingFusion
		const float weightLevelSetTerm;// = 0.2f;
		const float epsilon;// = 1e-5f;
		const float unity; // voxelSize/mu, i.e. 1/[narrow-band half-width in voxels] or [voxel size in metric units]/[narrow-band half-width in metric units]
	private:
		Parameters(const po::variables_map& vm, ConfigurationMode mode, float unity);
	};

	struct Switches {
		Switches();
		explicit Switches(ConfigurationMode mode);
		explicit Switches(const po::variables_map& vm);
		const bool enableDataTerm;
		const bool enableLevelSetTerm;
		const bool enableSmoothingTerm;
		const bool enableKillingRigidityEnforcementTerm;
		const bool enableSobolevGradientSmoothing;
	private:
		Switches(const po::variables_map& vm, ConfigurationMode mode);
	};

	const Parameters parameters;
	const Switches switches;

	explicit SlavchevaSurfaceTracker();

private:
	void PrintSettings();
};




/**
 * \brief Class responsible for tracking motion of rigid or dynamic surfaces within the scene
 * \tparam TVoxel TSDF voxel type
 * \tparam TWarp Warp vector voxel type
 * \tparam TIndex Indexing structure type used for voxel volumes
 */
template<typename TVoxel, typename TWarp, typename TIndex>
class SurfaceTrackerInterface {

public:



//TODO: reorder argument lists in both classes for consistency with reconstruction engine: warp field should come first,
//  canonical (as the "target") should come last


	virtual ~SurfaceTrackerInterface() = default;
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


};

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class SurfaceTracker;

}//namespace ITMLib



