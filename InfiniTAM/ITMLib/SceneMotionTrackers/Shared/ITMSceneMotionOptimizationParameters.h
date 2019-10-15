//  ================================================================
//  Created by Gregory Kramida on 10/10/19.
//  Copyright (c) 2019 Gregory Kramida
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

//============================= PARAMETER/SWITCH DATA STRUCTURES FOR PRECONFIGURED OPTIMIZATION SETTINGS ===============
namespace ITMLib{
	struct ITMSceneMotionOptimizationParameters {
		const float gradientDescentLearningRate;// = 0.1f;
		const float rigidityEnforcementFactor;// = 0.1f;
		const float weightDataTerm;
		const float weightSmoothingTerm;// = 0.2f; //0.2 is default for SobolevFusion, 0.5 is default for KillingFusion
		const float weightLevelSetTerm;// = 0.2f;
		const float epsilon;// = 1e-5f;
		const float unity; // voxelSize / mu, i.e. 1 / [narrow-band half-width in voxels] or [voxel size in metric units] / [narrow-band half-width in metric units]
	};

	struct ITMSceneMotionOptimizationSwitches {
		const bool enableDataTerm;
		const bool enableLevelSetTerm;
		const bool enableSmoothingTerm;
		const bool enableKillingTerm;
		const bool enableGradientSmoothing;
	};
} // namespace ITMLib


