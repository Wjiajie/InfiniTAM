//  ================================================================
//  Created by Gregory Kramida on 11/3/17.
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
#include "TestUtils.h"


#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Objects/Scene/ITMSceneManipulation.h"
#include "../ITMLib/Utils/ITMSceneLogger.h"
#include "../ITMLib/Utils/ITMSceneStatisticsCalculator.h"

using namespace ITMLib;

void GenerateAndLogKillingScene01() {
	auto settings = new ITMLibSettings();
	auto canonicalScene = new ITMScene<ITMVoxelCanonical, ITMVoxelIndex>(
			&settings->sceneParams, settings->swappingMode ==
			                        ITMLibSettings::SWAPPINGMODE_ENABLED, settings->GetMemoryType());
	auto liveScene = new ITMScene<ITMVoxelLive, ITMVoxelIndex>(
			&settings->sceneParams, settings->swappingMode ==
			                        ITMLibSettings::SWAPPINGMODE_ENABLED, settings->GetMemoryType());

	const std::string testScenePath = "/media/algomorph/Data/Reconstruction/debug_output/test_scene";
	GenerateTestScene01(*canonicalScene);
	Vector3i offset(5, 0, 0);
	CopySceneWithOffset_CPU(*liveScene, *canonicalScene,offset);


	ITMSceneLogger<ITMVoxelCanonical,ITMVoxelLive,ITMVoxelIndex> logger(testScenePath,canonicalScene,liveScene);
	logger.SaveScenesCompact();

	//TODO: fix with new highlight system --Greg (GitHub:Algomorph)
//	logger.LogHighlight(0,0,1,0);
//	logger.LogHighlight(0,0,1,1);
//	logger.LogHighlight(0,0,1,2);
//	logger.LogHighlight(0,0,1,4);
//	logger.LogHighlight(0,0,1,7);
//	logger.LogHighlight(0,0,1,8);
	logger.SaveHighlights();
	logger.StartSavingWarpState(0);
	logger.SetUpInterestRegionsForSaving();
	logger.SaveCurrentWarpState();
	logger.SaveAllInterestRegionWarps();
	Vector3f iterationIncrement(0.1,0,0);
	int iterationCount = static_cast<int>(offset.x / iterationIncrement.x);
	for(int iteration = 0; iteration < iterationCount; iteration++){
		OffsetWarps(*canonicalScene,iterationIncrement);
		logger.SaveCurrentWarpState();
		logger.SaveAllInterestRegionWarps();
	}
	logger.StopSavingWarpState();


	delete canonicalScene;
	delete liveScene;
	delete settings;
}
