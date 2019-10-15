//  ================================================================
//  Created by Gregory Kramida on 10/15/19.
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
#define BOOST_TEST_MODULE SceneConstruction
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif


//stdlib
#include <random>
#include <vector>
#include <chrono>

//boost
#include <boost/test/unit_test.hpp>

//local
#include "TestUtils.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/SceneFileIO/ITMSceneFileIOEngine.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/SceneMotionTrackers/Interface/ITMSceneMotionTracker.h"
#include "../ITMLib/SceneMotionTrackers/CPU/ITMSceneMotionTracker_CPU.h"
#include "../ITMLib/SceneMotionTrackers/Shared/ITMCalculateWarpGradientFunctor.h"
#include "../ITMLib/SceneMotionTrackers/Shared/ITMWarpGradientFunctors.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"

using namespace ITMLib;

typedef ITMSceneFileIOEngine<ITMVoxel, ITMPlainVoxelArray> SceneFileIOEngine_PVA;
typedef ITMSceneFileIOEngine<ITMVoxel, ITMVoxelBlockHash> SceneFileIOEngine_VBH;

BOOST_AUTO_TEST_CASE(testDataTerm_CPU) {
	ITMLibSettings* settings = &ITMLibSettings::Instance();
	settings->deviceType = ITMLibSettings::DEVICE_CPU;
	settings->enableKillingTerm = false;
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = false;
	settings->enableGradientSmoothing = true;
	settings->enableLevelSetTerm = false;

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> scene1(&settings->sceneParams,
	                                                   settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                   settings->GetMemoryType());
	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetScene(&scene1);


	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> canonical_scene2(&settings->sceneParams,
	                                                              settings->swappingMode ==
	                                                              ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                              settings->GetMemoryType());
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetScene(&canonical_scene2);
	canonical_scene2.LoadFromDirectory("TestData/snoopy_result_frame_17_PVA/canonical");
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> live_scene2(&settings->sceneParams,
	                                                         settings->swappingMode ==
	                                                         ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                         settings->GetMemoryType());
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetScene(&live_scene2);
	canonical_scene2.LoadFromDirectory("TestData/snoopy_result_frame_17_PVA/live");
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field2(&settings->sceneParams,
	                                                        settings->swappingMode ==
	                                                        ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                        settings->GetMemoryType());
	ManipulationEngine_CPU_PVA_Warp::Inst().ResetScene(&warp_field2);

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field3(&settings->sceneParams,
	                                                        settings->swappingMode ==
	                                                        ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                        settings->GetMemoryType());
	ManipulationEngine_CPU_PVA_Warp::Inst().ResetScene(&warp_field2);


	auto motionTracker1 = new ITMSceneMotionTracker_CPU<ITMVoxel, ITMWarp,
			ITMCalculateWarpGradientFunctor<ITMVoxel, ITMWarp, ITMPlainVoxelArray::IndexData, ITMPlainVoxelArray::IndexCache>,
			ITMPlainVoxelArray>();

	auto motionTracker2 = new ITMSceneMotionTracker_CPU<ITMVoxel, ITMWarp,
			ITMSceneMotionEnergyGradientCompositeFunctor<ITMVoxel, ITMWarp, ITMPlainVoxelArray::IndexData, ITMPlainVoxelArray::IndexCache>,
			ITMPlainVoxelArray>();

	std::chrono::duration<double> elapsed;
	std::cout << "Basic functor gradient computation: " << std::endl;
	auto start = std::chrono::high_resolution_clock::now();
	motionTracker1->CalculateWarpGradient(&canonical_scene2, &live_scene2, &warp_field2, false);
	auto finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << "Elapsed time: " << elapsed.count() << " s\n";

	std::cout << "Composable functor gradient computation: " << std::endl;
	start = std::chrono::high_resolution_clock::now();
	motionTracker2->CalculateWarpGradient(&canonical_scene2, &live_scene2, &warp_field3, false);
	finish = std::chrono::high_resolution_clock::now();
	elapsed  = finish - start;
	std::cout << "Elapsed time: " << elapsed.count() << " s\n";

	float tolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field2, &warp_field3, tolerance));


	//motionTracker1->CalculateWarpGradient(&canonical_scene2, &live_scene2, &warp_field2, false);

	//std::string path = "TestData/snoopy_result_frame_17_VBH/canonical";
	//scene1.LoadFromDirectory(path);



}