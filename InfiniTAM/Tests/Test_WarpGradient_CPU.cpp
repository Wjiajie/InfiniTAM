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
#include <atomic>

//boost
#include <boost/test/unit_test.hpp>

//local
#include "TestUtils.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/SceneFileIO/ITMSceneFileIOEngine.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Engines/Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"
#include "../ITMLib/SceneMotionTrackers/Interface/ITMSceneMotionTracker.h"
#include "../ITMLib/SceneMotionTrackers/CPU/ITMSceneMotionTracker_CPU.h"
#include "../ITMLib/SceneMotionTrackers/CUDA/ITMSceneMotionTracker_CUDA.h"
#include "../ITMLib/SceneMotionTrackers/Shared/ITMCalculateWarpGradientFunctor.h"
#include "../ITMLib/SceneMotionTrackers/Shared/ITMLegacyCalculateWarpGradientFunctor.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
#include "../ITMLib/Engines/Traversal/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h"

using namespace ITMLib;

typedef ITMSceneFileIOEngine<ITMVoxel, ITMPlainVoxelArray> SceneFileIOEngine_PVA;
typedef ITMSceneFileIOEngine<ITMVoxel, ITMVoxelBlockHash> SceneFileIOEngine_VBH;



template <typename TVoxel>
struct AlteredGradientCountFunctor{
	AlteredGradientCountFunctor() : count(0){};
	void operator()(const TVoxel& voxel) {
		if(voxel.gradient0 != Vector3f(0.0f)){
			count.fetch_add(1u);
		}
	}

	std::atomic<unsigned int> count;
};

BOOST_AUTO_TEST_CASE(testDataTerm_CPU) {
	ITMLibSettings* settings = &ITMLibSettings::Instance();
	settings->deviceType = ITMLibSettings::DEVICE_CPU;
	settings->enableKillingTerm = false;
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = false;
	settings->enableGradientSmoothing = true;
	settings->enableLevelSetTerm = false;


	Vector3i offsetSlice(-64, -24, 168);
	//64+16=80; -24+72=96; 312-168=300-156=304-160=144
	Vector3i sizeSlice(80, 96, 144);

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> canonical_scene_CPU(&settings->sceneParams,
	                                                              settings->swappingMode ==
	                                                              ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                                 settings->GetMemoryType(),
	                                                                 sizeSlice, offsetSlice);
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetScene(&canonical_scene_CPU);

	canonical_scene_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/canonical");


	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> live_scene_CPU(&settings->sceneParams,
	                                                         settings->swappingMode ==
	                                                         ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                            settings->GetMemoryType(),
	                                                            sizeSlice, offsetSlice);
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetScene(&live_scene_CPU);
	live_scene_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/live");


	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CPU1(&settings->sceneParams,
	                                                        settings->swappingMode ==
	                                                        ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                            settings->GetMemoryType(),
	                                                            sizeSlice, offsetSlice);
	ManipulationEngine_CPU_PVA_Warp::Inst().ResetScene(&warp_field_CPU1);

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CPU2(&settings->sceneParams,
	                                                            settings->swappingMode ==
	                                                            ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                            settings->GetMemoryType(),
	                                                            sizeSlice, offsetSlice);
	ManipulationEngine_CPU_PVA_Warp::Inst().ResetScene(&warp_field_CPU2);
	warp_field_CPU2.LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/gradient0");

	auto motionTracker_PVA_CPU = new ITMSceneMotionTracker_CPU<ITMVoxel, ITMWarp, ITMPlainVoxelArray>();


	TimeIt([&](){
		motionTracker_PVA_CPU->CalculateWarpGradient(&canonical_scene_CPU, &live_scene_CPU, &warp_field_CPU1, false);
	}, "Calculate Warp Gradient - Basic");


	AlteredGradientCountFunctor<ITMWarp> functor;
	functor.count.store(0u);
	ITMSceneTraversalEngine<ITMWarp, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 36627u);

	float tolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, &warp_field_CPU2, tolerance));



}