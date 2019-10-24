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
#define BOOST_TEST_MODULE WarpGradient_CUDA_VBH
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
#include "Test_WarpGradient_Common.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/SceneFileIO/ITMSceneFileIOEngine.h"
#include "../ITMLib/Engines/Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"
#include "../ITMLib/SceneMotionTrackers/Interface/ITMSceneMotionTracker.h"
#include "../ITMLib/SceneMotionTrackers/CUDA/ITMSceneMotionTracker_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"

using namespace ITMLib;


typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CPU, ITMVoxelBlockHash> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CUDA_VBH, DataFixture) {
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CUDA1(&settings->sceneParams,
	                                                            settings->swappingMode ==
	                                                            ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                            MEMORYDEVICE_CUDA,
	                                                            sizeSlice, offsetSlice);
	ManipulationEngine_CUDA_VBH_Warp::Inst().ResetScene(&warp_field_CUDA1);


	auto motionTracker_VBH_CUDA = new ITMSceneMotionTracker_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash>();

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> canonical_volume_CUDA(*canonical_volume, MEMORYDEVICE_CUDA);
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> live_volume_CUDA(*live_volume, MEMORYDEVICE_CUDA);

	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(&canonical_volume_CUDA, &live_volume_CUDA, &warp_field_CUDA1, false);
	}, "Calculate Warp Gradient - PVA CPU data term");

	BOOST_REQUIRE_EQUAL(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CUDA1), 366);

	//warp_field_CUDA1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_data_");
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU1(warp_field_CUDA1, MEMORYDEVICE_CPU);
	float tolerance = 1e-7;
	loadWarpFieldDataTerm();
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term, tolerance));
	clearWarpFieldDataTerm();
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CUDA_VBH, DataFixture) {
	settings->enableGradientSmoothing = false;
	auto motionTracker_VBH_CUDA = new ITMSceneMotionTracker_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash>();
	clearWarpFieldIter0();
	loadWarpFieldDataTerm();
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_copy(*warp_field_data_term,
	                                                            MemoryDeviceType::MEMORYDEVICE_CUDA);
	clearWarpFieldDataTerm();

	BOOST_REQUIRE_EQUAL(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_copy), 366);


	float maxWarp = motionTracker_VBH_CUDA->UpdateWarps(canonical_volume, live_volume, &warp_field_copy);
	//warp_field_copy.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warp_iter0");
	BOOST_REQUIRE_CLOSE(maxWarp, 0.0213166066f, 1e-7f);


	float tolerance = 1e-8;
	loadWarpFieldIter0();
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_copy, warp_field_iter0, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CUDA_VBH, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = true;
	settings->enableLevelSetTerm = false;
	settings->enableKillingTerm = false;

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);


	auto motionTracker_VBH_CUDA = new ITMSceneMotionTracker_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash>();


	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CUDA1, false);
	}, "Calculate Warp Gradient - PVA CPU data term + tikhonov term");
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CUDA1), 366);
	//warp_field_CUDA1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_tikhonov_");


	float tolerance = 1e-8;
	loadWarpFieldTikhonovTerm();
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CUDA1, warp_field_tikhonov_term, tolerance));
	clearWarpFieldTikhonovTerm();
}


BOOST_FIXTURE_TEST_CASE(testKillingTerm_CUDA_VBH, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = true;
	settings->enableLevelSetTerm = false;
	settings->enableKillingTerm = true;
	settings->sceneTrackingRigidityEnforcementFactor = 0.1;

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);


	auto motionTracker_VBH_CUDA = new ITMSceneMotionTracker_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash>();


	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CUDA1, false);
	}, "Calculate Warp Gradient - PVA CPU data term + tikhonov term");
	//warp_field_CUDA1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_killing_");

	float tolerance = 1e-8;
	loadWarpFieldKillingTerm();
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CUDA1, warp_field_killing_term, tolerance));
	clearWarpFieldKillingTerm();
}


BOOST_FIXTURE_TEST_CASE(testLevelSetTerm_CUDA_VBH, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = false;
	settings->enableLevelSetTerm = true;
	settings->enableKillingTerm = false;

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);


	auto motionTracker_VBH_CUDA = new ITMSceneMotionTracker_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash>();


	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CUDA1, false);
	}, "Calculate Warp Gradient - PVA CPU data term + tikhonov term");
	//warp_field_CUDA1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_level_set_");


	float tolerance = 1e-8;
	loadWarpFieldLevelSetTerm();
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CUDA1, warp_field_level_set_term, tolerance));
	clearWarpFieldLevelSetTerm();
}
