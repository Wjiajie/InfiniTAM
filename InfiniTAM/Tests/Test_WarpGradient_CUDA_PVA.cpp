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
#include "Test_WarpGradient_Common.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/SceneFileIO/ITMSceneFileIOEngine.h"
#include "../ITMLib/Engines/Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"
#include "../ITMLib/SceneMotionTrackers/Interface/ITMSceneMotionTracker.h"
#include "../ITMLib/SceneMotionTrackers/CUDA/ITMSceneMotionTracker_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"

using namespace ITMLib;

typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CUDA, ITMPlainVoxelArray> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CUDA, DataFixture) {

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CUDA1(&settings->sceneParams,
	                                                            settings->swappingMode ==
	                                                            ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                            MEMORYDEVICE_CUDA,
	                                                            sizeSlice, offsetSlice);
	ManipulationEngine_CUDA_PVA_Warp::Inst().ResetScene(&warp_field_CUDA1);


	auto motionTracker_PVA_CUDA = new ITMSceneMotionTracker_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray>();


	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CUDA1, false);
	}, "Calculate Warp Gradient - PVA CPU data term");

	float tolerance = 1e-5;
	loadWarpFieldDataTerm();
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_data_term, tolerance));
	clearWarpFieldDataTerm();
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CUDA, DataFixture) {
	settings->enableGradientSmoothing = false;
	auto motionTracker_PVA_CUDA = new ITMSceneMotionTracker_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray>();
	loadWarpFieldDataTerm();
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_copy(*warp_field_data_term,
	                                                            MemoryDeviceType::MEMORYDEVICE_CUDA);
	clearWarpFieldDataTerm();

	float maxWarp = motionTracker_PVA_CUDA->UpdateWarps(canonical_volume, live_volume, &warp_field_copy);
	BOOST_REQUIRE_CLOSE(maxWarp, 0.0870865062f, 1e-7);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_copy, warp_field_iter0, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CUDA, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = true;
	settings->enableLevelSetTerm = false;
	settings->enableKillingTerm = false;

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);


	auto motionTracker_PVA_CUDA = new ITMSceneMotionTracker_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray>();


	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CUDA1, false);
	}, "Calculate Warp Gradient - PVA CPU data term + Tikhonov term");


	float tolerance = 1e-8;
	loadWarpFieldTikhonovTerm();
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_tikhonov_term, tolerance));
	clearWarpFieldTikhonovTerm();
}


BOOST_FIXTURE_TEST_CASE(testKillingTerm_CUDA, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = true;
	settings->enableLevelSetTerm = false;
	settings->enableKillingTerm = true;
	settings->sceneTrackingRigidityEnforcementFactor = 0.1;

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);


	auto motionTracker_PVA_CUDA = new ITMSceneMotionTracker_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray>();


	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CUDA1, false);
	}, "Calculate Warp Gradient - PVA CPU data term + Killing term");


	float tolerance = 1e-8;
	loadWarpFieldKillingTerm();
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_killing_term, tolerance));
	clearWarpFieldKillingTerm();
}


BOOST_FIXTURE_TEST_CASE(testLevelSetTerm_CUDA, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = false;
	settings->enableLevelSetTerm = true;
	settings->enableKillingTerm = false;

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);


	auto motionTracker_PVA_CUDA = new ITMSceneMotionTracker_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray>();


	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(canonical_volume,
		                                              live_volume, &warp_field_CUDA1, false);
	}, "Calculate Warp Gradient - PVA CPU data term + level set term");


	float tolerance = 1e-7;
	loadWarpFieldLevelSetTerm();
	BOOST_REQUIRE(contentAlmostEqual_CUDA_Verbose(&warp_field_CUDA1, warp_field_level_set_term, tolerance));
	clearWarpFieldLevelSetTerm();
}