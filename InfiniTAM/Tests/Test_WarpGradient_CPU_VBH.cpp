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
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/SceneMotionTrackers/Interface/ITMSceneMotionTracker.h"
#include "../ITMLib/SceneMotionTrackers/CPU/ITMSceneMotionTracker_CPU.h"
#include "../ITMLib/SceneMotionTrackers/Shared/ITMLegacyCalculateWarpGradientFunctor.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Traversal/CPU/ITMSceneTraversal_CPU_VoxelBlockHash.h"

using namespace ITMLib;

template<typename TVoxel>
struct AlteredGradientCountFunctor {
	AlteredGradientCountFunctor() : count(0) {};

	void operator()(const TVoxel& voxel) {
		if (voxel.gradient0 != Vector3f(0.0f)) {
			count.fetch_add(1u);
		}
	}

	std::atomic<unsigned int> count;
};

template<typename TVoxel>
struct AlteredFlowWarpCountFunctor {
	AlteredFlowWarpCountFunctor() : count(0) {};

	void operator()(const TVoxel& voxel) {
		if (voxel.flow_warp != Vector3f(0.0f)) {
			count.fetch_add(1u);
		}
	}

	std::atomic<unsigned int> count;
};

typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CPU, ITMVoxelBlockHash> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CPU_VBH, DataFixture) {

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU1(&settings->sceneParams,
	                                                            settings->swappingMode ==
	                                                            ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                            MEMORYDEVICE_CPU, indexParameters);
	ManipulationEngine_CPU_VBH_Warp::Inst().ResetScene(&warp_field_CPU1);


	auto motionTracker_VBH_CPU = new ITMSceneMotionTracker_CPU<ITMVoxel, ITMWarp, ITMVoxelBlockHash>();


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1, false);
	}, "Calculate Warp Gradient - PVA CPU data term");

	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CPU1), 366);

	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 36618u);

//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_data_");
	float tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CPU_VBH, DataFixture) {
	settings->enableGradientSmoothing = false;
	auto motionTracker_VBH_CPU = new ITMSceneMotionTracker_CPU<ITMVoxel, ITMWarp, ITMVoxelBlockHash>();
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_copy(*warp_field_data_term,
	                                                            MemoryDeviceType::MEMORYDEVICE_CPU);

	AlteredGradientCountFunctor<ITMWarp> agcFunctor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
	VoxelTraversal(&warp_field_copy, agcFunctor);
	BOOST_REQUIRE_EQUAL(agcFunctor.count.load(), 36618u);
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_copy), 366);


	float maxWarp = motionTracker_VBH_CPU->UpdateWarps(canonical_volume, live_volume, &warp_field_copy);
//	warp_field_copy.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warp_iter0");
	BOOST_REQUIRE_CLOSE(maxWarp, 0.0213166066f, 1e-7f);


	AlteredFlowWarpCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
	VoxelTraversal(&warp_field_copy, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 36618u);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_copy, warp_field_iter0, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CPU_VBH, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = true;
	settings->enableLevelSetTerm = false;
	settings->enableKillingTerm = false;

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new ITMSceneMotionTracker_CPU<ITMVoxel, ITMWarp, ITMVoxelBlockHash>();


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1, false);
	}, "Calculate Warp Gradient - PVA CPU data term + tikhonov term");
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CPU1), 366);
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_tikhonov_");


	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 42493);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_tikhonov_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testKillingTerm_CPU_VBH, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = true;
	settings->enableLevelSetTerm = false;
	settings->enableKillingTerm = true;
	settings->sceneTrackingRigidityEnforcementFactor = 0.1;

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new ITMSceneMotionTracker_CPU<ITMVoxel, ITMWarp, ITMVoxelBlockHash>();


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1, false);
	}, "Calculate Warp Gradient - PVA CPU data term + tikhonov term");
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_killing_");

	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 42757);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_killing_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testLevelSetTerm_CPU_VBH, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = false;
	settings->enableLevelSetTerm = true;
	settings->enableKillingTerm = false;

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new ITMSceneMotionTracker_CPU<ITMVoxel, ITMWarp, ITMVoxelBlockHash>();


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1, false);
	}, "Calculate Warp Gradient - PVA CPU data term + tikhonov term");
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_level_set_");

	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 41377);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_level_set_term, tolerance));
}
