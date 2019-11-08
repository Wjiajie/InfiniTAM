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
#define BOOST_TEST_MODULE WarpGradient_CPU_VBH
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
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTrackerInterface.h"
#include "../ITMLib/SurfaceTrackers/CPU/ITMSceneMotionTracker_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
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
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = false;
	settings->enableLevelSetTerm = false;
	settings->enableKillingConstraintInSmoothingTerm = false;
//	settings->SetFocusCoordinates(Vector3i(-21,-4,189));
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU1(&settings->sceneParams,
	                                                            settings->swappingMode ==
	                                                            Configuration::SWAPPINGMODE_ENABLED,
	                                                            MEMORYDEVICE_CPU, indexParameters);
	ManipulationEngine_CPU_VBH_Warp::Inst().ResetScene(&warp_field_CPU1);


	auto motionTracker_VBH_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>();


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1, false);
	}, "Calculate Warp Gradient - VBH CPU data term");

	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CPU1), 366);

	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 36618u);

//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_data_");
	float tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CPU_VBH, DataFixture) {
	settings->enableGradientSmoothing = false;
	auto motionTracker_VBH_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>();
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_copy(*warp_field_data_term,
	                                                            MemoryDeviceType::MEMORYDEVICE_CPU);

	AlteredGradientCountFunctor<ITMWarp> agcFunctor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_copy, agcFunctor);
	BOOST_REQUIRE_EQUAL(agcFunctor.count.load(), 36618u);
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_copy), 366);


	float maxWarp = motionTracker_VBH_CPU->UpdateWarps(canonical_volume, live_volume, &warp_field_copy);
//	warp_field_copy.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warp_iter0");
	BOOST_REQUIRE_CLOSE(maxWarp, 0.0213166066f, 1e-7f);


	AlteredFlowWarpCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_copy, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 36618u);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_copy, warp_field_iter0, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testSmoothWarpGradient_CPU_VBH, DataFixture) {
	settings->enableGradientSmoothing = true;

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU1(*warp_field_data_term, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>();

	TimeIt([&]() {
		motionTracker_VBH_CPU->SmoothWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Smooth Warp Gradient - VBH CPU");
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_smoothed_");

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term_smoothed, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CPU_VBH, DataFixture){
	settings->enableDataTerm = false;
	settings->enableSmoothingTerm = true;
	settings->enableLevelSetTerm = false;
	settings->enableKillingConstraintInSmoothingTerm = false;

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>();
	Vector3i testPosition(-40, 60, 200);
	settings->SetFocusCoordinates(testPosition);

	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1, false);
	}, "Calculate Warp Gradient - VBH CPU data term + tikhonov term");
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CPU1), 366);
	//warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warp_field_1_tikhonov_");


	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 42493);


	ITMWarp warp1 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(&warp_field_CPU1, testPosition);
	ITMWarp warp2 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(warp_field_tikhonov_term, testPosition);
	float tolerance = 1e-8;
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, tolerance);

	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_tikhonov_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerm_CPU_VBH, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = true;
	settings->enableLevelSetTerm = false;
	settings->enableKillingConstraintInSmoothingTerm = false;

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>();

	Vector3i testPosition(-40, 60, 200);
	settings->SetFocusCoordinates(testPosition);

	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1, false);
	}, "Calculate Warp Gradient - VBH CPU data term + tikhonov term");
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CPU1), 366);
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_tikhonov_");


	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 42493);


	ITMWarp warp1 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(&warp_field_CPU1, testPosition);
	ITMWarp warp2 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(warp_field_data_and_tikhonov_term, testPosition);
	float tolerance = 1e-8;
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, tolerance);

	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_tikhonov_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndKillingTerm_CPU_VBH, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = true;
	settings->enableLevelSetTerm = false;
	settings->enableKillingConstraintInSmoothingTerm = true;
	settings->sceneTrackingRigidityEnforcementFactor = 0.1;

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>();


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1, false);
	}, "Calculate Warp Gradient - VBH CPU data term + tikhonov term");
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_killing_");

	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 42757);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_killing_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerm_CPU_VBH, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = false;
	settings->enableLevelSetTerm = true;
	settings->enableKillingConstraintInSmoothingTerm = false;

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>();


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1, false);
	}, "Calculate Warp Gradient - VBH CPU data term + tikhonov term");
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_level_set_");

	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 41377);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_level_set_term, tolerance));
}