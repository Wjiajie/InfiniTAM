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
#define BOOST_TEST_MODULE WarpGradient_CPU_PVA
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif


//stdlib
#include <vector>
#include <atomic>

//boost
#include <boost/test/unit_test.hpp>

//local
#include "TestUtils.h"
#include "Test_WarpGradient_Common.h"
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTrackerInterface.h"
#include "../ITMLib/SurfaceTrackers/CPU/SurfaceTracker_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Traversal/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h"

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

typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CPU, ITMPlainVoxelArray> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CPU_PVA, DataFixture) {

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CPU1(&settings->scene_parameters,
	                                                            settings->swappingMode ==
	                                                            Configuration::SWAPPINGMODE_ENABLED,
	                                                            MEMORYDEVICE_CPU,
	                                                            indexParameters);
	ManipulationEngine_CPU_PVA_Warp::Inst().ResetScene(&warp_field_CPU1);


	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>();


	TimeIt([&]() {
		motionTracker_PVA_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warp Gradient - PVA CPU data term");


	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 36627u);

	float tolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CPU_PVA, DataFixture) {
	settings->enableGradientSmoothing = false;
	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>();
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_copy(*warp_field_data_term,
	                                                            MemoryDeviceType::MEMORYDEVICE_CPU);

	AlteredGradientCountFunctor<ITMWarp> agcFunctor;
	ITMSceneTraversalEngine<ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_copy, agcFunctor);
	BOOST_REQUIRE_EQUAL(agcFunctor.count.load(), 36627u);


	float maxWarp = motionTracker_PVA_CPU->UpdateWarps(canonical_volume, live_volume, &warp_field_copy);
	BOOST_REQUIRE_CLOSE(maxWarp, 0.0870865062f, 1e-7);

	AlteredFlowWarpCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_copy, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 36627u);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_copy, warp_field_iter0, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testSmoothWarpGradient_CPU_PVA, DataFixture) {
	settings->enableGradientSmoothing = true;

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CPU1(*warp_field_data_term, MEMORYDEVICE_CPU);


	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>();

	TimeIt([&]() {
		motionTracker_PVA_CPU->SmoothWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Smooth Warp Gradient - PVA CPU");
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_smoothed_");

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term_smoothed, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerm_CPU_PVA, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = true;
	settings->enableLevelSetTerm = false;
	settings->enableKillingConstraintInSmoothingTerm = false;

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>();


	TimeIt([&]() {
		motionTracker_PVA_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warp Gradient - PVA CPU data + tikhonov term");


	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 42417);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_tikhonov_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndKillingTerm_CPU_PVA, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = true;
	settings->enableLevelSetTerm = false;
	settings->enableKillingConstraintInSmoothingTerm = true;
	settings->sceneTrackingRigidityEnforcementFactor = 0.1;

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>();


	TimeIt([&]() {
		motionTracker_PVA_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warp Gradient - PVA CPU data term + tikhonov term");


	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 42670);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_killing_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerm_CPU_PVA, DataFixture) {
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = false;
	settings->enableLevelSetTerm = true;
	settings->enableKillingConstraintInSmoothingTerm = false;

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>();


	TimeIt([&]() {
		motionTracker_PVA_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warp Gradient - PVA CPU data term + tikhonov term");


	AlteredGradientCountFunctor<ITMWarp> functor;
	ITMSceneTraversalEngine<ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 41275);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_level_set_term, tolerance));
}
