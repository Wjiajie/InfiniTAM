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
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTrackerInterface.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/SceneStatisticsCalculator_CPU.h"
#include "../ITMLib/Engines/Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"

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
struct AlteredFramewiseWarpCountFunctor {
	AlteredFramewiseWarpCountFunctor() : count(0) {};

	void operator()(const TVoxel& voxel) {
		if (voxel.framewise_warp != Vector3f(0.0f)) {
			count.fetch_add(1u);
		}
	}

	std::atomic<unsigned int> count;
};


typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CPU, VoxelBlockHash> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CPU_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_CPU1(&configuration::get().general_voxel_volume_parameters,
	                                                           configuration::get().swapping_mode ==
	                                                           configuration::SWAPPINGMODE_ENABLED,
	                                                       MEMORYDEVICE_CPU, indexParameters);
	ManipulationEngine_CPU_VBH_Warp::Inst().ResetVolume(&warp_field_CPU1);


	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, false, false, false));


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warping Gradient - VBH CPU data term");

	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CPU1), 589);

	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 37525u);

//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_data_");
	float tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CPU_VBH, DataFixture) {

	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, false, false, false));
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_copy(*warp_field_data_term,
	                                                       MemoryDeviceType::MEMORYDEVICE_CPU);

	AlteredGradientCountFunctor<WarpVoxel> agcFunctor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_copy, agcFunctor);
	BOOST_REQUIRE_EQUAL(agcFunctor.count.load(), 37525u);
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_copy), 589);

	IndexingEngine<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::Instance().AllocateUsingOtherVolume(canonical_volume, live_volume);

	float maxWarp = motionTracker_VBH_CPU->UpdateWarps(canonical_volume, live_volume, &warp_field_copy);
//	warp_field_copy.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warp_iter0");
	BOOST_REQUIRE_CLOSE(maxWarp, 0.18186526f, 1e-7f);


	AlteredFramewiseWarpCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_copy, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 37525u);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_copy, warp_field_iter0, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testSmoothWarpGradient_CPU_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_CPU1(*warp_field_data_term, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, false, false, true)
	);

	TimeIt([&]() {
		motionTracker_VBH_CPU->SmoothWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Smooth Warping Gradient - VBH CPU");
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_smoothed_");

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term_smoothed, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CPU_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, true, false, false)
	);
	Vector3i testPosition(-40, 60, 200);

	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warping Gradient - VBH CPU data term + tikhonov term");
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CPU1), 589);
	//warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warp_field_1_tikhonov_");


	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 57413);


	WarpVoxel warp1 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(&warp_field_CPU1, testPosition);
	WarpVoxel warp2 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(warp_field_tikhonov_term, testPosition);
	float tolerance = 1e-8;
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, tolerance);

	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_tikhonov_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerm_CPU_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, true, false, false));

	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warping Gradient - VBH CPU data term + tikhonov term");
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CPU1), 589);
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_tikhonov_");


	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 57413);

	Vector3i testPosition(-40, 60, 200);
	WarpVoxel warp1 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(&warp_field_CPU1, testPosition);
	WarpVoxel warp2 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(warp_field_data_and_tikhonov_term, testPosition);
	float tolerance = 1e-8;
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, tolerance);

	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_tikhonov_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndKillingTerm_CPU_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, true, true, false));


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warping Gradient - VBH CPU data term + tikhonov term");
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_killing_");

	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 59083);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_killing_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerm_CPU_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);

	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, true, false, false, false));


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warping Gradient - VBH CPU data term + level set term");
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_level_set_");

	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 55369);

	float tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_level_set_term, tolerance));
}

//#define GENERATE_DATA
#ifdef GENERATE_DATA
BOOST_AUTO_TEST_CASE(Test_WarpGradient_CUDA_VBH_GenerateTestData){
	GenerateTestData<VoxelBlockHash,MEMORYDEVICE_CPU>();
}
#endif