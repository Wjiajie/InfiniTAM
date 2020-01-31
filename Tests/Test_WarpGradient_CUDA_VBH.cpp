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
#include <vector>
#include <atomic>

//boost
#include <boost/test/unit_test.hpp>

//local
#include "TestUtils.h"
#include "Test_WarpGradient_Common.h"
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTrackerInterface.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/SceneStatisticsCalculator_CUDA.h"


using namespace ITMLib;


typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CUDA, VoxelBlockHash> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CUDA_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_CUDA1(&configuration::get().general_voxel_volume_parameters,
	                                                            configuration::get().swapping_mode ==
	                                                            configuration::SWAPPINGMODE_ENABLED,
	                                                        MEMORYDEVICE_CUDA, indexParameters);
	ManipulationEngine_CUDA_VBH_Warp::Inst().ResetVolume(&warp_field_CUDA1);


	auto motionTracker_VBH_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, false, false, false));


	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CUDA1);
	}, "Calculate Warping Gradient - VBH CUDA data term");

	BOOST_REQUIRE_EQUAL(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CUDA1), 589);
//
//	TSDFVoxel voxel = ManipulationEngine_CUDA_VBH_Voxel::Inst().ReadVoxel(live_volume, Vector3i(-21, -5, 189));
//	std::cout << "SDF (-21, -5, 189): " << voxel.sdf << std::endl;

	//warp_field_CUDA1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_data_");

	float tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CUDA_Verbose(&warp_field_CUDA1, warp_field_data_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CUDA_VBH, DataFixture) {

	auto motionTracker_VBH_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, false, false, false));
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_copy(*warp_field_data_term,
	                                                       MemoryDeviceType::MEMORYDEVICE_CUDA);

	BOOST_REQUIRE_EQUAL(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_copy), 589);

	float maxWarp = motionTracker_VBH_CUDA->UpdateWarps(canonical_volume, live_volume, &warp_field_copy);
	//warp_field_copy.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warp_iter0");
	BOOST_REQUIRE_CLOSE(maxWarp, 0.18186526f, 1e-7f);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_copy, warp_field_iter0, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testSmoothWarpGradient_CUDA_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_CUDA1(*warp_field_data_term, MEMORYDEVICE_CUDA);
	auto motionTracker_VBH_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, false, false, true));


	TimeIt([&]() {
		motionTracker_VBH_CUDA->SmoothWarpGradient(canonical_volume, live_volume, &warp_field_CUDA1);
	}, "Smooth Warping Gradient - VBH CUDA");
//	warp_field_CUDA1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_smoothed_");

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_data_term_smoothed, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CUDA_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);

	auto motionTracker_VBH_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, true, false, false));


	Vector3i testPosition(-40, 60, 200);
	//settings->SetFocusCoordinates(testPosition);

	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CUDA1);
	}, "Calculate Warping Gradient - VBH CUDA tikhonov term");
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CUDA1), 589);
	//warp_field_CUDA1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warp_field_1_tikhonov_");

	WarpVoxel warp1 = ManipulationEngine_CUDA_VBH_Warp::Inst().ReadVoxel(&warp_field_CUDA1, testPosition);
	WarpVoxel warp2 = ManipulationEngine_CUDA_VBH_Warp::Inst().ReadVoxel(warp_field_tikhonov_term, testPosition);

	float relativeTolerance = 1.0f; //percent
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, relativeTolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, relativeTolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, relativeTolerance);

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_tikhonov_term, absoluteTolerance));
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerm_CUDA_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);


	auto motionTracker_VBH_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, true, false, false));

	Vector3i testPosition(-40, 60, 200);
	//settings->SetFocusCoordinates(testPosition);

	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CUDA1);
	}, "Calculate Warping Gradient - VBH CUDA data term + tikhonov term");
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeAllocatedHashBlockCount(&warp_field_CUDA1), 589);
	//warp_field_CUDA1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_tikhonov_");

	WarpVoxel warp1 = ManipulationEngine_CUDA_VBH_Warp::Inst().ReadVoxel(&warp_field_CUDA1, testPosition);
	WarpVoxel warp2 = ManipulationEngine_CUDA_VBH_Warp::Inst().ReadVoxel(warp_field_data_and_tikhonov_term, testPosition);
	float tolerance = 1e-6;
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, tolerance);

	BOOST_REQUIRE(contentAlmostEqual_CUDA_Verbose(&warp_field_CUDA1, warp_field_data_and_tikhonov_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndKillingTerm_CUDA_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);

	auto motionTracker_VBH_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, true, true, false));


	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CUDA1);
	}, "Calculate Warping Gradient - VBH CUDA data term + killing term");
	//warp_field_CUDA1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_killing_");

	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_data_and_killing_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerm_CUDA_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);

	auto motionTracker_VBH_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, true, false, false, false)
			);

	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CUDA1);
	}, "Calculate Warping Gradient - VBH CUDA data term + level set term");
	//warp_field_CUDA1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/gradient0_level_set_");


	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_data_and_level_set_term, tolerance));
}

#ifdef GENERATE_DATA
BOOST_AUTO_TEST_CASE(Test_WarpGradient_CUDA_VBH_GenerateTestData){
	GenerateTestData<VoxelBlockHash,MEMORYDEVICE_CUDA>();
}
#endif