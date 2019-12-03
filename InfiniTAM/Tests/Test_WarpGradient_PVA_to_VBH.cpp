//  ================================================================
//  Created by Gregory Kramida on 11/7/19.
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

#define BOOST_TEST_MODULE WarpGradient_VBH_to_PVA
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <unordered_map>

//boost
#include <boost/test/unit_test.hpp>

//local
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"

#include "../ITMLib/Engines/Reconstruction/CPU/ITMDynamicSceneReconstructionEngine_CPU.h"

#include "../ITMLib/Engines/Reconstruction/CUDA/ITMDynamicSceneReconstructionEngine_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"

#include "../ITMLib/Engines/Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"


#include "TestUtils.h"
#include "TestUtilsForSnoopyFrames16And17.h"
#include "../ITMLib/Objects/RenderStates/ITMRenderStateFactory.h"

using namespace ITMLib;

typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray> RecoEngine_CUDA_PVA;
typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash> RecoEngine_CUDA_VBH;


//TODO: move the construction checks into Test_ConstructVolumeFromImage test files as appropriate 

enum GenericWarpTestMode {
	SAVE_SUCCESSIVE_ITERATIONS,
	SAVE_FINAL_ITERATION_AND_FUSION,
	TEST_SUCCESSIVE_ITERATIONS,
	TEST_FINAL_ITERATION_AND_FUSION
};


std::string get_path_warps(std::string prefix, int iteration) {
	return "TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_" + std::to_string(iteration) + "_";
}

std::string get_path_warped_live(std::string prefix, int iteration) {
	return get_path_warps(prefix, iteration) + "warped_live_";
}

std::string get_path_fused(std::string prefix, int iteration) {
	return get_path_warps(prefix, iteration) + "fused_";
}


unsigned int switches_to_int_code(const SlavchevaSurfaceTracker::Switches& switches) {
	unsigned int code = 0;
	code |= static_cast<unsigned int>(switches.enableDataTerm) << 0u;
	code |= static_cast<unsigned int>(switches.enableLevelSetTerm) << 1u;
	code |= static_cast<unsigned int>(switches.enableSmoothingTerm) << 2u;
	code |= static_cast<unsigned int>(switches.enableKillingRigidityEnforcementTerm) << 3u;
	code |= static_cast<unsigned int>(switches.enableSobolevGradientSmoothing) << 4u;
	return code;
}

std::string switches_to_prefix(const SlavchevaSurfaceTracker::Switches& switches) {
	static std::unordered_map<unsigned int, std::string> prefix_by_switches_map = {
			{switches_to_int_code(SlavchevaSurfaceTracker::Switches(true, false, false, false, false)), "data_only"},
			{switches_to_int_code(SlavchevaSurfaceTracker::Switches(true, false, true, false,
			                                                        false)),                            "data_tikhonov"},
			{switches_to_int_code(SlavchevaSurfaceTracker::Switches(true, false, true, false, true)),
			                                                                                            "data_tikhonov_sobolev"}
	};
	return prefix_by_switches_map[switches_to_int_code(switches)];
}

template<typename TIndex>
void
GenericWarpConsistencySubtest_CPU(const SlavchevaSurfaceTracker::Switches& switches,
                                  int iteration_limit = 10,
                                  GenericWarpTestMode mode = TEST_SUCCESSIVE_ITERATIONS,
                                  float absolute_tolerance = 1e-7, bool allocateLiveFromBothImages = false) {

	std::string prefix = switches_to_prefix(switches);
	if (iteration_limit < 2) {
		DIEWITHEXCEPTION_REPORTLOCATION("Iteration limit must be at least 2");
	}

	ITMVoxelVolume<ITMWarp, TIndex> warp_field_CPU(&Configuration::get().scene_parameters,
	                                               Configuration::get().swapping_mode ==
	                                               Configuration::SWAPPINGMODE_ENABLED,
	                                               MEMORYDEVICE_CPU,
	                                               Frame16And17Fixture::InitParams<TIndex>());

	ITMVoxelVolume<ITMVoxel, TIndex>* volume_16;
	ITMView* view = nullptr;
	buildSdfVolumeFromImage(&volume_16, &view,
	                        "TestData/snoopy_depth_000016.png",
	                        "TestData/snoopy_color_000016.png",
	                        "TestData/snoopy_omask_000016.png",
	                        "TestData/snoopy_calib.txt",
	                        MEMORYDEVICE_CPU,
	                        Frame16And17Fixture::InitParams<TIndex>());

	ITMVoxelVolume<ITMVoxel, TIndex>* volume_17;
	initializeVolume(&volume_17, Frame16And17Fixture::InitParams<TIndex>(), MEMORYDEVICE_CPU);
	Vector2i imageSize(640, 480);
	ITMRenderState* renderState = ITMRenderStateFactory<TIndex>::CreateRenderState(imageSize,
	                                                                               &Configuration::get().scene_parameters,
	                                                                               MEMORYDEVICE_CPU,
	                                                                               volume_17->index);
	ITMTrackingState trackingState(imageSize, MEMORYDEVICE_CPU);
	if (allocateLiveFromBothImages) {
		ITMIndexingEngine<ITMVoxel, TIndex, MEMORYDEVICE_CPU>::Instance().AllocateFromDepth(volume_17, view,
		                                                                                    &trackingState,
		                                                                                    renderState, false, false);
	}
	updateView("TestData/snoopy_depth_000017.png",
	           "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
	           "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU, &view);
	ITMIndexingEngine<ITMVoxel, TIndex, MEMORYDEVICE_CPU>::Instance().AllocateFromDepth(volume_17, view, &trackingState,
	                                                                                    renderState, false, false);
	ITMDynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, TIndex>* reconstructionEngine =
			ITMDynamicSceneReconstructionEngineFactory
			::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, TIndex>(MEMORYDEVICE_CPU);

	reconstructionEngine->IntegrateIntoScene(volume_17, view, &trackingState, renderState);
//	buildSdfVolumeFromImage(&volume_17, "TestData/snoopy_depth_000017.png",
//	                        "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
//	                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU,
//	                        Fixture::InitParams<TIndex>());

	ITMSceneManipulationEngine_CPU<ITMWarp, TIndex>::Inst().ResetScene(&warp_field_CPU);

	SurfaceTracker<ITMVoxel, ITMWarp, TIndex, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker_CPU(switches);

	motionTracker_CPU.CalculateWarpGradient(volume_16, volume_17, &warp_field_CPU);
	motionTracker_CPU.SmoothWarpGradient(volume_16, volume_17, &warp_field_CPU);
	motionTracker_CPU.UpdateWarps(volume_16, volume_17, &warp_field_CPU);

	ITMVoxelVolume<ITMWarp, TIndex> warp_field_CPU2(&Configuration::get().scene_parameters,
	                                                Configuration::get().swapping_mode ==
	                                                Configuration::SWAPPINGMODE_ENABLED,
	                                                MEMORYDEVICE_CPU,
	                                                Frame16And17Fixture::InitParams<TIndex>());
	ITMSceneManipulationEngine_CPU<ITMWarp, TIndex>::Inst().ResetScene(&warp_field_CPU2);
	switch (mode) {
		case SAVE_SUCCESSIVE_ITERATIONS:
			warp_field_CPU.SaveToDirectory(std::string("../../Tests/") + get_path_warps(prefix, 0));
			break;
		case TEST_SUCCESSIVE_ITERATIONS:
			warp_field_CPU2.LoadFromDirectory(get_path_warps(prefix, 0));
			BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU, &warp_field_CPU2, absolute_tolerance));
			break;
		default:
			break;
	}

	ITMDynamicSceneReconstructionEngine_CPU<ITMVoxel, ITMWarp, TIndex> recoEngine;

	ITMVoxelVolume<ITMVoxel, TIndex>* warped_volumes[2] = {
			new ITMVoxelVolume<ITMVoxel, TIndex>(&Configuration::get().scene_parameters,
			                                     Configuration::get().swapping_mode ==
			                                     Configuration::SWAPPINGMODE_ENABLED,
			                                     MEMORYDEVICE_CPU,
			                                     Frame16And17Fixture::InitParams<TIndex>()),
			new ITMVoxelVolume<ITMVoxel, TIndex>(&Configuration::get().scene_parameters,
			                                     Configuration::get().swapping_mode ==
			                                     Configuration::SWAPPINGMODE_ENABLED,
			                                     MEMORYDEVICE_CPU,
			                                     Frame16And17Fixture::InitParams<TIndex>())
	};
	ITMSceneManipulationEngine_CPU<ITMVoxel, TIndex>::Inst().ResetScene(warped_volumes[0]);
	ITMSceneManipulationEngine_CPU<ITMVoxel, TIndex>::Inst().ResetScene(warped_volumes[1]);

	recoEngine.WarpScene_FlowWarps(&warp_field_CPU, volume_17, warped_volumes[1]);
	switch (mode) {
		case SAVE_SUCCESSIVE_ITERATIONS:
			warped_volumes[1]->SaveToDirectory(std::string("../../Tests/") + get_path_warped_live(prefix, 0));
			break;
		case TEST_SUCCESSIVE_ITERATIONS:
			ITMSceneManipulationEngine_CPU<ITMVoxel, TIndex>::Inst().ResetScene(volume_17);
			volume_17->LoadFromDirectory(get_path_warped_live(prefix, 0));
			BOOST_REQUIRE(contentAlmostEqual_CPU(volume_17, warped_volumes[1], absolute_tolerance));
			break;
		default:
			break;
	}

	int source_warped_field_ix = 0;
	int target_warped_field_ix = 0;
	for (int iteration = 1; iteration < iteration_limit; iteration++) {
		source_warped_field_ix = iteration % 2;
		target_warped_field_ix = (iteration + 1) % 2;
		motionTracker_CPU.CalculateWarpGradient(volume_16, warped_volumes[source_warped_field_ix], &warp_field_CPU);
		motionTracker_CPU.SmoothWarpGradient(volume_16, warped_volumes[source_warped_field_ix], &warp_field_CPU);
		motionTracker_CPU.UpdateWarps(volume_16, warped_volumes[source_warped_field_ix], &warp_field_CPU);
		recoEngine.WarpScene_FlowWarps(&warp_field_CPU, warped_volumes[source_warped_field_ix],
		                               warped_volumes[target_warped_field_ix]);
		std::string path = get_path_warps(prefix, iteration);
		std::string path_warped_live = get_path_warped_live(prefix, iteration);
		switch (mode) {
			case SAVE_SUCCESSIVE_ITERATIONS:
				warped_volumes[target_warped_field_ix]->SaveToDirectory(std::string("../../Tests/") + path_warped_live);
				warp_field_CPU.SaveToDirectory(std::string("../../Tests/") + path);
				break;
			case TEST_SUCCESSIVE_ITERATIONS:
				ITMSceneManipulationEngine_CPU<ITMWarp, TIndex>::Inst().ResetScene(&warp_field_CPU2);
				warp_field_CPU2.LoadFromDirectory(path);
				BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU, &warp_field_CPU2, absolute_tolerance));
				ITMSceneManipulationEngine_CPU<ITMVoxel, TIndex>::Inst().ResetScene(volume_17);
				volume_17->LoadFromDirectory(path_warped_live);
				BOOST_REQUIRE(
						contentAlmostEqual_CPU(volume_17, warped_volumes[target_warped_field_ix], absolute_tolerance));
				break;
			default:
				break;
		}
	}
	switch (mode) {
		case SAVE_FINAL_ITERATION_AND_FUSION:
			warp_field_CPU.SaveToDirectory(std::string("../../Tests/") + get_path_warps(prefix, iteration_limit - 1));
			warped_volumes[target_warped_field_ix]->SaveToDirectory(
					std::string("../../Tests/") + get_path_warped_live(prefix, iteration_limit - 1));
			recoEngine.FuseLiveIntoCanonicalSdf(volume_16, warped_volumes[target_warped_field_ix]);
			volume_16->SaveToDirectory(std::string("../../Tests/") + get_path_fused(prefix, iteration_limit - 1));
			break;
		case TEST_FINAL_ITERATION_AND_FUSION:
			ITMSceneManipulationEngine_CPU<ITMWarp, TIndex>::Inst().ResetScene(&warp_field_CPU2);
			warp_field_CPU2.LoadFromDirectory(get_path_warps(prefix, iteration_limit - 1));
			BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU, &warp_field_CPU2, absolute_tolerance));
			warped_volumes[source_warped_field_ix]->LoadFromDirectory(
					get_path_warped_live(prefix, iteration_limit - 1));
			BOOST_REQUIRE(contentAlmostEqual_CPU(warped_volumes[target_warped_field_ix],
			                                     warped_volumes[source_warped_field_ix], absolute_tolerance));
			recoEngine.FuseLiveIntoCanonicalSdf(volume_16, warped_volumes[target_warped_field_ix]);
			warped_volumes[source_warped_field_ix]->LoadFromDirectory(get_path_fused(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentAlmostEqual_CPU(volume_16, warped_volumes[source_warped_field_ix], absolute_tolerance));
			break;
		default:
			break;
	}

	delete volume_16;
	delete volume_17;
	delete warped_volumes[0];
	delete warped_volumes[1];
}


template<typename TIndex>
void
GenericWarpConsistencySubtest_CUDA(const SlavchevaSurfaceTracker::Switches& switches,
                                   int iteration_limit = 10,
                                   GenericWarpTestMode mode = TEST_SUCCESSIVE_ITERATIONS,
                                   float absolute_tolerance = 1e-7) {
	std::string prefix = switches_to_prefix(switches);
	if (iteration_limit < 2) {
		DIEWITHEXCEPTION_REPORTLOCATION("Iteration limit must be at least 2");
	}

	ITMVoxelVolume<ITMWarp, TIndex> warp_field_CUDA(&Configuration::get().scene_parameters,
	                                                Configuration::get().swapping_mode ==
	                                                Configuration::SWAPPINGMODE_ENABLED,
	                                                MEMORYDEVICE_CUDA,
	                                                Frame16And17Fixture::InitParams<TIndex>());

	ITMVoxelVolume<ITMVoxel, TIndex>* volume_16;
	buildSdfVolumeFromImage(&volume_16, "TestData/snoopy_depth_000016.png",
	                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
	                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
	                        Frame16And17Fixture::InitParams<TIndex>());

	ITMVoxelVolume<ITMVoxel, TIndex>* volume_17;
	buildSdfVolumeFromImage(&volume_17, "TestData/snoopy_depth_000017.png",
	                        "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
	                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
	                        Frame16And17Fixture::InitParams<TIndex>());

	ITMSceneManipulationEngine_CUDA<ITMWarp, TIndex>::Inst().ResetScene(&warp_field_CUDA);

	SurfaceTracker<ITMVoxel, ITMWarp, TIndex, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker_CUDA(switches);
	motionTracker_CUDA.CalculateWarpGradient(volume_16, volume_17, &warp_field_CUDA);
	motionTracker_CUDA.SmoothWarpGradient(volume_16, volume_17, &warp_field_CUDA);
	motionTracker_CUDA.UpdateWarps(volume_16, volume_17, &warp_field_CUDA);

	ITMVoxelVolume<ITMWarp, TIndex> warp_field_CUDA2(&Configuration::get().scene_parameters,
	                                                 Configuration::get().swapping_mode ==
	                                                 Configuration::SWAPPINGMODE_ENABLED,
	                                                 MEMORYDEVICE_CUDA,
	                                                 Frame16And17Fixture::InitParams<TIndex>());

	switch (mode) {
		case SAVE_SUCCESSIVE_ITERATIONS:
			warp_field_CUDA.SaveToDirectory(std::string("../../Tests/") + get_path_warps(prefix, 0));
			break;
		case TEST_SUCCESSIVE_ITERATIONS:
			ITMSceneManipulationEngine_CUDA<ITMWarp, TIndex>::Inst().ResetScene(&warp_field_CUDA2);
			warp_field_CUDA2.LoadFromDirectory(get_path_warps(prefix, 0));
			BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA, &warp_field_CUDA2, absolute_tolerance));
			break;
		default:
			break;
	}

	ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, TIndex> recoEngine;

	ITMVoxelVolume<ITMVoxel, TIndex>* warped_volumes[2] = {
			new ITMVoxelVolume<ITMVoxel, TIndex>(&Configuration::get().scene_parameters,
			                                     Configuration::get().swapping_mode ==
			                                     Configuration::SWAPPINGMODE_ENABLED,
			                                     MEMORYDEVICE_CUDA,
			                                     Frame16And17Fixture::InitParams<TIndex>()),
			new ITMVoxelVolume<ITMVoxel, TIndex>(&Configuration::get().scene_parameters,
			                                     Configuration::get().swapping_mode ==
			                                     Configuration::SWAPPINGMODE_ENABLED,
			                                     MEMORYDEVICE_CUDA,
			                                     Frame16And17Fixture::InitParams<TIndex>())
	};
	ITMSceneManipulationEngine_CUDA<ITMVoxel, TIndex>::Inst().ResetScene(warped_volumes[0]);
	ITMSceneManipulationEngine_CUDA<ITMVoxel, TIndex>::Inst().ResetScene(warped_volumes[1]);

	recoEngine.WarpScene_FlowWarps(&warp_field_CUDA, volume_17, warped_volumes[1]);
	int source_warped_field_ix = 0;
	int target_warped_field_ix = 0;
	for (int iteration = 1; iteration < iteration_limit; iteration++) {
		source_warped_field_ix = iteration % 2;
		target_warped_field_ix = (iteration + 1) % 2;

		motionTracker_CUDA.CalculateWarpGradient(volume_16, warped_volumes[source_warped_field_ix], &warp_field_CUDA);
		motionTracker_CUDA.SmoothWarpGradient(volume_16, warped_volumes[source_warped_field_ix], &warp_field_CUDA);
		motionTracker_CUDA.UpdateWarps(volume_16, warped_volumes[source_warped_field_ix], &warp_field_CUDA);
		recoEngine.WarpScene_FlowWarps(&warp_field_CUDA, warped_volumes[source_warped_field_ix],
		                               warped_volumes[target_warped_field_ix]);
		switch (mode) {
			case SAVE_SUCCESSIVE_ITERATIONS:
				warp_field_CUDA.SaveToDirectory(std::string("../../Tests/") + get_path_warps(prefix, iteration));
				break;
			case TEST_SUCCESSIVE_ITERATIONS:
				ITMSceneManipulationEngine_CUDA<ITMWarp, TIndex>::Inst().ResetScene(&warp_field_CUDA2);
				warp_field_CUDA2.LoadFromDirectory(get_path_warps(prefix, iteration));
				BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA, &warp_field_CUDA2, absolute_tolerance));
				break;
			default:
				break;
		}
	}

	switch (mode) {
		case SAVE_FINAL_ITERATION_AND_FUSION:
			warp_field_CUDA.SaveToDirectory(std::string("../../Tests/") + get_path_warps(prefix, iteration_limit - 1));
			warped_volumes[target_warped_field_ix]->SaveToDirectory(
					std::string("../../Tests/") + get_path_warped_live(prefix, iteration_limit - 1));
			recoEngine.FuseLiveIntoCanonicalSdf(volume_16, warped_volumes[target_warped_field_ix]);
			volume_16->SaveToDirectory(std::string("../../Tests/") + get_path_fused(prefix, iteration_limit - 1));
			break;
		case TEST_FINAL_ITERATION_AND_FUSION:
			ITMSceneManipulationEngine_CUDA<ITMWarp, TIndex>::Inst().ResetScene(&warp_field_CUDA2);
			warp_field_CUDA2.LoadFromDirectory(get_path_warps(prefix, iteration_limit - 1));
			BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA, &warp_field_CUDA2, absolute_tolerance));
			warped_volumes[source_warped_field_ix]->LoadFromDirectory(
					get_path_warped_live(prefix, iteration_limit - 1));
			BOOST_REQUIRE(contentAlmostEqual_CUDA(warped_volumes[target_warped_field_ix],
			                                      warped_volumes[source_warped_field_ix], absolute_tolerance));
			recoEngine.FuseLiveIntoCanonicalSdf(volume_16, warped_volumes[target_warped_field_ix]);
			warped_volumes[source_warped_field_ix]->LoadFromDirectory(get_path_fused(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentAlmostEqual_CUDA(volume_16, warped_volumes[source_warped_field_ix], absolute_tolerance));
			break;
		default:
			break;
	}

	delete volume_16;
	delete volume_17;
	delete warped_volumes[0];
	delete warped_volumes[1];
}

///CAUTION: SAVE modes require the build directory to be immediately inside the root source directory.
void
GenericWarpTest_CUDA(const SlavchevaSurfaceTracker::Switches& switches, int iteration_limit = 10,
                     GenericWarpTestMode mode = TEST_SUCCESSIVE_ITERATIONS) {

	float absoluteTolerance = 1e-7;
	GenericWarpConsistencySubtest_CUDA<ITMPlainVoxelArray>(switches, iteration_limit, mode, absoluteTolerance);
	GenericWarpConsistencySubtest_CUDA<ITMVoxelBlockHash>(switches, iteration_limit, mode, absoluteTolerance);

	std::string prefix = switches_to_prefix(switches);

	switch (mode) {
		case TEST_SUCCESSIVE_ITERATIONS: {

			ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CUDA_PVA(&Configuration::get().scene_parameters,
			                                                                Configuration::get().swapping_mode ==
			                                                                Configuration::SWAPPINGMODE_ENABLED,
			                                                                MEMORYDEVICE_CUDA,
			                                                                Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
			ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CUDA_VBH(&Configuration::get().scene_parameters,
			                                                               Configuration::get().swapping_mode ==
			                                                               Configuration::SWAPPINGMODE_ENABLED,
			                                                               MEMORYDEVICE_CUDA,
			                                                               Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());

			for (int iteration = 0; iteration < iteration_limit; iteration++) {
				warp_field_CUDA_PVA.LoadFromDirectory(get_path_warps(prefix, iteration));
				warp_field_CUDA_VBH.LoadFromDirectory(get_path_warps(prefix, iteration));
				BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA_Verbose(&warp_field_CUDA_PVA, &warp_field_CUDA_VBH,
				                                                       absoluteTolerance));
			}
		}
			break;
		case TEST_FINAL_ITERATION_AND_FUSION: {
			ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> volume_PVA(&Configuration::get().scene_parameters,
			                                                        Configuration::get().swapping_mode ==
			                                                        Configuration::SWAPPINGMODE_ENABLED,
			                                                        MEMORYDEVICE_CUDA,
			                                                        Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
			ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> volume_VBH(&Configuration::get().scene_parameters,
			                                                       Configuration::get().swapping_mode ==
			                                                       Configuration::SWAPPINGMODE_ENABLED,
			                                                       MEMORYDEVICE_CUDA,
			                                                       Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());
			ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetScene(&volume_VBH);
			volume_PVA.LoadFromDirectory(get_path_warped_live(prefix, iteration_limit - 1));
			volume_VBH.LoadFromDirectory(get_path_warped_live(prefix, iteration_limit - 1));
			BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(&volume_PVA, &volume_VBH, absoluteTolerance));
			volume_PVA.LoadFromDirectory(get_path_fused(prefix, iteration_limit - 1));
			volume_VBH.LoadFromDirectory(get_path_fused(prefix, iteration_limit - 1));
			BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(&volume_PVA, &volume_VBH, absoluteTolerance));
		}
			break;
		default:
			break;
	}
}


///CAUTION: SAVE modes require the build directory to be immediately inside the root source directory.
void
GenericWarpTest_CPU(const SlavchevaSurfaceTracker::Switches& switches, int iteration_limit = 10,
                    GenericWarpTestMode mode = TEST_SUCCESSIVE_ITERATIONS) {

	std::string prefix = switches_to_prefix(switches);
	float absoluteTolerance = 1e-7;
	GenericWarpConsistencySubtest_CPU<ITMPlainVoxelArray>(switches, iteration_limit, mode, absoluteTolerance);
	GenericWarpConsistencySubtest_CPU<ITMVoxelBlockHash>(switches, iteration_limit, mode, absoluteTolerance);

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> volume_PVA(&Configuration::get().scene_parameters,
	                                                        Configuration::get().swapping_mode ==
	                                                        Configuration::SWAPPINGMODE_ENABLED,
	                                                        MEMORYDEVICE_CPU,
	                                                        Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> volume_VBH(&Configuration::get().scene_parameters,
	                                                       Configuration::get().swapping_mode ==
	                                                       Configuration::SWAPPINGMODE_ENABLED,
	                                                       MEMORYDEVICE_CPU,
	                                                       Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());
	switch (mode) {
		case TEST_SUCCESSIVE_ITERATIONS: {

			ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CPU_PVA(&Configuration::get().scene_parameters,
			                                                               Configuration::get().swapping_mode ==
			                                                               Configuration::SWAPPINGMODE_ENABLED,
			                                                               MEMORYDEVICE_CPU,
			                                                               Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
			ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU_VBH(&Configuration::get().scene_parameters,
			                                                              Configuration::get().swapping_mode ==
			                                                              Configuration::SWAPPINGMODE_ENABLED,
			                                                              MEMORYDEVICE_CPU,
			                                                              Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());

			for (int iteration = 0; iteration < iteration_limit; iteration++) {
				std::cout << "Testing iteration " << iteration << std::endl;
				warp_field_CPU_PVA.LoadFromDirectory(get_path_warps(prefix, iteration));
				ManipulationEngine_CPU_VBH_Warp::Inst().ResetScene(&warp_field_CPU_VBH);
				warp_field_CPU_VBH.LoadFromDirectory(get_path_warps(prefix, iteration));
				BOOST_REQUIRE(allocatedContentAlmostEqual_CPU_Verbose(&warp_field_CPU_PVA, &warp_field_CPU_VBH,
				                                                      absoluteTolerance));
				ManipulationEngine_CPU_VBH_Voxel::Inst().ResetScene(&volume_VBH);
				volume_PVA.LoadFromDirectory(get_path_warped_live(prefix, iteration));
				volume_VBH.LoadFromDirectory(get_path_warped_live(prefix, iteration));
				BOOST_REQUIRE(contentForFlagsAlmostEqual_CPU(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED,
				                                             absoluteTolerance));
			}
		}
			break;
		case TEST_FINAL_ITERATION_AND_FUSION: {

			ManipulationEngine_CPU_VBH_Voxel::Inst().ResetScene(&volume_VBH);
			volume_PVA.LoadFromDirectory(get_path_warped_live(prefix, iteration_limit - 1));
			volume_VBH.LoadFromDirectory(get_path_warped_live(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentForFlagsAlmostEqual_CUDA(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED, absoluteTolerance));
			volume_PVA.LoadFromDirectory(get_path_fused(prefix, iteration_limit - 1));
			volume_VBH.LoadFromDirectory(get_path_fused(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentForFlagsAlmostEqual_CUDA(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED, absoluteTolerance));
		}
			break;
		default:
			break;
	}
}


BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataTermOnly_CPU) {
	Configuration::get().telemetry_settings.focus_coordinates_specified = true;
	Configuration::get().telemetry_settings.focus_coordinates = Vector3i(-15, 6, 170);
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	GenericWarpTest_CPU(switches, 2, TEST_SUCCESSIVE_ITERATIONS);
}


BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataTermOnly_CUDA) {
//	Configuration::get().telemetry_settings.focus_coordinates_specified = true;
//	Configuration::get().telemetry_settings.focus_coordinates = Vector3i(-24, 63, 240);
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	GenericWarpTest_CUDA(switches, 2, TEST_SUCCESSIVE_ITERATIONS);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonov) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, false);
	GenericWarpTest_CPU(switches, 2, TEST_SUCCESSIVE_ITERATIONS);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, true);
	GenericWarpTest_CPU(switches, 2, TEST_SUCCESSIVE_ITERATIONS);
}


BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_Fusion) {
	GenericWarpTest_CUDA(SlavchevaSurfaceTracker::Switches(true, false, true, false, true), 100,
	                     GenericWarpTestMode::SAVE_FINAL_ITERATION_AND_FUSION);
}


void Warp_PVA_VBH_simple_CPU_subtest(int iteration, SlavchevaSurfaceTracker::Switches trackerSwitches) {

	if (iteration < 0) {
		DIEWITHEXCEPTION_REPORTLOCATION("Expecting iteration >= 0, got less than that, aborting.");
	}
	std::string path_frame_17_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_17_";
	std::string path_frame_16_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_16_";
	std::string path_frame_17_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_";
	std::string path_frame_16_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_16_";

	std::string prefix = switches_to_prefix(trackerSwitches);
	float absoluteTolerance = 1e-7;

	// *** initialize/load warps
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warps_PVA;
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warps_VBH;
	if (iteration > 0) {
		std::string path_warps = get_path_warps(prefix, iteration - 1);
		loadVolume(&warps_PVA, path_warps, MEMORYDEVICE_CPU,
		           Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
		loadVolume(&warps_VBH, path_warps, MEMORYDEVICE_CPU,
		           Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());
		BOOST_REQUIRE(allocatedContentAlmostEqual_CPU(warps_PVA, warps_VBH, absoluteTolerance));
	} else {
		initializeVolume(&warps_PVA, Frame16And17Fixture::InitParams<ITMPlainVoxelArray>(), MEMORYDEVICE_CPU);
		initializeVolume(&warps_VBH, Frame16And17Fixture::InitParams<ITMVoxelBlockHash>(), MEMORYDEVICE_CPU);
		BOOST_REQUIRE(allocatedContentAlmostEqual_CPU(warps_PVA, warps_VBH, absoluteTolerance));
	}

	// *** load warped live scene



	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* warped_live_PVA;
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* warped_live_VBH;

	if (iteration > 0) {
		std::string path_warped_live = get_path_warped_live(prefix, iteration - 1);
		loadVolume(&warped_live_PVA, path_warped_live, MEMORYDEVICE_CPU,
		           Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
		loadVolume(&warped_live_VBH, path_warped_live, MEMORYDEVICE_CPU,
		           Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());
	} else {
		loadVolume(&warped_live_PVA, path_frame_17_PVA, MEMORYDEVICE_CPU,
		           Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
		loadVolume(&warped_live_VBH, path_frame_17_VBH, MEMORYDEVICE_CPU,
		           Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());
	}

	//_DEBUG
	Vector3i test_pos(-57, -9, 195);
	Configuration::get().telemetry_settings.focus_coordinates_specified = true;
	Configuration::get().telemetry_settings.focus_coordinates = test_pos;
	ITMVoxel voxelPVA = ITMSceneManipulationEngine_CPU<ITMVoxel, ITMPlainVoxelArray>::Inst()
			.ReadVoxel(warped_live_PVA,test_pos);
	voxelPVA.print_self();
	ITMVoxel voxelVBH = ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash>::Inst()
			.ReadVoxel(warped_live_VBH, test_pos);
	voxelVBH.print_self();

	// *** load canonical volume as the two different data structures
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume_16_PVA;
	loadVolume(&volume_16_PVA, path_frame_16_PVA, MEMORYDEVICE_CPU,
	           Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume_16_VBH;
	loadVolume(&volume_16_VBH, path_frame_16_VBH, MEMORYDEVICE_CPU,
	           Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());

	// *** perform the warp gradient computation and warp updates


	SurfaceTracker<ITMVoxel, ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker_CPU_PVA(trackerSwitches);

	motionTracker_CPU_PVA.CalculateWarpGradient(volume_16_PVA, warped_live_PVA, warps_PVA);
	motionTracker_CPU_PVA.SmoothWarpGradient(volume_16_PVA, warped_live_PVA, warps_PVA);
	motionTracker_CPU_PVA.UpdateWarps(volume_16_PVA, warped_live_PVA, warps_PVA);

	SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker_CPU_VBH(trackerSwitches);

	motionTracker_CPU_VBH.CalculateWarpGradient(volume_16_VBH, warped_live_VBH, warps_VBH);
	motionTracker_CPU_VBH.SmoothWarpGradient(volume_16_VBH, warped_live_VBH, warps_VBH);
	motionTracker_CPU_VBH.UpdateWarps(volume_16_VBH, warped_live_VBH, warps_VBH);

	// *** test content

	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU_Verbose(warps_PVA, warps_VBH, absoluteTolerance));


	//_DEBUG
	ITMWarp warpPVA = ITMSceneManipulationEngine_CPU<ITMWarp, ITMPlainVoxelArray>::Inst()
			.ReadVoxel(warps_PVA, test_pos);
	warpPVA.print_self();
	ITMWarp warpVBH = ITMSceneManipulationEngine_CPU<ITMWarp, ITMVoxelBlockHash>::Inst()
			.ReadVoxel(warps_VBH, test_pos);
	warpVBH.print_self();

	delete volume_16_PVA;
	delete volume_16_VBH;
	delete warped_live_PVA;
	delete warped_live_VBH;

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* loaded_warps_PVA;
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* loaded_warps_VBH;
	std::string path_loaded_warps = get_path_warps(prefix, iteration);
	loadVolume(&loaded_warps_PVA, path_loaded_warps, MEMORYDEVICE_CPU,
	           Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
	loadVolume(&loaded_warps_VBH, path_loaded_warps, MEMORYDEVICE_CPU,
	           Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());

	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(warps_PVA, loaded_warps_PVA, absoluteTolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(warps_VBH, loaded_warps_VBH, absoluteTolerance));

	delete warps_PVA;
	delete warps_VBH;
	delete loaded_warps_PVA;
	delete loaded_warps_VBH;
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CPU) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	Warp_PVA_VBH_simple_CPU_subtest(0, switches);
}