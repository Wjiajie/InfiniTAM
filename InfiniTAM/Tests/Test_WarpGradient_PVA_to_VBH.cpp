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

#define BOOST_TEST_MODULE WarpScene
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//local
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"

#include "../ITMLib/Engines/Reconstruction/CPU/ITMDynamicSceneReconstructionEngine_CPU.h"

#include "../ITMLib/Engines/Reconstruction/CUDA/ITMDynamicSceneReconstructionEngine_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"


#include "TestUtils.h"

using namespace ITMLib;

typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray> RecoEngine_CUDA_PVA;
typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash> RecoEngine_CUDA_VBH;

struct Fixture {
	template<typename TIndex>
	static typename TIndex::InitializationParameters InitParams();
};

template<>
ITMPlainVoxelArray::InitializationParameters Fixture::InitParams<ITMPlainVoxelArray>() {
	return {Vector3i(80, 96, 248), Vector3i(-64, -24, 64)};
}

template<>
ITMVoxelBlockHash::InitializationParameters Fixture::InitParams<ITMVoxelBlockHash>() {
	return {1200, 0x20000};
}

BOOST_FIXTURE_TEST_CASE(Test_SceneConstruct16_PVA_VBH, Fixture) {

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume_PVA_16;
	buildSdfVolumeFromImage(&volume_PVA_16, "TestData/snoopy_depth_000016.png",
	                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
	                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
	                        InitParams<ITMPlainVoxelArray>());

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume_VBH_16;
	buildSdfVolumeFromImage(&volume_VBH_16, "TestData/snoopy_depth_000016.png",
	                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
	                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
	                        InitParams<ITMVoxelBlockHash>());

	std::cout << SceneStatCalc_CUDA_PVA_Voxel::Instance().FindMinimumNonTruncatedBoundingBox(volume_PVA_16)
	          << std::endl;
	std::cout << SceneStatCalc_CUDA_VBH_Voxel::Instance().FindMinimumNonTruncatedBoundingBox(volume_VBH_16)
	          << std::endl;

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(volume_PVA_16, volume_VBH_16, absoluteTolerance));

	delete volume_VBH_16;
	delete volume_PVA_16;
}

BOOST_FIXTURE_TEST_CASE(Test_SceneConstruct17_PVA_VBH, Fixture) {

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume_PVA_17;
	buildSdfVolumeFromImage(&volume_PVA_17, "TestData/snoopy_depth_000017.png",
	                        "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
	                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
	                        InitParams<ITMPlainVoxelArray>());

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume_VBH_17;
	buildSdfVolumeFromImage(&volume_VBH_17, "TestData/snoopy_depth_000017.png",
	                        "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
	                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
	                        InitParams<ITMVoxelBlockHash>());

	std::cout << SceneStatCalc_CUDA_PVA_Voxel::Instance().FindMinimumNonTruncatedBoundingBox(volume_PVA_17)
	          << std::endl;
	std::cout << SceneStatCalc_CUDA_VBH_Voxel::Instance().FindMinimumNonTruncatedBoundingBox(volume_VBH_17)
	          << std::endl;

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(volume_PVA_17, volume_VBH_17, absoluteTolerance));

	delete volume_VBH_17;
	delete volume_PVA_17;
}

enum GenericWarpTestMode {
	SAVE_SUCCESSIVE_ITERATIONS,
	SAVE_FINAL_ITERATION_AND_FUSION,
	TEST_SUCCESSIVE_ITERATIONS,
	TEST_FINAL_ITERATION_AND_FUSION
};

template<typename TIndex>
void
GenericWarpConsistencySubtest(const SlavchevaSurfaceTracker::Switches& switches, const std::string& prefix,
                              int iteration_limit = 10,
                              GenericWarpTestMode mode = TEST_SUCCESSIVE_ITERATIONS, float absolute_tolerance = 1e-7) {
	if (iteration_limit < 2) {
		DIEWITHEXCEPTION_REPORTLOCATION("Iteration limit must be at least 2");
	}
	std::string path;

	ITMVoxelVolume<ITMWarp, TIndex> warp_field_CUDA(&Configuration::get().scene_parameters,
	                                                Configuration::get().swapping_mode ==
	                                                Configuration::SWAPPINGMODE_ENABLED,
	                                                MEMORYDEVICE_CUDA,
	                                                Fixture::InitParams<TIndex>());

	ITMVoxelVolume<ITMVoxel, TIndex>* volume_16;
	buildSdfVolumeFromImage(&volume_16, "TestData/snoopy_depth_000016.png",
	                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
	                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
	                        Fixture::InitParams<TIndex>());

	ITMVoxelVolume<ITMVoxel, TIndex>* volume_17;
	buildSdfVolumeFromImage(&volume_17, "TestData/snoopy_depth_000017.png",
	                        "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
	                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
	                        Fixture::InitParams<TIndex>());

	ITMSceneManipulationEngine_CUDA<ITMWarp, TIndex>::Inst().ResetScene(&warp_field_CUDA);

	SurfaceTracker<ITMVoxel, ITMWarp, TIndex, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker_CUDA(
			SlavchevaSurfaceTracker::Switches(true, false, false, false, false)
	);
	motionTracker_CUDA.CalculateWarpGradient(volume_16, volume_17, &warp_field_CUDA);
	motionTracker_CUDA.SmoothWarpGradient(volume_16, volume_17, &warp_field_CUDA);
	motionTracker_CUDA.UpdateWarps(volume_16, volume_17, &warp_field_CUDA);

	ITMVoxelVolume<ITMWarp, TIndex> warp_field_CUDA2(&Configuration::get().scene_parameters,
	                                                 Configuration::get().swapping_mode ==
	                                                 Configuration::SWAPPINGMODE_ENABLED,
	                                                 MEMORYDEVICE_CUDA,
	                                                 Fixture::InitParams<TIndex>());
	ITMSceneManipulationEngine_CUDA<ITMWarp, TIndex>::Inst().ResetScene(&warp_field_CUDA2);
	path = "TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_0_";
	switch (mode) {
		case SAVE_SUCCESSIVE_ITERATIONS:
			warp_field_CUDA.SaveToDirectory(std::string("../../Tests/") + path);
			break;
		case TEST_SUCCESSIVE_ITERATIONS:
			warp_field_CUDA2.LoadFromDirectory(path);
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
			                                     Fixture::InitParams<TIndex>()),
			new ITMVoxelVolume<ITMVoxel, TIndex>(&Configuration::get().scene_parameters,
			                                     Configuration::get().swapping_mode ==
			                                     Configuration::SWAPPINGMODE_ENABLED,
			                                     MEMORYDEVICE_CUDA,
			                                     Fixture::InitParams<TIndex>())
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
		path = std::string("TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_") +
		       std::to_string(iteration) + "_";
		switch (mode) {
			case SAVE_SUCCESSIVE_ITERATIONS:
				warp_field_CUDA.SaveToDirectory(std::string("../../Tests/") + path);
				break;
			case TEST_SUCCESSIVE_ITERATIONS:
				warp_field_CUDA2.LoadFromDirectory(path);
				BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA, &warp_field_CUDA2, absolute_tolerance));
				break;
			default:
				break;
		}
	}
	path = "TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_" + std::to_string(iteration_limit - 1) + "_";
	std::string path_warped_live = path + "warped_live_";
	std::string path_fused = path + "fused_";
	switch (mode) {
		case SAVE_FINAL_ITERATION_AND_FUSION:
			warp_field_CUDA.SaveToDirectory(std::string("../../Tests/") + path);
			warped_volumes[target_warped_field_ix]->SaveToDirectory(std::string("../../Tests/") + path_warped_live);
			recoEngine.FuseLiveIntoCanonicalSdf(volume_16, warped_volumes[target_warped_field_ix]);
			volume_16->SaveToDirectory(std::string("../../Tests/") + path_fused);
			break;
		case TEST_FINAL_ITERATION_AND_FUSION:
			warp_field_CUDA2.LoadFromDirectory(path);
			BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA, &warp_field_CUDA2, absolute_tolerance));
			warped_volumes[source_warped_field_ix]->LoadFromDirectory(path_warped_live);
			BOOST_REQUIRE(contentAlmostEqual_CUDA(warped_volumes[target_warped_field_ix],
			                                      warped_volumes[source_warped_field_ix], absolute_tolerance));
			recoEngine.FuseLiveIntoCanonicalSdf(volume_16, warped_volumes[target_warped_field_ix]);
			warped_volumes[source_warped_field_ix]->LoadFromDirectory(path_fused);
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
GenericWarpTest(const SlavchevaSurfaceTracker::Switches& switches, const std::string& prefix, int iteration_limit = 10,
                GenericWarpTestMode mode = TEST_SUCCESSIVE_ITERATIONS) {

	float absoluteTolerance = 1e-7;
	GenericWarpConsistencySubtest<ITMPlainVoxelArray>(switches, prefix, iteration_limit, mode, absoluteTolerance);
	GenericWarpConsistencySubtest<ITMVoxelBlockHash>(switches, prefix, iteration_limit, mode, absoluteTolerance);


	std::string path;
	std::string path_warped_live;
	switch (mode) {
		case TEST_SUCCESSIVE_ITERATIONS: {

			ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CUDA_PVA(&Configuration::get().scene_parameters,
			                                                                Configuration::get().swapping_mode ==
			                                                                Configuration::SWAPPINGMODE_ENABLED,
			                                                                MEMORYDEVICE_CUDA,
			                                                                Fixture::InitParams<ITMPlainVoxelArray>());
			ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CUDA_VBH(&Configuration::get().scene_parameters,
			                                                               Configuration::get().swapping_mode ==
			                                                               Configuration::SWAPPINGMODE_ENABLED,
			                                                               MEMORYDEVICE_CUDA,
			                                                               Fixture::InitParams<ITMVoxelBlockHash>());
			ManipulationEngine_CUDA_VBH_Warp::Inst().ResetScene(&warp_field_CUDA_VBH);
			for (int iteration = 0; iteration < iteration_limit; iteration++) {
				path = "TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_" + std::to_string(iteration) + "_";
				warp_field_CUDA_PVA.LoadFromDirectory(path);
				warp_field_CUDA_VBH.LoadFromDirectory(path);
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
			                                                        Fixture::InitParams<ITMPlainVoxelArray>());
			ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> volume_VBH(&Configuration::get().scene_parameters,
			                                                       Configuration::get().swapping_mode ==
			                                                       Configuration::SWAPPINGMODE_ENABLED,
			                                                       MEMORYDEVICE_CUDA,
			                                                       Fixture::InitParams<ITMVoxelBlockHash>());
			ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetScene(&volume_VBH);
			path = "TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_" + std::to_string(iteration_limit - 1) +
			       "_";
			path_warped_live = path + "warped_live_";
			std::string path_fused = path + "fused_";
			volume_PVA.LoadFromDirectory(path_warped_live);
			volume_VBH.LoadFromDirectory(path_warped_live);
			BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(&volume_PVA, &volume_VBH, absoluteTolerance));
			volume_PVA.LoadFromDirectory(path_fused);
			volume_VBH.LoadFromDirectory(path_fused);
			BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(&volume_PVA, &volume_VBH, absoluteTolerance));
		}
			break;
		default:
			break;
	}
}


BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataTermOnly) {
	Configuration::get().telemetry_settings.focus_coordinates_specified = true;
	Configuration::get().telemetry_settings.focus_coordinates = Vector3i(-7,23, 224);
	GenericWarpTest(SlavchevaSurfaceTracker::Switches(true,
	                                                  false,
	                                                  false,
	                                                  false,
	                                                  false),
	                                                  		"data_only",
	                2,
	                TEST_SUCCESSIVE_ITERATIONS);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonov) {
	GenericWarpTest(SlavchevaSurfaceTracker::Switches(true,
	                                                  false,
	                                                  true,
	                                                  false,
	                                                  false), "data_tikhonov");//, 10, true);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing) {
	GenericWarpTest(SlavchevaSurfaceTracker::Switches(true,
	                                                  false,
	                                                  true,
	                                                  false,
	                                                  true), "data_tikhonov_sobolev");//, 10, true);
}


BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_Fusion) {
	GenericWarpTest(SlavchevaSurfaceTracker::Switches(true,
	                                                  false,
	                                                  true,
	                                                  false,
	                                                  true),
	                "data_tikhonov_sobolev_fusion", 100,
	                GenericWarpTestMode::SAVE_FINAL_ITERATION_AND_FUSION);
}
