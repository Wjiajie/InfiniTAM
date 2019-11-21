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

BOOST_FIXTURE_TEST_CASE(Test_Warp_PVA_VBH_CPU_DataTermOnly, Fixture) {
	float absoluteTolerance = 1e-7;
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CPU_VBH(&Configuration::get().scene_parameters,
	                                                              Configuration::get().swapping_mode ==
	                                                              Configuration::SWAPPINGMODE_ENABLED,
	                                                              MEMORYDEVICE_CPU,
	                                                              InitParams<ITMVoxelBlockHash>());
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume_VBH_16;
	buildSdfVolumeFromImage(&volume_VBH_16, "TestData/snoopy_depth_000016.png",
	                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
	                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU,
	                        InitParams<ITMVoxelBlockHash>());

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume_VBH_17;
	buildSdfVolumeFromImage(&volume_VBH_17, "TestData/snoopy_depth_000016.png",
	                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
	                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU,
	                        InitParams<ITMVoxelBlockHash>());


	ManipulationEngine_CPU_VBH_Warp::Inst().ResetScene(&warp_field_CPU_VBH);

	SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker_VBH_CPU(
			SlavchevaSurfaceTracker::Switches(true, false, false, false, false)
	);
	motionTracker_VBH_CPU.CalculateWarpGradient(volume_VBH_16, volume_VBH_17, &warp_field_CPU_VBH);
	motionTracker_VBH_CPU.UpdateWarps(volume_VBH_16, volume_VBH_17, &warp_field_CPU_VBH);

	//warp_field_CPU_VBH.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_warps/data_only_iter_0_");
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* data_only_iter_0_gt;
	loadSdfVolume(&data_only_iter_0_gt, "TestData/snoopy_result_fr16-17_warps/data_only_iter_0_",
	              MEMORYDEVICE_CPU, InitParams<ITMVoxelBlockHash>(), Configuration::get().swapping_mode);
	BOOST_REQUIRE(contentAlmostEqual_CPU(data_only_iter_0_gt, &warp_field_CPU_VBH, absoluteTolerance));
	delete data_only_iter_0_gt;


	ITMDynamicSceneReconstructionEngine_CPU<ITMVoxel, ITMWarp, ITMVoxelBlockHash> recoEngine;

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* warped_fields[2] = {
			new ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>(&Configuration::get().scene_parameters,
			                                                Configuration::get().swapping_mode ==
			                                                Configuration::SWAPPINGMODE_ENABLED,
			                                                MEMORYDEVICE_CPU,
			                                                InitParams<ITMVoxelBlockHash>()),

			new ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>(&Configuration::get().scene_parameters,
			                                                Configuration::get().swapping_mode ==
			                                                Configuration::SWAPPINGMODE_ENABLED,
			                                                MEMORYDEVICE_CPU,
			                                                InitParams<ITMVoxelBlockHash>())
	};
	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetScene(warped_fields[0]);
	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetScene(warped_fields[1]);

	recoEngine.WarpScene_FlowWarps(&warp_field_CPU_VBH, volume_VBH_17, warped_fields[0]);

	const int iteration_limit = 10;
	for (int iteration = 1; iteration < iteration_limit; iteration++) {
		int source_warped_field_ix = iteration % 2;
		int target_warped_field = (iteration + 1) % 2;

		motionTracker_VBH_CPU.CalculateWarpGradient(volume_VBH_16, warped_fields[source_warped_field_ix],
		                                            &warp_field_CPU_VBH);
		motionTracker_VBH_CPU.UpdateWarps(volume_VBH_16, warped_fields[source_warped_field_ix], &warp_field_CPU_VBH);
		recoEngine.WarpScene_FlowWarps(&warp_field_CPU_VBH, warped_fields[source_warped_field_ix],
		                               warped_fields[target_warped_field]);
//		std::string path = std::string("../../Tests/TestData/snoopy_result_fr16-17_warps/data_only_iter_") +
//		                   std::to_string(iteration) + "_";
//		warp_field_CPU_VBH.SaveToDirectory(path);
	}


	delete volume_VBH_16;
	delete volume_VBH_17;

	delete warped_fields[0];
	delete warped_fields[1];
}

void
GenericWarpTest(const SlavchevaSurfaceTracker::Switches& switches, const std::string& prefix, int iteration_limit = 10,
                bool save = false) {
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
	float absoluteTolerance = 1e-7;
	{
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume_PVA_16;
		buildSdfVolumeFromImage(&volume_PVA_16, "TestData/snoopy_depth_000016.png",
		                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
		                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
		                        Fixture::InitParams<ITMPlainVoxelArray>());

		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume_PVA_17;
		buildSdfVolumeFromImage(&volume_PVA_17, "TestData/snoopy_depth_000016.png",
		                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
		                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
		                        Fixture::InitParams<ITMPlainVoxelArray>());


		ManipulationEngine_CUDA_PVA_Warp::Inst().ResetScene(&warp_field_CUDA_PVA);

		SurfaceTracker<ITMVoxel, ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>
				motionTracker_PVA_CUDA(
				SlavchevaSurfaceTracker::Switches(true, false, false, false, false)
		);
		motionTracker_PVA_CUDA.CalculateWarpGradient(volume_PVA_16, volume_PVA_17, &warp_field_CUDA_PVA);
		motionTracker_PVA_CUDA.SmoothWarpGradient(volume_PVA_16, volume_PVA_17, &warp_field_CUDA_PVA);
		motionTracker_PVA_CUDA.UpdateWarps(volume_PVA_16, volume_PVA_17, &warp_field_CUDA_PVA);

		ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CUDA_PVA2(&Configuration::get().scene_parameters,
		                                                                 Configuration::get().swapping_mode ==
		                                                                 Configuration::SWAPPINGMODE_ENABLED,
		                                                                 MEMORYDEVICE_CUDA,
		                                                                 Fixture::InitParams<ITMPlainVoxelArray>());
		std::string path = "TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_0_";
		if (save) {
			warp_field_CUDA_PVA.SaveToDirectory(std::string("../../Tests/") + path);
		} else {
			warp_field_CUDA_PVA2.LoadFromDirectory(path);
		}
		BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA_PVA2, &warp_field_CUDA_PVA, absoluteTolerance));


		ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray> recoEngine;

		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* warped_volumes[2] = {
				new ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>(&Configuration::get().scene_parameters,
				                                                 Configuration::get().swapping_mode ==
				                                                 Configuration::SWAPPINGMODE_ENABLED,
				                                                 MEMORYDEVICE_CUDA,
				                                                 Fixture::InitParams<ITMPlainVoxelArray>()),
				new ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>(&Configuration::get().scene_parameters,
				                                                 Configuration::get().swapping_mode ==
				                                                 Configuration::SWAPPINGMODE_ENABLED,
				                                                 MEMORYDEVICE_CUDA,
				                                                 Fixture::InitParams<ITMPlainVoxelArray>())
		};
		ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetScene(warped_volumes[0]);
		ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetScene(warped_volumes[1]);

		recoEngine.WarpScene_FlowWarps(&warp_field_CUDA_PVA, volume_PVA_17, warped_volumes[0]);

		for (int iteration = 1; iteration < iteration_limit; iteration++) {
			int source_warped_field_ix = iteration % 2;
			int target_warped_field = (iteration + 1) % 2;

			motionTracker_PVA_CUDA.CalculateWarpGradient(volume_PVA_16, warped_volumes[source_warped_field_ix],
			                                             &warp_field_CUDA_PVA);
			motionTracker_PVA_CUDA.SmoothWarpGradient(volume_PVA_16, warped_volumes[source_warped_field_ix],
			                                          &warp_field_CUDA_PVA);
			motionTracker_PVA_CUDA.UpdateWarps(volume_PVA_16, warped_volumes[source_warped_field_ix],
			                                   &warp_field_CUDA_PVA);
			recoEngine.WarpScene_FlowWarps(&warp_field_CUDA_PVA, warped_volumes[source_warped_field_ix],
			                               warped_volumes[target_warped_field]);
			std::string path = std::string("TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_") +
			                   std::to_string(iteration) + "_";
			if (save) {
				warp_field_CUDA_PVA.SaveToDirectory(std::string("../../Tests/") + path);
			} else {
				warp_field_CUDA_PVA2.LoadFromDirectory(path);
				BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA_PVA, &warp_field_CUDA_PVA2, absoluteTolerance));
			}
		}


		delete volume_PVA_16;
		delete volume_PVA_17;
		delete warped_volumes[0];
		delete warped_volumes[1];
	}

	{
		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume_VBH_16;
		buildSdfVolumeFromImage(&volume_VBH_16, "TestData/snoopy_depth_000016.png",
		                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
		                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
		                        Fixture::InitParams<ITMVoxelBlockHash>());

		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume_VBH_17;
		buildSdfVolumeFromImage(&volume_VBH_17, "TestData/snoopy_depth_000016.png",
		                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
		                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
		                        Fixture::InitParams<ITMVoxelBlockHash>());


		ManipulationEngine_CUDA_VBH_Warp::Inst().ResetScene(&warp_field_CUDA_VBH);

		SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>
				motionTracker_VBH_CUDA(
				SlavchevaSurfaceTracker::Switches(true, false, false, false, false)
		);
		motionTracker_VBH_CUDA.CalculateWarpGradient(volume_VBH_16, volume_VBH_17, &warp_field_CUDA_VBH);
		motionTracker_VBH_CUDA.SmoothWarpGradient(volume_VBH_16, volume_VBH_17, &warp_field_CUDA_VBH);
		motionTracker_VBH_CUDA.UpdateWarps(volume_VBH_16, volume_VBH_17, &warp_field_CUDA_VBH);

		ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_CUDA_VBH2(&Configuration::get().scene_parameters,
		                                                                Configuration::get().swapping_mode ==
		                                                                Configuration::SWAPPINGMODE_ENABLED,
		                                                                MEMORYDEVICE_CUDA,
		                                                                Fixture::InitParams<ITMVoxelBlockHash>());
		std::string path = "TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_0_";
		ManipulationEngine_CUDA_VBH_Warp::Inst().ResetScene(&warp_field_CUDA_VBH2);
		if (save) {
			warp_field_CUDA_VBH.SaveToDirectory(std::string("../../Tests/") + path);
		} else {
			warp_field_CUDA_VBH2.LoadFromDirectory(path);
		}
		BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA_VBH2, &warp_field_CUDA_VBH, absoluteTolerance));


		ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash> recoEngine;

		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* warped_volumes[2] = {
				new ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>(&Configuration::get().scene_parameters,
				                                                Configuration::get().swapping_mode ==
				                                                Configuration::SWAPPINGMODE_ENABLED,
				                                                MEMORYDEVICE_CUDA,
				                                                Fixture::InitParams<ITMVoxelBlockHash>()),
				new ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>(&Configuration::get().scene_parameters,
				                                                Configuration::get().swapping_mode ==
				                                                Configuration::SWAPPINGMODE_ENABLED,
				                                                MEMORYDEVICE_CUDA,
				                                                Fixture::InitParams<ITMVoxelBlockHash>())
		};
		ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetScene(warped_volumes[0]);
		ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetScene(warped_volumes[1]);

		recoEngine.WarpScene_FlowWarps(&warp_field_CUDA_VBH, volume_VBH_17, warped_volumes[0]);


		for (int iteration = 1; iteration < iteration_limit; iteration++) {
			int source_warped_field_ix = iteration % 2;
			int target_warped_field = (iteration + 1) % 2;

			motionTracker_VBH_CUDA.CalculateWarpGradient(volume_VBH_16, warped_volumes[source_warped_field_ix],
			                                             &warp_field_CUDA_VBH);
			motionTracker_VBH_CUDA.SmoothWarpGradient(volume_VBH_16, warped_volumes[source_warped_field_ix],
			                                          &warp_field_CUDA_VBH);
			motionTracker_VBH_CUDA.UpdateWarps(volume_VBH_16, warped_volumes[source_warped_field_ix],
			                                   &warp_field_CUDA_VBH);
			recoEngine.WarpScene_FlowWarps(&warp_field_CUDA_VBH, warped_volumes[source_warped_field_ix],
			                               warped_volumes[target_warped_field]);

			std::string path = std::string("TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_") +
			                   std::to_string(iteration) + "_";
			if (save) {
				warp_field_CUDA_VBH.SaveToDirectory(std::string("../../Tests/") + path);
			} else {
				warp_field_CUDA_VBH2.LoadFromDirectory(path);
				BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA_VBH, &warp_field_CUDA_VBH2, absoluteTolerance));
			}
		}


		delete volume_VBH_16;
		delete volume_VBH_17;

		delete warped_volumes[0];
		delete warped_volumes[1];
	}

	if (!save) {
		for (int iteration = 0; iteration < iteration_limit; iteration++) {
			warp_field_CUDA_PVA.LoadFromDirectory(
					std::string("TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_") +
					std::to_string(iteration) +
					"_");
			warp_field_CUDA_VBH.LoadFromDirectory(
					std::string("TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_") +
					std::to_string(iteration) +
					"_");
			BOOST_REQUIRE(
					allocatedContentAlmostEqual_CUDA(&warp_field_CUDA_PVA, &warp_field_CUDA_VBH, absoluteTolerance));
		}
	}
}


BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataTermOnly) {
	GenericWarpTest(SlavchevaSurfaceTracker::Switches(true,
	                                                  false,
	                                                  false,
	                                                  false,
	                                                  false), "data_only");
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