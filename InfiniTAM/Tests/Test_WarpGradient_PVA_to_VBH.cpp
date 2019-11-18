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
#include "../ITMLib/Engines/Reconstruction/CUDA/ITMDynamicSceneReconstructionEngine_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"

#include "TestUtils.h"

using namespace ITMLib;

typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray> RecoEngine_CUDA_PVA;
typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash> RecoEngine_CUDA_VBH;

struct Fixture {
	template<typename TIndex>
	typename TIndex::InitializationParameters InitParams();


};

template<>
ITMPlainVoxelArray::InitializationParameters Fixture::InitParams<ITMPlainVoxelArray>() {
	return {Vector3i(80, 96, 248), Vector3i(-64, -24, 64)};
}

template<>
ITMVoxelBlockHash::InitializationParameters Fixture::InitParams<ITMVoxelBlockHash>() {
	return {800, 0x20000};
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


BOOST_FIXTURE_TEST_CASE(Test_Warp_PVA_VBH_SelfConsistency_DataTermOnly, Fixture) {
	{
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume_PVA_16;
		buildSdfVolumeFromImage(&volume_PVA_16, "TestData/snoopy_depth_000016.png",
		                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
		                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
		                        InitParams<ITMPlainVoxelArray>());

		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume_PVA_17;
		buildSdfVolumeFromImage(&volume_PVA_17, "TestData/snoopy_depth_000016.png",
		                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
		                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
		                        InitParams<ITMPlainVoxelArray>());

		ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CUDA1(&Configuration::get().scene_parameters,
		                                                             Configuration::get().swapping_mode ==
		                                                             Configuration::SWAPPINGMODE_ENABLED,
		                                                             MEMORYDEVICE_CUDA,
		                                                             InitParams<ITMPlainVoxelArray>());
		ManipulationEngine_CUDA_PVA_Warp::Inst().ResetScene(&warp_field_CUDA1);

		SurfaceTracker<ITMVoxel, ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>
		motionTracker_PVA_CUDA(
				SlavchevaSurfaceTracker::Switches(true, false, false, false, false)
		);
		motionTracker_PVA_CUDA.CalculateWarpGradient(volume_PVA_16, volume_PVA_17, &warp_field_CUDA1);
		motionTracker_PVA_CUDA.UpdateWarps(volume_PVA_16, volume_PVA_17, &warp_field_CUDA1);

		ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray> recoEngine;

		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* warped_fields[2] = {
				new ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>(&Configuration::get().scene_parameters,
				                                             Configuration::get().swapping_mode ==
				                                             Configuration::SWAPPINGMODE_ENABLED,
				                                             MEMORYDEVICE_CUDA,
				                                             InitParams<ITMPlainVoxelArray>()),
				new ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>(&Configuration::get().scene_parameters,
				                                             Configuration::get().swapping_mode ==
				                                             Configuration::SWAPPINGMODE_ENABLED,
				                                             MEMORYDEVICE_CUDA,
				                                             InitParams<ITMPlainVoxelArray>())
		};

		recoEngine.WarpScene_FlowWarps(&warp_field_CUDA1, volume_PVA_17, warped_fields[0]);

		const int iteration_limit = 10;
		for(int iteration = 1; iteration < iteration_limit; iteration ++){
			int source_warped_field_ix = iteration % 2;
			int target_warped_field = (iteration + 1) % 2;

			motionTracker_PVA_CUDA.CalculateWarpGradient(volume_PVA_16, warped_fields[source_warped_field_ix], &warp_field_CUDA1);
			motionTracker_PVA_CUDA.UpdateWarps(volume_PVA_16, warped_fields[source_warped_field_ix], &warp_field_CUDA1);
			recoEngine.WarpScene_FlowWarps(&warp_field_CUDA1, warped_fields[source_warped_field_ix], warped_fields[target_warped_field]);
		}


		delete volume_PVA_16;
		delete volume_PVA_17;
		delete warped_fields[0];
		delete warped_fields[1];
	}

	{
		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume_VBH_16;
		buildSdfVolumeFromImage(&volume_VBH_16, "TestData/snoopy_depth_000016.png",
		                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
		                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
		                        InitParams<ITMVoxelBlockHash>());

		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume_VBH_17;
		buildSdfVolumeFromImage(&volume_VBH_17, "TestData/snoopy_depth_000016.png",
		                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
		                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA,
		                        InitParams<ITMVoxelBlockHash>());

		float absoluteTolerance = 1e-7;


		delete volume_VBH_16;
		delete volume_VBH_17;
	}
}