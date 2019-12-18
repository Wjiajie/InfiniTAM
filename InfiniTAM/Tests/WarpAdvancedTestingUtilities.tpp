//  ================================================================
//  Created by Gregory Kramida on 12/17/19.
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
#pragma once

//boost
#include <boost/test/test_tools.hpp>

//local
#include "WarpAdvancedTestingUtilities.h"
#include "TestUtils.h"
#include "TestUtilsForSnoopyFrames16And17.h"

#include "../ITMLib/Engines/Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"
#include "../ITMLib/Engines/Manipulation/ITMSceneManipulationEngineFactory.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison.h"

#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"
#endif


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void
GenerateRawLiveAndCanonicalVolumes(bool allocateLiveFromBothImages,
                                   bool expand_raw_live_allocation,
                                   ITMVoxelVolume<ITMVoxel, TIndex>** canonical_volume,
                                   ITMVoxelVolume<ITMVoxel, TIndex>** live_volumes,
                                   int& live_index_to_start_from) {
	ITMView* view = nullptr;
	buildSdfVolumeFromImage(canonical_volume, &view,
	                        "TestData/snoopy_depth_000016.png",
	                        "TestData/snoopy_color_000016.png",
	                        "TestData/snoopy_omask_000016.png",
	                        "TestData/snoopy_calib.txt",
	                        TMemoryDeviceType,
	                        Frame16And17Fixture::InitParams<TIndex>());

	Vector2i imageSize(640, 480);

	ITMTrackingState trackingState(imageSize, TMemoryDeviceType);
	if (allocateLiveFromBothImages) {
		ITMIndexingEngine<ITMVoxel, TIndex, TMemoryDeviceType>::Instance().AllocateFromDepth(
				live_volumes[1], view, &trackingState, false, false);
	}

	updateView("TestData/snoopy_depth_000017.png",
	           "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
	           "TestData/snoopy_calib.txt", TMemoryDeviceType, &view);
	ITMIndexingEngine<ITMVoxel, TIndex, TMemoryDeviceType>::Instance().AllocateFromDepth(
			live_volumes[1], view, &trackingState, false, false);

	live_index_to_start_from = expand_raw_live_allocation ? 0 : 1;
	if (expand_raw_live_allocation) {
		ITMIndexingEngine<ITMVoxel, TIndex, TMemoryDeviceType>::Instance().AllocateUsingOtherVolumeExpanded(
				live_volumes[0], live_volumes[1]);
	}
	ITMDynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, TIndex>* reconstructionEngine =
			ITMDynamicSceneReconstructionEngineFactory
			::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, TIndex>(TMemoryDeviceType);
	reconstructionEngine->IntegrateDepthImageIntoTsdfVolume(live_volumes[live_index_to_start_from], view, &trackingState);
	ITMSceneStatisticsCalculator<ITMVoxel,TIndex,TMemoryDeviceType>& calculator =
			ITMSceneStatisticsCalculator<ITMVoxel,TIndex,TMemoryDeviceType>::Instance();
	BOOST_REQUIRE_EQUAL(calculator.ComputeAlteredVoxelCount(live_volumes[live_index_to_start_from]), 116110);
}

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void
GenericWarpConsistencySubtest(const SlavchevaSurfaceTracker::Switches& switches,
                              int iteration_limit,
                              GenericWarpTestMode mode,
                              float absolute_tolerance,
                              bool allocateLiveFromBothImages,
                              bool expand_raw_live_allocation) {

	std::string prefix = switches_to_prefix(switches);
	if (iteration_limit < 2) {
		DIEWITHEXCEPTION_REPORTLOCATION("Iteration limit must be at least 2");
	}

	ITMVoxelVolume<ITMWarp, TIndex> warp_field(&Configuration::get().scene_parameters,
	                                           Configuration::get().swapping_mode ==
	                                           Configuration::SWAPPINGMODE_ENABLED,
	                                           TMemoryDeviceType,
	                                           Frame16And17Fixture::InitParams<TIndex>());
	ITMSceneManipulationEngineFactory::Instance<ITMWarp, TIndex, TMemoryDeviceType>().ResetScene(&warp_field);

	ITMVoxelVolume<ITMVoxel, TIndex>* canonical_volume;
	ITMVoxelVolume<ITMVoxel, TIndex>* live_volumes[2] = {
			new ITMVoxelVolume<ITMVoxel, TIndex>(&Configuration::get().scene_parameters,
			                                     Configuration::get().swapping_mode ==
			                                     Configuration::SWAPPINGMODE_ENABLED,
			                                     TMemoryDeviceType,
			                                     Frame16And17Fixture::InitParams<TIndex>()),
			new ITMVoxelVolume<ITMVoxel, TIndex>(&Configuration::get().scene_parameters,
			                                     Configuration::get().swapping_mode ==
			                                     Configuration::SWAPPINGMODE_ENABLED,
			                                     TMemoryDeviceType,
			                                     Frame16And17Fixture::InitParams<TIndex>())
	};
	ITMSceneManipulationEngineFactory::Instance<ITMVoxel, TIndex, TMemoryDeviceType>().ResetScene(live_volumes[0]);
	ITMSceneManipulationEngineFactory::Instance<ITMVoxel, TIndex, TMemoryDeviceType>().ResetScene(live_volumes[1]);

	int live_index_to_start_from;
	GenerateRawLiveAndCanonicalVolumes<TIndex, TMemoryDeviceType>(allocateLiveFromBothImages,
	                                                              expand_raw_live_allocation,
	                                                              &canonical_volume, live_volumes,
	                                                              live_index_to_start_from);

	SurfaceTracker<ITMVoxel, ITMWarp, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker(switches);

	ITMVoxelVolume<ITMWarp, TIndex> ground_truth_warp_field(
			&Configuration::get().scene_parameters,
			Configuration::get().swapping_mode == Configuration::SWAPPINGMODE_ENABLED,
			TMemoryDeviceType, Frame16And17Fixture::InitParams<TIndex>());
	ITMVoxelVolume<ITMVoxel, TIndex> ground_truth_sdf_volume(
			&Configuration::get().scene_parameters,
			Configuration::get().swapping_mode == Configuration::SWAPPINGMODE_ENABLED,
			TMemoryDeviceType, Frame16And17Fixture::InitParams<TIndex>());

	ITMSceneManipulationEngineFactory::Instance<ITMWarp, TIndex, TMemoryDeviceType>().ResetScene(
			&ground_truth_warp_field);

	ITMDynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, TIndex>* recoEngine =
			ITMDynamicSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, TIndex>(
					TMemoryDeviceType);

	//note: will be swapped before first iteration
	int source_warped_field_ix = (live_index_to_start_from + 1) % 2;
	int target_warped_field_ix = live_index_to_start_from;
	for (int iteration = 0; iteration < iteration_limit; iteration++) {
		std::swap(source_warped_field_ix, target_warped_field_ix);
		std::cout << "Subtest " << getIndexString<TIndex>() << " iteration " << std::to_string(iteration) << std::endl;
		motionTracker.CalculateWarpGradient(canonical_volume, live_volumes[source_warped_field_ix], &warp_field);
		motionTracker.SmoothWarpGradient(canonical_volume, live_volumes[source_warped_field_ix], &warp_field);
		motionTracker.UpdateWarps(canonical_volume, live_volumes[source_warped_field_ix], &warp_field);
		recoEngine->WarpScene_FlowWarps(&warp_field, live_volumes[source_warped_field_ix],
		                                live_volumes[target_warped_field_ix]);
		std::string path = get_path_warps(prefix, iteration);
		std::string path_warped_live = get_path_warped_live(prefix, iteration);
		switch (mode) {
			case SAVE_SUCCESSIVE_ITERATIONS:
				live_volumes[target_warped_field_ix]->SaveToDirectory(std::string("../../Tests/") + path_warped_live);
				warp_field.SaveToDirectory(std::string("../../Tests/") + path);
				break;
			case TEST_SUCCESSIVE_ITERATIONS:
				ITMSceneManipulationEngineFactory::Instance<ITMWarp, TIndex, TMemoryDeviceType>().ResetScene(
						&ground_truth_warp_field);
				ground_truth_warp_field.LoadFromDirectory(path);
				BOOST_REQUIRE(contentAlmostEqual(&warp_field, &ground_truth_warp_field, absolute_tolerance,
				                                 TMemoryDeviceType));
				ITMSceneManipulationEngineFactory::Instance<ITMVoxel, TIndex, TMemoryDeviceType>().ResetScene(
						&ground_truth_sdf_volume);
				ground_truth_sdf_volume.LoadFromDirectory(path_warped_live);
				BOOST_REQUIRE(contentAlmostEqual(live_volumes[target_warped_field_ix], &ground_truth_sdf_volume,
				                                 absolute_tolerance, TMemoryDeviceType));
				break;
			default:
				break;
		}
	}
	switch (mode) {
		case SAVE_FINAL_ITERATION_AND_FUSION:
			warp_field.SaveToDirectory(std::string("../../Tests/") + get_path_warps(prefix, iteration_limit - 1));
			live_volumes[target_warped_field_ix]->SaveToDirectory(
					std::string("../../Tests/") + get_path_warped_live(prefix, iteration_limit - 1));
			recoEngine->FuseOneTsdfVolumeIntoAnother(canonical_volume, live_volumes[target_warped_field_ix]);
			canonical_volume->SaveToDirectory(
					std::string("../../Tests/") + get_path_fused(prefix, iteration_limit - 1));
			break;
		case TEST_FINAL_ITERATION_AND_FUSION:
			ITMSceneManipulationEngineFactory::Instance<ITMWarp, TIndex, TMemoryDeviceType>().ResetScene(
					&ground_truth_warp_field);
			ground_truth_warp_field.LoadFromDirectory(get_path_warps(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentAlmostEqual(&warp_field, &ground_truth_warp_field, absolute_tolerance, TMemoryDeviceType));
			ground_truth_sdf_volume.LoadFromDirectory(
					get_path_warped_live(prefix, iteration_limit - 1));
			BOOST_REQUIRE(contentAlmostEqual(live_volumes[target_warped_field_ix], &ground_truth_sdf_volume,
			                                 absolute_tolerance, TMemoryDeviceType));
			recoEngine->FuseOneTsdfVolumeIntoAnother(canonical_volume, live_volumes[target_warped_field_ix]);
			ground_truth_sdf_volume.LoadFromDirectory(get_path_fused(prefix, iteration_limit - 1));
			BOOST_REQUIRE(contentAlmostEqual(canonical_volume, &ground_truth_sdf_volume, absolute_tolerance,
			                                 TMemoryDeviceType));
			break;
		default:
			break;
	}

	delete canonical_volume;
	delete live_volumes[0];
	delete live_volumes[1];
	delete recoEngine;
}