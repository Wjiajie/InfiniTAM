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
#include "../ITMLib/Engines/VolumeEditAndCopy/VolumeEditAndCopyEngineFactory.h"
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

	if (allocateLiveFromBothImages) {
		ITMIndexingEngine<ITMVoxel, TIndex, TMemoryDeviceType>::Instance().AllocateFromDepth(
				live_volumes[1], view);
	}

	updateView(&view, "TestData/snoopy_depth_000017.png",
	           "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
	           "TestData/snoopy_calib.txt", TMemoryDeviceType);
	ITMIndexingEngine<ITMVoxel, TIndex, TMemoryDeviceType>::Instance().AllocateFromDepth(
			live_volumes[1], view);

	live_index_to_start_from = expand_raw_live_allocation ? 0 : 1;
	if (expand_raw_live_allocation) {
		ITMIndexingEngine<ITMVoxel, TIndex, TMemoryDeviceType>::Instance().AllocateUsingOtherVolumeAndSetVisibilityExpanded(
				live_volumes[0], live_volumes[1], view);
	}
	ITMDynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, TIndex>* reconstructionEngine =
			ITMDynamicSceneReconstructionEngineFactory
			::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, TIndex>(TMemoryDeviceType);
	reconstructionEngine->IntegrateDepthImageIntoTsdfVolume(live_volumes[live_index_to_start_from], view);
	ITMSceneStatisticsCalculator<ITMVoxel,TIndex,TMemoryDeviceType>& calculator =
			ITMSceneStatisticsCalculator<ITMVoxel,TIndex,TMemoryDeviceType>::Instance();
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

	ITMVoxelVolume<ITMWarp, TIndex> warp_field(&configuration::get().general_voxel_volume_parameters,
	                                           configuration::get().swapping_mode ==
	                                           configuration::SWAPPINGMODE_ENABLED,
	                                           TMemoryDeviceType,
	                                           Frame16And17Fixture::InitParams<TIndex>());
	VolumeEditAndCopyEngineFactory::Instance<ITMWarp, TIndex, TMemoryDeviceType>().ResetScene(&warp_field);

	ITMVoxelVolume<ITMVoxel, TIndex>* canonical_volume;
	ITMVoxelVolume<ITMVoxel, TIndex>* live_volumes[2] = {
			new ITMVoxelVolume<ITMVoxel, TIndex>(&configuration::get().general_voxel_volume_parameters,
			                                     configuration::get().swapping_mode ==
			                                     configuration::SWAPPINGMODE_ENABLED,
			                                     TMemoryDeviceType,
			                                     Frame16And17Fixture::InitParams<TIndex>()),
			new ITMVoxelVolume<ITMVoxel, TIndex>(&configuration::get().general_voxel_volume_parameters,
			                                     configuration::get().swapping_mode ==
			                                     configuration::SWAPPINGMODE_ENABLED,
			                                     TMemoryDeviceType,
			                                     Frame16And17Fixture::InitParams<TIndex>())
	};
	live_volumes[0]->Reset();
	live_volumes[1]->Reset();

	int live_index_to_start_from;
	GenerateRawLiveAndCanonicalVolumes<TIndex, TMemoryDeviceType>(allocateLiveFromBothImages,
	                                                              expand_raw_live_allocation,
	                                                              &canonical_volume, live_volumes,
	                                                              live_index_to_start_from);

	SurfaceTracker<ITMVoxel, ITMWarp, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker(switches);

	ITMVoxelVolume<ITMWarp, TIndex> ground_truth_warp_field(
			&configuration::get().general_voxel_volume_parameters,
			configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			TMemoryDeviceType, Frame16And17Fixture::InitParams<TIndex>());
	ITMVoxelVolume<ITMVoxel, TIndex> ground_truth_sdf_volume(
			&configuration::get().general_voxel_volume_parameters,
			configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			TMemoryDeviceType, Frame16And17Fixture::InitParams<TIndex>());

	VolumeEditAndCopyEngineFactory::Instance<ITMWarp, TIndex, TMemoryDeviceType>().ResetScene(
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
		recoEngine->WarpScene_FramewiseWarps(&warp_field, live_volumes[source_warped_field_ix],
		                                live_volumes[target_warped_field_ix]);
		std::string path = get_path_warps(prefix, iteration);
		std::string path_warped_live = get_path_warped_live(prefix, iteration);
		switch (mode) {
			case SAVE_SUCCESSIVE_ITERATIONS:
				live_volumes[target_warped_field_ix]->SaveToDirectory(std::string("../../Tests/") + path_warped_live);
				warp_field.SaveToDirectory(std::string("../../Tests/") + path);
				break;
			case TEST_SUCCESSIVE_ITERATIONS:
				VolumeEditAndCopyEngineFactory::Instance<ITMWarp, TIndex, TMemoryDeviceType>().ResetScene(
						&ground_truth_warp_field);
				ground_truth_warp_field.LoadFromDirectory(path);

				BOOST_REQUIRE(contentAlmostEqual_Verbose(&warp_field, &ground_truth_warp_field, absolute_tolerance,
				                                 TMemoryDeviceType));
				VolumeEditAndCopyEngineFactory::Instance<ITMVoxel, TIndex, TMemoryDeviceType>().ResetScene(
						&ground_truth_sdf_volume);
				ground_truth_sdf_volume.LoadFromDirectory(path_warped_live);
				BOOST_REQUIRE(contentAlmostEqual_Verbose(live_volumes[target_warped_field_ix], &ground_truth_sdf_volume,
				                                 absolute_tolerance, TMemoryDeviceType));
				break;
			default:
				break;
		}
	}
	std::cout << getIndexString<TIndex>() << " fusion test" << std::endl;
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
			VolumeEditAndCopyEngineFactory::Instance<ITMWarp, TIndex, TMemoryDeviceType>().ResetScene(
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


template<MemoryDeviceType TMemoryDeviceType>
void Warp_PVA_VBH_simple_subtest(int iteration, SlavchevaSurfaceTracker::Switches trackerSwitches, bool expanded_allocation) {

	if (iteration < 0) {
		DIEWITHEXCEPTION_REPORTLOCATION("Expecting iteration >= 0, got less than that, aborting.");
	}
	std::string path_frame_17_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_17_";
	std::string path_frame_16_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_16_";
	std::string path_frame_17_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_";
	std::string path_frame_16_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_16_";

	if(expanded_allocation){
		path_frame_17_VBH += "expanded_";
	}

	std::string prefix = switches_to_prefix(trackerSwitches);
	float absoluteTolerance = 1e-7;

	// *** initialize/load warps
	ITMVoxelVolume<ITMWarp, PlainVoxelArray>* warps_PVA;
	ITMVoxelVolume<ITMWarp, VoxelBlockHash>* warps_VBH;
	if (iteration > 0) {
		std::string path_warps = get_path_warps(prefix, iteration - 1);
		loadVolume(&warps_PVA, path_warps, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<PlainVoxelArray>());
		loadVolume(&warps_VBH, path_warps, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<VoxelBlockHash>());
		BOOST_REQUIRE(allocatedContentAlmostEqual(warps_PVA, warps_VBH, absoluteTolerance, TMemoryDeviceType));
	} else {
		initializeVolume(&warps_PVA, Frame16And17Fixture::InitParams<PlainVoxelArray>(), TMemoryDeviceType);
		initializeVolume(&warps_VBH, Frame16And17Fixture::InitParams<VoxelBlockHash>(), TMemoryDeviceType);
		BOOST_REQUIRE(allocatedContentAlmostEqual(warps_PVA, warps_VBH, absoluteTolerance, TMemoryDeviceType));
	}

	// *** load warped live scene



	ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* warped_live_PVA;
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* warped_live_VBH;

	if (iteration > 0) {
		std::string path_warped_live = get_path_warped_live(prefix, iteration - 1);
		loadVolume(&warped_live_PVA, path_warped_live, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<PlainVoxelArray>());
		loadVolume(&warped_live_VBH, path_warped_live, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<VoxelBlockHash>());
	} else {
		loadVolume(&warped_live_PVA, path_frame_17_PVA, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<PlainVoxelArray>());
		loadVolume(&warped_live_VBH, path_frame_17_VBH, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<VoxelBlockHash>());
	}

	// *** load canonical volume as the two different data structures
	ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* volume_16_PVA;
	loadVolume(&volume_16_PVA, path_frame_16_PVA, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<PlainVoxelArray>());
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* volume_16_VBH;
	loadVolume(&volume_16_VBH, path_frame_16_VBH, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<VoxelBlockHash>());

	//_DEBUG
//	Vector3i test_pos(8, -4, 202);
//	Vector3s voxel_block_pos = TO_SHORT_FLOOR3(test_pos.toFloat() / VOXEL_BLOCK_SIZE);
//	configuration::get().verbosity_level >= Configuration::VERBOSITY_FOCUS_SPOTS = true;
//	configuration::get().telemetry_settings.focus_coordinates = test_pos;
//
//	int hashCode;
//	ITMHashEntry entry = volume_16_VBH->index.GetHashEntryAt(voxel_block_pos, hashCode);
//	printf("Entry %d %d %d: %d\n", voxel_block_pos.x, voxel_block_pos.y, voxel_block_pos.z, hashCode);
//
//	ITMVoxel voxelPVA_canonical = volume_16_PVA->GetValueAt(test_pos);
//	std::cout << "PVA canonical voxel of interest: ";
//	voxelPVA_canonical.print_self();
//	ITMVoxel voxelVBH_canonical = volume_16_VBH->GetValueAt(test_pos);
//	std::cout << "VBH canonical voxel of interest: ";
//	voxelVBH_canonical.print_self();
//
//	ITMVoxel voxelPVA = warped_live_PVA->GetValueAt(test_pos);
//	std::cout << "PVA live voxel of interest: ";
//	voxelPVA.print_self();
//	ITMVoxel voxelVBH = warped_live_VBH->GetValueAt(test_pos);
//	std::cout << "VBH live voxel of interest: ";
//	voxelVBH.print_self();


//	voxelVBH_canonical = volume_16_VBH->GetValueAt(test_pos);
//	std::cout << "VBH canonical voxel of interest (after allocation): ";
//	voxelVBH_canonical.print_self();
//	entry = volume_16_VBH->index.GetHashEntryAt(voxel_block_pos, hashCode);
//	std::cout << "VBH canonical hash block: " << voxel_block_pos << " code " << hashCode << " ptr: " << entry.ptr << std::endl;

//	alternative_entry = volume_16_VBH->index.GetHashEntry(alternative_index);
//	std::cout << "VBH canonical " << alternative_index << " hash block ptr: " << alternative_entry.ptr << std::endl;
//	ITMWarp warpPVA = warps_PVA->GetValueAt(test_pos);
//	std::cout << "PVA Warp value of interest: ";
//	warpPVA.print_self();
//	ITMWarp warpVBH = warps_VBH->GetValueAt(test_pos);
//	std::cout << "VBH Warp value of interest: ";
//	warpVBH.print_self();

	// *** perform the warp gradient computation and warp updates
	SurfaceTracker<ITMVoxel, ITMWarp, PlainVoxelArray, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker_PVA(trackerSwitches);

	std::cout << "==== CALCULATE PVA WARPS === " << (expanded_allocation ? "(expanded)" : "") << std::endl;
	motionTracker_PVA.CalculateWarpGradient(volume_16_PVA, warped_live_PVA, warps_PVA);
	motionTracker_PVA.SmoothWarpGradient(volume_16_PVA, warped_live_PVA, warps_PVA);
	motionTracker_PVA.UpdateWarps(volume_16_PVA, warped_live_PVA, warps_PVA);

	SurfaceTracker<ITMVoxel, ITMWarp, VoxelBlockHash, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker_VBH(trackerSwitches);


	std::cout << "==== CALCULATE VBH WARPS === " << (expanded_allocation ? "(expanded)" : "") << std::endl;
	motionTracker_VBH.CalculateWarpGradient(volume_16_VBH, warped_live_VBH, warps_VBH);
	motionTracker_VBH.SmoothWarpGradient(volume_16_VBH, warped_live_VBH, warps_VBH);
	motionTracker_VBH.UpdateWarps(volume_16_VBH, warped_live_VBH, warps_VBH);

	// *** test content

//	ITMWarp warpPVA = warps_PVA->GetValueAt(test_pos);
//	std::cout << "PVA Warp value of interest: ";
//	warpPVA.print_self();
//	ITMWarp warpVBH = warps_VBH->GetValueAt(test_pos);
//	std::cout << "VBH Warp value of interest: ";
//	warpVBH.print_self();

	BOOST_REQUIRE(allocatedContentAlmostEqual_Verbose(warps_PVA, warps_VBH, absoluteTolerance, TMemoryDeviceType));


	//_DEBUG
//	ITMWarp warpPVA = VolumeEditAndCopyEngineInterface<ITMWarp, PlainVoxelArray>::Inst()
//			.ReadVoxel(warps_PVA, test_pos);
//	warpPVA.print_self();
//	ITMWarp warpVBH = VolumeEditAndCopyEngineInterface<ITMWarp, VoxelBlockHash>::Inst()
//			.ReadVoxel(warps_VBH, test_pos);
//	warpVBH.print_self();

	delete volume_16_PVA;
	delete volume_16_VBH;
	delete warped_live_PVA;
	delete warped_live_VBH;

	ITMVoxelVolume<ITMWarp, PlainVoxelArray>* loaded_warps_PVA;
	ITMVoxelVolume<ITMWarp, VoxelBlockHash>* loaded_warps_VBH;
	std::string path_loaded_warps = get_path_warps(prefix, iteration);
	loadVolume(&loaded_warps_PVA, path_loaded_warps, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<PlainVoxelArray>());
	loadVolume(&loaded_warps_VBH, path_loaded_warps, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<VoxelBlockHash>());

	BOOST_REQUIRE(contentAlmostEqual_Verbose(warps_PVA, loaded_warps_PVA, absoluteTolerance, TMemoryDeviceType));
	BOOST_REQUIRE(contentAlmostEqual_Verbose(warps_VBH, loaded_warps_VBH, absoluteTolerance, TMemoryDeviceType));

	delete warps_PVA;
	delete warps_VBH;
	delete loaded_warps_PVA;
	delete loaded_warps_VBH;
}