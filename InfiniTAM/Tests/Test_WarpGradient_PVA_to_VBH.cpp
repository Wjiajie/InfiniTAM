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
#include "TestUtils.h"
#include "TestUtilsForSnoopyFrames16And17.h"
#include "WarpAdvancedTestingUtilities.h"

#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"

//local CPU
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
#include "../ITMLib/Engines/VolumeEditAndCopy/ITMSceneManipulationEngineFactory.h"
#include "../ITMLib/Engines/Reconstruction/CPU/ITMDynamicSceneReconstructionEngine_CPU.h"

//local CUDA
#include "../ITMLib/Engines/Reconstruction/CUDA/ITMDynamicSceneReconstructionEngine_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"
#include "../ITMLib/Engines/Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"


using namespace ITMLib;

typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray> RecoEngine_CUDA_PVA;
typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash> RecoEngine_CUDA_VBH;


///CAUTION: SAVE modes require the build directory to be immediately inside the root source directory.
template<MemoryDeviceType TMemoryDeviceType>
void
GenericWarpTest(const SlavchevaSurfaceTracker::Switches& switches, int iteration_limit = 10,
                GenericWarpTestMode mode = TEST_SUCCESSIVE_ITERATIONS, float absoluteTolerance = 1e-7) {

	std::string prefix = switches_to_prefix(switches);
	GenericWarpConsistencySubtest<ITMPlainVoxelArray, TMemoryDeviceType>(switches, iteration_limit, mode,
	                                                                     absoluteTolerance);
	GenericWarpConsistencySubtest<ITMVoxelBlockHash, TMemoryDeviceType>(switches, iteration_limit, mode,
	                                                                    absoluteTolerance);

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> volume_PVA(&Configuration::get().scene_parameters,
	                                                        Configuration::get().swapping_mode ==
	                                                        Configuration::SWAPPINGMODE_ENABLED,
	                                                        TMemoryDeviceType,
	                                                        Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> volume_VBH(&Configuration::get().scene_parameters,
	                                                       Configuration::get().swapping_mode ==
	                                                       Configuration::SWAPPINGMODE_ENABLED,
	                                                       TMemoryDeviceType,
	                                                       Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());
	switch (mode) {
		case TEST_SUCCESSIVE_ITERATIONS: {

			ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_PVA(&Configuration::get().scene_parameters,
			                                                           Configuration::get().swapping_mode ==
			                                                           Configuration::SWAPPINGMODE_ENABLED,
			                                                           TMemoryDeviceType,
			                                                           Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
			ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> warp_field_VBH(&Configuration::get().scene_parameters,
		 	                                                          Configuration::get().swapping_mode ==
			                                                          Configuration::SWAPPINGMODE_ENABLED,
			                                                          TMemoryDeviceType,
			                                                          Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());

			for (int iteration = 0; iteration < iteration_limit; iteration++) {
				std::cout << "Testing iteration " << iteration << std::endl;
				warp_field_PVA.LoadFromDirectory(get_path_warps(prefix, iteration));
				ITMSceneManipulationEngineFactory::Instance<ITMWarp, ITMVoxelBlockHash, TMemoryDeviceType>().ResetScene(
						&warp_field_VBH);
				warp_field_VBH.LoadFromDirectory(get_path_warps(prefix, iteration));
				BOOST_REQUIRE(allocatedContentAlmostEqual_Verbose(&warp_field_PVA, &warp_field_VBH,
				                                                  absoluteTolerance, TMemoryDeviceType));
				ITMSceneManipulationEngineFactory::Instance<ITMVoxel, ITMVoxelBlockHash, TMemoryDeviceType>().ResetScene(
						&volume_VBH);
				volume_PVA.LoadFromDirectory(get_path_warped_live(prefix, iteration));
				volume_VBH.LoadFromDirectory(get_path_warped_live(prefix, iteration));
				BOOST_REQUIRE(contentForFlagsAlmostEqual_Verbose(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED,
				                                                 absoluteTolerance, TMemoryDeviceType));
			}
		}
			break;
		case TEST_FINAL_ITERATION_AND_FUSION: {
			volume_PVA.LoadFromDirectory(get_path_warped_live(prefix, iteration_limit - 1));
			ITMSceneManipulationEngineFactory::Instance<ITMVoxel, ITMVoxelBlockHash, TMemoryDeviceType>().ResetScene(
					&volume_VBH);
			volume_VBH.LoadFromDirectory(get_path_warped_live(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentForFlagsAlmostEqual(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED, absoluteTolerance,
					                           TMemoryDeviceType));
			volume_PVA.LoadFromDirectory(get_path_fused(prefix, iteration_limit - 1));
			ITMSceneManipulationEngineFactory::Instance<ITMVoxel, ITMVoxelBlockHash, TMemoryDeviceType>().ResetScene(
					&volume_VBH);
			volume_VBH.LoadFromDirectory(get_path_fused(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentForFlagsAlmostEqual(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED, absoluteTolerance,
					                           TMemoryDeviceType));
		}
			break;
		default:
			break;
	}
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataTermOnly_CPU) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 10, TEST_SUCCESSIVE_ITERATIONS);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataTermOnly_CUDA) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	GenericWarpTest<MEMORYDEVICE_CUDA>(switches, 10, TEST_SUCCESSIVE_ITERATIONS, 1e-5);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonov_CPU) {
	Vector3i test_pos(-57, -9, 196);
	Configuration::get().telemetry_settings.focus_coordinates_specified = true;
	Configuration::get().telemetry_settings.focus_coordinates = test_pos;
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, false);
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 1e-7);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonov_CUDA) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, false);
	GenericWarpTest<MEMORYDEVICE_CUDA>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 1e-7);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_CUDA) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, true);
	GenericWarpTest<MEMORYDEVICE_CUDA>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 1e-7);
}


BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_Fusion_CUDA) {
	GenericWarpTest<MEMORYDEVICE_CUDA>(SlavchevaSurfaceTracker::Switches(true, false, true, false, true), 100,
	                                   GenericWarpTestMode::SAVE_FINAL_ITERATION_AND_FUSION);
}

template<MemoryDeviceType TMemoryDeviceType>
void Warp_PVA_VBH_simple_subtest(int iteration, SlavchevaSurfaceTracker::Switches trackerSwitches, bool expanded_allocation = false) {

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
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warps_PVA;
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warps_VBH;
	if (iteration > 0) {
		std::string path_warps = get_path_warps(prefix, iteration - 1);
		loadVolume(&warps_PVA, path_warps, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
		loadVolume(&warps_VBH, path_warps, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());
		BOOST_REQUIRE(allocatedContentAlmostEqual(warps_PVA, warps_VBH, absoluteTolerance, TMemoryDeviceType));
	} else {
		initializeVolume(&warps_PVA, Frame16And17Fixture::InitParams<ITMPlainVoxelArray>(), TMemoryDeviceType);
		initializeVolume(&warps_VBH, Frame16And17Fixture::InitParams<ITMVoxelBlockHash>(), TMemoryDeviceType);
		BOOST_REQUIRE(allocatedContentAlmostEqual(warps_PVA, warps_VBH, absoluteTolerance, TMemoryDeviceType));
	}

	// *** load warped live scene



	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* warped_live_PVA;
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* warped_live_VBH;

	if (iteration > 0) {
		std::string path_warped_live = get_path_warped_live(prefix, iteration - 1);
		loadVolume(&warped_live_PVA, path_warped_live, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
		loadVolume(&warped_live_VBH, path_warped_live, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());
	} else {
		loadVolume(&warped_live_PVA, path_frame_17_PVA, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
		loadVolume(&warped_live_VBH, path_frame_17_VBH, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());
	}

	// *** load canonical volume as the two different data structures
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume_16_PVA;
	loadVolume(&volume_16_PVA, path_frame_16_PVA, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume_16_VBH;
	loadVolume(&volume_16_VBH, path_frame_16_VBH, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());

	//_DEBUG
	Vector3i test_pos(-5, 8, 195);
	Vector3s voxel_block_pos = TO_SHORT_FLOOR3(test_pos / VOXEL_BLOCK_SIZE);
	Configuration::get().telemetry_settings.focus_coordinates_specified = true;
	Configuration::get().telemetry_settings.focus_coordinates = test_pos;
	ITMVoxel voxelPVA_canonical = volume_16_PVA->GetValueAt(test_pos);
	std::cout << "PVA canonical voxel of interest: ";
	voxelPVA_canonical.print_self();
	ITMVoxel voxelVBH_canonical = volume_16_VBH->GetValueAt(test_pos);
	std::cout << "VBH canonical voxel of interest: ";
	voxelVBH_canonical.print_self();
	int hashCode;
	volume_16_VBH->index.GetHashEntryAt(voxel_block_pos, hashCode);
	std::cout << "VBH canonical hash block: " << voxel_block_pos << " code " << hashCode;
	ITMVoxel voxelPVA = warped_live_PVA->GetValueAt(test_pos);
	std::cout << "PVA live voxel of interest: ";
	voxelPVA.print_self();
	ITMVoxel voxelVBH = warped_live_VBH->GetValueAt(test_pos);
	std::cout << "VBH live voxel of interest: ";
	voxelVBH.print_self();

	ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, TMemoryDeviceType>::Instance()
			.AllocateUsingOtherVolume(volume_16_VBH, warped_live_VBH);
	voxelVBH_canonical = volume_16_VBH->GetValueAt(test_pos);
	std::cout << "VBH canonical voxel of interest (after allocation): ";
	voxelVBH_canonical.print_self();
	volume_16_VBH->index.GetHashEntryAt(voxel_block_pos, hashCode);
	std::cout << "VBH canonical hash block: " << voxel_block_pos << " code " << hashCode;

	// *** perform the warp gradient computation and warp updates
	SurfaceTracker<ITMVoxel, ITMWarp, ITMPlainVoxelArray, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker_PVA(trackerSwitches);

	std::cout << "==== CALCULATE PVA WARPS === " << (expanded_allocation ? "(expanded)" : "") << std::endl;
	motionTracker_PVA.CalculateWarpGradient(volume_16_PVA, warped_live_PVA, warps_PVA);
	motionTracker_PVA.SmoothWarpGradient(volume_16_PVA, warped_live_PVA, warps_PVA);
	motionTracker_PVA.UpdateWarps(volume_16_PVA, warped_live_PVA, warps_PVA);

	SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker_VBH(trackerSwitches);


	std::cout << "==== CALCULATE VBH WARPS === " << (expanded_allocation ? "(expanded)" : "") << std::endl;
	motionTracker_VBH.CalculateWarpGradient(volume_16_VBH, warped_live_VBH, warps_VBH);
	motionTracker_VBH.SmoothWarpGradient(volume_16_VBH, warped_live_VBH, warps_VBH);
	motionTracker_VBH.UpdateWarps(volume_16_VBH, warped_live_VBH, warps_VBH);

	// *** test content

	BOOST_REQUIRE(allocatedContentAlmostEqual_Verbose(warps_PVA, warps_VBH, absoluteTolerance, TMemoryDeviceType));


	//_DEBUG
//	ITMWarp warpPVA = VolumeEditAndCopyEngineInterface<ITMWarp, ITMPlainVoxelArray>::Inst()
//			.ReadVoxel(warps_PVA, test_pos);
//	warpPVA.print_self();
//	ITMWarp warpVBH = VolumeEditAndCopyEngineInterface<ITMWarp, ITMVoxelBlockHash>::Inst()
//			.ReadVoxel(warps_VBH, test_pos);
//	warpVBH.print_self();

	delete volume_16_PVA;
	delete volume_16_VBH;
	delete warped_live_PVA;
	delete warped_live_VBH;

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* loaded_warps_PVA;
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* loaded_warps_VBH;
	std::string path_loaded_warps = get_path_warps(prefix, iteration);
	loadVolume(&loaded_warps_PVA, path_loaded_warps, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<ITMPlainVoxelArray>());
	loadVolume(&loaded_warps_VBH, path_loaded_warps, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<ITMVoxelBlockHash>());

	BOOST_REQUIRE(contentAlmostEqual_Verbose(warps_PVA, loaded_warps_PVA, absoluteTolerance, TMemoryDeviceType));
	BOOST_REQUIRE(contentAlmostEqual_Verbose(warps_VBH, loaded_warps_VBH, absoluteTolerance, TMemoryDeviceType));

	delete warps_PVA;
	delete warps_VBH;
	delete loaded_warps_PVA;
	delete loaded_warps_VBH;
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CPU_data_only_basic) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CPU>(0, switches, false);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CPU_data_only_expanded) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CPU>(0, switches, true);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CUDA_data_only) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CUDA>(0, switches, true);
}