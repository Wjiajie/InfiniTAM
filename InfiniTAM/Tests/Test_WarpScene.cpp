//  ================================================================
//  Created by Gregory Kramida on 10/31/19.
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
#include "../ITMLib/Engines/Reconstruction/CPU/ITMDynamicSceneReconstructionEngine_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"


#ifndef COMPILE_WITHOUT_CUDA

#include "../ITMLib/Engines/Reconstruction/CUDA/ITMDynamicSceneReconstructionEngine_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"

#endif

#include "TestUtils.h"
#include "TestUtilsForSnoopyFrames16And17.h"

using namespace ITMLib;

typedef ITMDynamicSceneReconstructionEngine_CPU<ITMVoxel, ITMWarp, ITMPlainVoxelArray> RecoEngine_CPU_PVA;
typedef ITMDynamicSceneReconstructionEngine_CPU<ITMVoxel, ITMWarp, ITMVoxelBlockHash> RecoEngine_CPU_VBH;

BOOST_AUTO_TEST_CASE(Test_WarpScene_CPU_PVA) {
	const Configuration& settings = Configuration::get();
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warps;
	loadVolume(&warps, "TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_complete_",
	           MEMORYDEVICE_CPU);
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* live_volume;
	loadVolume(&live_volume, "TestData/snoopy_result_fr16-17_partial_PVA/live",
	           MEMORYDEVICE_CPU);
	auto warped_live_volume = new ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>(
			&settings.scene_parameters, false, MEMORYDEVICE_CPU,
			{live_volume->index.GetVolumeSize(), live_volume->index.GetVolumeOffset()});
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetScene(warped_live_volume);

	RecoEngine_CPU_PVA recoEngine;

	recoEngine.WarpScene_FlowWarps(warps, live_volume, warped_live_volume);
	//warped_live_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warped_live_");

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* warped_live_volume_gt;
	loadVolume(&warped_live_volume_gt, "../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warped_live_",
	           MEMORYDEVICE_CPU);

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(!contentAlmostEqual_CPU(warped_live_volume, live_volume, absoluteTolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(warped_live_volume, warped_live_volume_gt, absoluteTolerance));

	delete warps;
	delete live_volume;
	delete warped_live_volume;
}

BOOST_AUTO_TEST_CASE(Test_WarpScene_CPU_VBH) {
	const Configuration& settings = Configuration::get();
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warps;
	loadVolume(&warps, "TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_complete_",
	           MEMORYDEVICE_CPU);
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* live_volume;
	loadVolume(&live_volume, "TestData/snoopy_result_fr16-17_partial_VBH/live",
	           MEMORYDEVICE_CPU);
	auto warped_live_volume = new ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>(
			&settings.scene_parameters, false, MEMORYDEVICE_CPU,
			{live_volume->index.GetAllocatedBlockCount(), live_volume->index.GetExcessListSize()});
	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetScene(warped_live_volume);

	RecoEngine_CPU_VBH recoEngine;

	recoEngine.WarpScene_FlowWarps(warps, live_volume, warped_live_volume);
	//warped_live_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warped_live_");

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* warped_live_volume_gt;
	loadVolume(&warped_live_volume_gt, "TestData/snoopy_result_fr16-17_partial_VBH/warped_live_",
	           MEMORYDEVICE_CPU);


	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(!contentAlmostEqual_CPU(warped_live_volume, live_volume, absoluteTolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(warped_live_volume, warped_live_volume_gt, absoluteTolerance));

	delete warps;
	delete live_volume;
	delete warped_live_volume;
}

BOOST_FIXTURE_TEST_CASE(Test_WarpScene_CPU_VBH_to_PVA, Frame16And17Fixture) {
	const int iteration = 0;
	// *** load warps
	std::string path_warps = "TestData/snoopy_result_fr16-17_warps/data_only_iter_" + std::to_string(iteration) + "_";
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warps_PVA;
	loadVolume(&warps_PVA, path_warps, MEMORYDEVICE_CPU, InitParams<ITMPlainVoxelArray>());
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warps_VBH;
	loadVolume(&warps_VBH, path_warps, MEMORYDEVICE_CPU, InitParams<ITMVoxelBlockHash>());

	std::string path_frame_17_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_17_";
//	std::string path_frame_16_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_16_";
	std::string path_frame_17_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_";
//	std::string path_frame_16_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_16_";

	std::string source_path_PVA;
	std::string source_path_VBH;
	if (iteration == 0) {
		source_path_PVA = path_frame_17_PVA;
		source_path_VBH = path_frame_17_VBH;
	} else {
		source_path_PVA =
				"TestData/snoopy_result_fr16-17_warps/data_only_iter_" + std::to_string(iteration - 1) +
				"_warped_live_";
		source_path_VBH =
				"TestData/snoopy_result_fr16-17_warps/data_only_iter_" + std::to_string(iteration - 1) +
				"_warped_live_";
	}

	// *** load same frame scene as the two different data structures
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* source_volume_PVA;
	loadVolume(&source_volume_PVA, source_path_PVA, MEMORYDEVICE_CPU, InitParams<ITMPlainVoxelArray>());
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* source_volume_VBH;
	loadVolume(&source_volume_VBH, source_path_VBH, MEMORYDEVICE_CPU, InitParams<ITMVoxelBlockHash>());

	// *** initialize target scenes
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* target_PVA;
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* target_VBH;
	initializeVolume(&target_PVA, InitParams<ITMPlainVoxelArray>(), MEMORYDEVICE_CPU);
	initializeVolume(&target_VBH, InitParams<ITMVoxelBlockHash>(), MEMORYDEVICE_CPU);

	// *** perform the warping
	ITMDynamicSceneReconstructionEngine_CPU<ITMVoxel, ITMWarp, ITMPlainVoxelArray> recoEngine_PVA;
	ITMDynamicSceneReconstructionEngine_CPU<ITMVoxel, ITMWarp, ITMVoxelBlockHash> recoEngine_VBH;

//_DEBUG
//	Configuration::get().telemetry_settings.focus_coordinates_specified = true;
//	Configuration::get().telemetry_settings.focus_coordinates = Vector3i(-39, -9, 175);

	recoEngine_PVA.WarpScene_FlowWarps(warps_PVA, source_volume_PVA, target_PVA);
	recoEngine_VBH.WarpScene_FlowWarps(warps_VBH, source_volume_VBH, target_VBH);

	// *** test content
	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(
			contentForFlagsAlmostEqual_CPU_Verbose(target_PVA, target_VBH, VOXEL_NONTRUNCATED, absoluteTolerance));
//	BOOST_REQUIRE(
//			contentForFlagsAlmostEqual_CPU_Verbose(target_PVA, target_VBH, VOXEL_TRUNCATED, absoluteTolerance));

//_DEBUG
	Vector3i test_pos(  -53, -3, 160);
	ITMVoxel voxelPVA = ITMSceneManipulationEngine_CPU<ITMVoxel, ITMPlainVoxelArray>::Inst()
			.ReadVoxel(target_PVA, test_pos);
	voxelPVA.print_self();
	ITMVoxel voxelVBH = ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash>::Inst()
			.ReadVoxel(target_VBH, test_pos);
	voxelVBH.print_self();

	delete warps_PVA;
	delete warps_VBH;
	delete source_volume_PVA;
	delete source_volume_VBH;
	delete target_PVA;
	delete target_VBH;
}

#ifndef COMPILE_WITHOUT_CUDA
typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray> RecoEngine_CUDA_PVA;
typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash> RecoEngine_CUDA_VBH;


BOOST_FIXTURE_TEST_CASE(Test_WarpScene_CUDA_VBH_to_PVA, Frame16And17Fixture) {
	const int iteration = 0;
	// *** load warps
	std::string path_warps = "TestData/snoopy_result_fr16-17_warps/data_only_iter_" + std::to_string(iteration) + "_";
//	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warps_PVA;
//	loadVolume(&warps_PVA, path_warps, MEMORYDEVICE_CUDA, InitParams<ITMPlainVoxelArray>());
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warps_VBH_CPU;
	loadVolume(&warps_VBH_CPU, path_warps, MEMORYDEVICE_CPU, InitParams<ITMVoxelBlockHash>());
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warps_VBH;
	loadVolume(&warps_VBH, path_warps, MEMORYDEVICE_CUDA, InitParams<ITMVoxelBlockHash>());

	std::string path_frame_17_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_17_";
//	std::string path_frame_16_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_16_";
	std::string path_frame_17_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_";
//	std::string path_frame_16_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_16_";

	std::string source_path_PVA;
	std::string source_path_VBH;
	if (iteration == 0) {
//		source_path_PVA = path_frame_17_PVA;
		source_path_VBH = path_frame_17_VBH;
	} else {
		source_path_PVA =
				"TestData/snoopy_result_fr16-17_warps/data_only_iter_" + std::to_string(iteration - 1) +
				"_warped_live_";
		source_path_VBH =
				"TestData/snoopy_result_fr16-17_warps/data_only_iter_" + std::to_string(iteration - 1) +
				"_warped_live_";
	}

	// *** load same frame scene as the two different data structures
//	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* source_volume_PVA;
//	loadVolume(&source_volume_PVA, source_path_PVA, MEMORYDEVICE_CUDA, InitParams<ITMPlainVoxelArray>());
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* source_volume_VBH_CPU;
	loadVolume(&source_volume_VBH_CPU, source_path_VBH, MEMORYDEVICE_CPU, InitParams<ITMVoxelBlockHash>());
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* source_volume_VBH;
	loadVolume(&source_volume_VBH, source_path_VBH, MEMORYDEVICE_CUDA, InitParams<ITMVoxelBlockHash>());

	// *** initialize target scenes
//	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* target_PVA;
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* target_VBH_CPU;
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* target_VBH;
//	initializeVolume(&target_PVA, InitParams<ITMPlainVoxelArray>(), MEMORYDEVICE_CUDA);
	initializeVolume(&target_VBH_CPU, InitParams<ITMVoxelBlockHash>(), MEMORYDEVICE_CPU);
	initializeVolume(&target_VBH, InitParams<ITMVoxelBlockHash>(), MEMORYDEVICE_CUDA);

	// *** perform the warping
	ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray> recoEngine_PVA;
	ITMDynamicSceneReconstructionEngine_CPU<ITMVoxel, ITMWarp, ITMVoxelBlockHash> recoEngine_VBH_CPU;
	ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash> recoEngine_VBH;

//_DEBUG
//	Configuration::get().telemetry_settings.focus_coordinates_specified = true;
//	Configuration::get().telemetry_settings.focus_coordinates = Vector3i(-39, -9, 175);

//	recoEngine_PVA.WarpScene_FlowWarps(warps_PVA, source_volume_PVA, target_PVA);
	recoEngine_VBH_CPU.WarpScene_FlowWarps(warps_VBH_CPU, source_volume_VBH_CPU, target_VBH_CPU);
	recoEngine_VBH.WarpScene_FlowWarps(warps_VBH, source_volume_VBH, target_VBH);
//_DEBUG
	Vector3i test_pos(  -53, -3, 160);

	int hashCode;
	ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().FindHashEntry(target_VBH->index, Vector3s(-7, -1, 20), hashCode);
	std::cout  << "hash code: " << hashCode << std::endl;

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> target_VBH_copy(*target_VBH, MEMORYDEVICE_CPU);

//	ITMVoxel voxelPVA = ITMSceneManipulationEngine_CUDA<ITMVoxel, ITMPlainVoxelArray>::Inst()
//			.ReadVoxel(target_PVA, test_pos);
//	voxelPVA.print_self();
	ITMVoxel voxelVBH = ITMSceneManipulationEngine_CUDA<ITMVoxel, ITMVoxelBlockHash>::Inst()
			.ReadVoxel(target_VBH, test_pos);
	voxelVBH.print_self();
	ITMVoxel voxelVBH_CPU = ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash>::Inst()
			.ReadVoxel(target_VBH_CPU, test_pos);
	voxelVBH_CPU.print_self();

	// *** test content
	float absoluteTolerance = 1e-7;



	BOOST_REQUIRE(
			contentAlmostEqual_CPU_Verbose(&target_VBH_copy, target_VBH_CPU, absoluteTolerance));
//	BOOST_REQUIRE(
//			allocatedContentAlmostEqual_CUDA_Verbose(target_PVA, target_VBH, absoluteTolerance));
//	BOOST_REQUIRE(
//			contentForFlagsAlmostEqual_CUDA_Verbose(target_PVA, target_VBH, VOXEL_TRUNCATED, absoluteTolerance));


//	delete warps_PVA;
	delete warps_VBH_CPU;
	delete warps_VBH;
//	delete source_volume_PVA;
	delete  source_volume_VBH_CPU;
	delete source_volume_VBH;
//	delete target_PVA;
	delete target_VBH_CPU;
	delete target_VBH;
}

BOOST_AUTO_TEST_CASE(Test_WarpScene_CUDA_PVA) {
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warps;
	loadVolume(&warps, "TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_complete_",
	           MEMORYDEVICE_CUDA);
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* live_volume;
	loadVolume(&live_volume, "TestData/snoopy_result_fr16-17_partial_PVA/live",
	           MEMORYDEVICE_CUDA);
	auto warped_live_volume = new ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>(
			&Configuration::get().scene_parameters, false, MEMORYDEVICE_CUDA,
			{live_volume->index.GetVolumeSize(), live_volume->index.GetVolumeOffset()});
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetScene(warped_live_volume);

	RecoEngine_CUDA_PVA recoEngine;

	recoEngine.WarpScene_FlowWarps(warps, live_volume, warped_live_volume);
	//warped_live_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warped_live_");

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* warped_live_volume_gt;
	loadVolume(&warped_live_volume_gt, "TestData/snoopy_result_fr16-17_partial_PVA/warped_live_",
	           MEMORYDEVICE_CUDA);

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(warped_live_volume, warped_live_volume_gt, absoluteTolerance));

	delete warps;
	delete live_volume;
	delete warped_live_volume;
}

BOOST_AUTO_TEST_CASE(Test_WarpScene_CUDA_VBH) {

	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warps;
	loadVolume(&warps, "TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_complete_",
	           MEMORYDEVICE_CUDA);
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* live_volume;
	loadVolume(&live_volume, "TestData/snoopy_result_fr16-17_partial_VBH/live",
	           MEMORYDEVICE_CUDA);
	auto warped_live_volume = new ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>(
			&Configuration::get().scene_parameters, false, MEMORYDEVICE_CUDA,
			{live_volume->index.GetAllocatedBlockCount(), live_volume->index.GetExcessListSize()});
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetScene(warped_live_volume);

	RecoEngine_CUDA_VBH recoEngine;

	recoEngine.WarpScene_FlowWarps(warps, live_volume, warped_live_volume);
	//warped_live_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warped_live_");

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* warped_live_volume_gt;
	loadVolume(&warped_live_volume_gt, "TestData/snoopy_result_fr16-17_partial_VBH/warped_live_",
	           MEMORYDEVICE_CUDA);

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(warped_live_volume, warped_live_volume_gt, absoluteTolerance));

	delete warps;
	delete live_volume;
	delete warped_live_volume;
}


#endif