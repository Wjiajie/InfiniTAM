//  ================================================================
//  Created by Gregory Kramida on 12/20/19.
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
#define BOOST_TEST_MODULE depth_to_tsdf_CPU_vs_CUDA
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

// *** ITMLib ***
#include "TestUtilsForSnoopyFrames16And17.h"
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Engines/Reconstruction/Interface/ITMDynamicSceneReconstructionEngine.h"
#include "../ITMLib/Engines/Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"
#include "../ITMLib/Engines/Manipulation/ITMSceneManipulationEngineFactory.h"
#include "../ITMLib/Utils/Analytics/ITMAlmostEqual.h"
#include "TestUtils.h"
//(cpu)
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"
//(CUDA)

using namespace ITMLib;

BOOST_FIXTURE_TEST_CASE(Test_SceneConstruct17_VBH_Expnaded_CPU_CUDA, Frame16And17Fixture) {

	ITMView* view_CPU = nullptr;
	updateView(&view_CPU, "TestData/snoopy_depth_000017.png",
	           "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
	           "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU);

// *** construct volumes ***
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> volume_PVA_17(MEMORYDEVICE_CPU, InitParams<ITMPlainVoxelArray>());
	ITMSceneManipulationEngineFactory::Instance<ITMVoxel, ITMPlainVoxelArray, MEMORYDEVICE_CPU>().ResetScene(
			&volume_PVA_17);
	ITMDynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMPlainVoxelArray>* reconstructionEngine_PVA =
			ITMDynamicSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMPlainVoxelArray>(
					MEMORYDEVICE_CPU);
	reconstructionEngine_PVA->GenerateTsdfVolumeFromView(&volume_PVA_17, view_CPU);


	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> volume_VBH_17(MEMORYDEVICE_CPU, InitParams<ITMVoxelBlockHash>());
	ITMSceneManipulationEngineFactory::Instance<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>().ResetScene(
			&volume_VBH_17);
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> volume_VBH_17_depth_allocation(MEMORYDEVICE_CPU,
	                                                                           InitParams<ITMVoxelBlockHash>());
	ITMSceneManipulationEngineFactory::Instance<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>().ResetScene(
			&volume_VBH_17_depth_allocation);

	ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>& indexer =
			ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::Instance();
	indexer.AllocateFromDepth(&volume_VBH_17_depth_allocation, view_CPU);
	indexer.AllocateUsingOtherVolumeAndSetVisibilityExpanded(&volume_VBH_17, &volume_VBH_17_depth_allocation, view_CPU);

	ITMDynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMVoxelBlockHash>* reconstructionEngine_VBH =
			ITMDynamicSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMVoxelBlockHash>(
					MEMORYDEVICE_CPU);
	reconstructionEngine_VBH->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17, view_CPU);
	reconstructionEngine_VBH->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17_depth_allocation, view_CPU);


	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU_Verbose(&volume_PVA_17, &volume_VBH_17_depth_allocation,
	                                                      absoluteTolerance));
	BOOST_REQUIRE(contentForFlagsAlmostEqual_CPU_Verbose(&volume_PVA_17, &volume_VBH_17_depth_allocation,
	                                                     VoxelFlags::VOXEL_NONTRUNCATED,
	                                                     absoluteTolerance));

	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU_Verbose(&volume_PVA_17, &volume_VBH_17, absoluteTolerance));
	BOOST_REQUIRE(contentForFlagsAlmostEqual_CPU_Verbose(&volume_PVA_17, &volume_VBH_17, VoxelFlags::VOXEL_NONTRUNCATED,
	                                                     absoluteTolerance));

	delete reconstructionEngine_PVA;
	delete reconstructionEngine_VBH;
	delete view_CPU;
}