//  ================================================================
//  Created by Gregory Kramida on 9/5/19.
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
#define BOOST_TEST_MODULE SceneConstruction
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>
//ITMLib
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "TestUtils.h"
#include "../ITMLib/Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"

using namespace ITMLib;

typedef VolumeFileIOEngine<ITMVoxel, PlainVoxelArray> SceneFileIOEngine_PVA;
typedef VolumeFileIOEngine<ITMVoxel, VoxelBlockHash> SceneFileIOEngine_VBH;
//typedef ITMSceneStatisticsCalculator_CUDA<ITMVoxel, PlainVoxelArray> SceneStatisticsCalculator_PVA;
//typedef ITMSceneStatisticsCalculator_CUDA<ITMVoxel, VoxelBlockHash> SceneStatCalc_CPU_VBH_Voxel;

BOOST_AUTO_TEST_CASE(testSaveSceneCompact_CUDA) {


	Vector3i volumeSize(40, 68, 20);
	Vector3i volumeOffset(-20, 0, 0);

	VoxelVolume<ITMVoxel, PlainVoxelArray> scene1(
			&configuration::get().general_voxel_volume_parameters, configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CUDA, {volumeSize, volumeOffset});

	VoxelVolume<ITMVoxel, PlainVoxelArray> scene2(
			&configuration::get().general_voxel_volume_parameters, configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CUDA, {volumeSize, volumeOffset});

	GenerateTestScene_CUDA(&scene1);
	std::string path = "TestData/test_PVA_";
	SceneFileIOEngine_PVA::SaveToDirectoryCompact(&scene1, path);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetVolume(&scene2);
	SceneFileIOEngine_PVA::LoadFromDirectoryCompact(&scene2, path);

	float tolerance = 1e-8;
	BOOST_REQUIRE_EQUAL( SceneStatCalc_CUDA_PVA_Voxel ::Instance().ComputeNonTruncatedVoxelCount(&scene2), 19456);
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene1, &scene2, tolerance));

	VoxelVolume<ITMVoxel, VoxelBlockHash> scene3(
			&configuration::get().general_voxel_volume_parameters, configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CUDA, {0x800, 0x20000});

	VoxelVolume<ITMVoxel, VoxelBlockHash> scene4(
			&configuration::get().general_voxel_volume_parameters, configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CUDA, {0x800, 0x20000});

	GenerateTestScene_CUDA(&scene3);
	path = "TestData/test_VBH_";
	SceneFileIOEngine_VBH::SaveToDirectoryCompact(&scene3, path);
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetVolume(&scene4);
	SceneFileIOEngine_VBH::LoadFromDirectoryCompact(&scene4, path);

	BOOST_REQUIRE_EQUAL( SceneStatCalc_CUDA_VBH_Voxel::Instance().ComputeNonTruncatedVoxelCount(&scene4), 19456);
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene3, &scene4, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene1, &scene4, tolerance));
}