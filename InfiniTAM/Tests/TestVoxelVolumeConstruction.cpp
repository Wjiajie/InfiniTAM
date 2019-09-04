//  ================================================================
//  Created by Gregory Kramida on 10/23/17.
//  Copyright (c) 2017-2025 Gregory Kramida
//  Licensed under the Apache Li
// cense, Version 2.0 (the "License");
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

//stdlib
#include <vector>
#include <random>

//boost
#include <boost/test/unit_test.hpp>

//ITMlib
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"
#include "../ITMLib/Objects/Scene/ITMRepresentationAccess.h"
#include "../ITMLib/Objects/Camera/ITMCalibIO.h"

#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Utils/Analytics/ITMSceneStatisticsCalculator.h"
#include "../ITMLib/Utils/Analytics/ITMVoxelVolumeComparison_CPU.h"

#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Engines/Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"
#include "../ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../ITMLib/Engines/SceneFileIO/ITMSceneFileIOEngine.h"
#include "../ITMLib/Engines/Reconstruction/Interface/ITMDynamicSceneReconstructionEngine.h"
#include "../ITMLib/Engines/Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"

#include "../InputSource/ImageSourceEngine.h"
#include "../ORUtils/FileUtils.h"

//local
#include "TestUtils.h"


using namespace ITMLib;


BOOST_AUTO_TEST_CASE(testSetVoxelAndCopy_PlainVoxelArray_CPU) {
	ITMLibSettings* settings = &ITMLibSettings::Instance();
	settings->deviceType = ITMLibSettings::DEVICE_CPU;

	Vector3i volumeSize(20);
	Vector3i volumeOffset(-10, -10, 0);

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> scene1(&settings->sceneParams,
	                                                    settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                    settings->GetMemoryType(),
	                                                    volumeSize,
	                                                    volumeOffset);

	typedef ITMSceneManipulationEngine_CPU<ITMVoxel, ITMPlainVoxelArray> SceneManipulationEngine;

	SceneManipulationEngine::ResetScene(&scene1);

	ITMVoxel voxelZero;
	voxelZero.sdf = 0.0f;
	SceneManipulationEngine::SetVoxel(&scene1, Vector3i(0, 0, 0), voxelZero);

	ITMVoxel out;
	out = SceneManipulationEngine::ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	ITMVoxel voxelHalf;
	voxelHalf.sdf = 0.5f;

	SceneManipulationEngine::SetVoxel(&scene1, Vector3i(1, 1, 1), voxelHalf);
	out = SceneManipulationEngine::ReadVoxel(&scene1, Vector3i(1, 1, 1));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	SceneManipulationEngine::SetVoxel(&scene1, Vector3i(9, 9, 9), voxelZero);
	SceneManipulationEngine::SetVoxel(&scene1, Vector3i(9, 9, 9), voxelHalf);
	SceneManipulationEngine::SetVoxel(&scene1, Vector3i(3, 3, 3), voxelHalf);
	out = SceneManipulationEngine::ReadVoxel(&scene1, Vector3i(9, 9, 9));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	Vector3i voxelPos(8, 5, 2);
	SceneManipulationEngine::SetVoxel(&scene1, voxelPos, voxelZero);
	out = SceneManipulationEngine::ReadVoxel(&scene1, voxelPos);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = SceneManipulationEngine::ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);

	Vector3i offset(-2, 3, 4);
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> scene2(&settings->sceneParams,
	                                                    settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                    settings->GetMemoryType());
	SceneManipulationEngine::ResetScene(&scene2);

	SceneManipulationEngine::CopyScene(&scene2, &scene1, offset);
	out = SceneManipulationEngine::ReadVoxel(&scene2, voxelPos + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = SceneManipulationEngine::ReadVoxel(&scene2, Vector3i(0, 0, 0) + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = SceneManipulationEngine::ReadVoxel(&scene2, Vector3i(3, 3, 3) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	out = SceneManipulationEngine::ReadVoxel(&scene2, Vector3i(1, 1, 1) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
}

BOOST_AUTO_TEST_CASE(testSetVoxelAndCopy_VoxelBlockHash_CPU) {
	ITMLibSettings* settings = &ITMLibSettings::Instance();

	settings->deviceType = ITMLibSettings::DEVICE_CPU;
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> scene1(&settings->sceneParams,
	                                                   settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                   settings->GetMemoryType());

	typedef ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash> SceneManipulationEngine;

	SceneManipulationEngine::ResetScene(&scene1);

	ITMVoxel voxelZero;
	voxelZero.sdf = 0.0f;
	SceneManipulationEngine::SetVoxel(&scene1, Vector3i(0, 0, 0), voxelZero);

	ITMVoxel out;
	out = SceneManipulationEngine::ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	ITMVoxel voxelHalf;
	voxelHalf.sdf = 0.5f;

	SceneManipulationEngine::SetVoxel(&scene1, Vector3i(1, 1, 1), voxelHalf);
	out = SceneManipulationEngine::ReadVoxel(&scene1, Vector3i(1, 1, 1));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	SceneManipulationEngine::SetVoxel(&scene1, Vector3i(9, 9, 9), voxelZero);
	SceneManipulationEngine::SetVoxel(&scene1, Vector3i(9, 9, 9), voxelHalf);
	out = SceneManipulationEngine::ReadVoxel(&scene1, Vector3i(9, 9, 9));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	Vector3i voxelPos(232, 125, 62);
	SceneManipulationEngine::SetVoxel(&scene1, voxelPos, voxelZero);
	out = SceneManipulationEngine::ReadVoxel(&scene1, voxelPos);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = SceneManipulationEngine::ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);

	Vector3i offset(-34, 6, 9);
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> scene2(&settings->sceneParams,
	                                                   settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                   settings->GetMemoryType());

	SceneManipulationEngine::CopyScene(&scene2, &scene1, offset);
	out = SceneManipulationEngine::ReadVoxel(&scene2, voxelPos + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = SceneManipulationEngine::ReadVoxel(&scene2, Vector3i(0, 0, 0) + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = SceneManipulationEngine::ReadVoxel(&scene2, Vector3i(9, 9, 9) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	out = SceneManipulationEngine::ReadVoxel(&scene2, Vector3i(1, 1, 1) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
}


BOOST_AUTO_TEST_CASE(testCompareVoxelVolumes_CPU) {
	typedef ITMSceneManipulationEngine_CPU<ITMVoxel, ITMPlainVoxelArray> PVA_ManipulationEngine;
	typedef ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash> VBH_ManipulationEngine;

	ITMLibSettings* settings = &ITMLibSettings::Instance();
	settings->deviceType = ITMLibSettings::DEVICE_CPU;

	Vector3i volumeSize(40);
	Vector3i volumeOffset(-20, -20, 0);
	Vector3i extentEndVoxel = volumeOffset + volumeSize;

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> scene1(&settings->sceneParams,
	                                                    settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                    settings->GetMemoryType(),
	                                                    volumeSize,
	                                                    volumeOffset);
	PVA_ManipulationEngine::ResetScene(&scene1);
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> scene2(&settings->sceneParams,
	                                                    settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                    settings->GetMemoryType(),
	                                                    volumeSize,
	                                                    volumeOffset);
	PVA_ManipulationEngine::ResetScene(&scene2);
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> scene3(&settings->sceneParams,
	                                                   settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                   settings->GetMemoryType());
	VBH_ManipulationEngine::ResetScene(&scene3);
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> scene4(&settings->sceneParams,
	                                                   settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                   settings->GetMemoryType());
	VBH_ManipulationEngine::ResetScene(&scene4);

	std::random_device random_device;
	std::mt19937 generator(random_device());
	std::uniform_real_distribution<float> sdf_distribution(-1.0f, 1.0f);
	std::uniform_int_distribution<int> coordinate_distribution(0, extentEndVoxel.x);
	const int modifiedVoxelCount = 120;

//	generate only in the positive coordinates' volume, to make sure that the unneeded voxel hash blocks are properly dismissed
	for (int iVoxel = 0; iVoxel < modifiedVoxelCount; iVoxel++) {
		ITMVoxel voxel;
		voxel.sdf = sdf_distribution(generator);
		Vector3i coordinate(coordinate_distribution(generator),
		                    coordinate_distribution(generator),
		                    coordinate_distribution(generator));
		PVA_ManipulationEngine::SetVoxel(&scene1, coordinate, voxel);
		PVA_ManipulationEngine::SetVoxel(&scene2, coordinate, voxel);
		VBH_ManipulationEngine::SetVoxel(&scene3, coordinate, voxel);
		VBH_ManipulationEngine::SetVoxel(&scene4, coordinate, voxel);
	}
	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene3, &scene4, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene1, &scene3, tolerance));

	std::uniform_int_distribution<int> coordinate_distribution2(volumeOffset.x, 0);
	ITMVoxel voxel;
	voxel.sdf = -0.1;
	Vector3i coordinate(coordinate_distribution2(generator),
	                    coordinate_distribution2(generator),
	                    0);
	PVA_ManipulationEngine::SetVoxel(&scene2, coordinate, voxel);
	VBH_ManipulationEngine::SetVoxel(&scene4, coordinate, voxel);
	BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
	BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene3, &scene4, tolerance));
	BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene4, tolerance));
}

BOOST_AUTO_TEST_CASE(testGenerateVoxelVolumeFromImage_CPU_PlainVoxelArray) {
	ITMLibSettings* settings = &ITMLibSettings::Instance();

	// region ================================= CONSTRUCT VIEW =========================================================

	settings->deviceType = ITMLibSettings::DEVICE_CPU;
	settings->useBilateralFilter = false;
	settings->useThresholdFilter = false;

	ITMRGBDCalib calibrationData;
	readRGBDCalib("TestData/snoopy_calib.txt", calibrationData);

	ITMViewBuilder* viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calibrationData, settings->deviceType);
	Vector2i imageSize(640, 480);
	ITMView* view = nullptr;

	auto* rgb = new ITMUChar4Image(true, false);
	auto* depth = new ITMShortImage(true, false);
	BOOST_REQUIRE(ReadImageFromFile(rgb, "TestData/stripes_color.png"));
	BOOST_REQUIRE(ReadImageFromFile(depth, "TestData/stripes_depth.png"));

	viewBuilder->UpdateView(&view, rgb, depth, settings->useThresholdFilter,
	                        settings->useBilateralFilter, false, true);

	// endregion =======================================================================================================

	ITMDynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMPlainVoxelArray>* reconstructionEngine =
			ITMDynamicSceneReconstructionEngineFactory
			::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMPlainVoxelArray>(settings->deviceType);

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> scene1(&settings->sceneParams,
	                                                    settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                    settings->GetMemoryType());
	ITMTrackingState trackingState(imageSize, settings->GetMemoryType());

	reconstructionEngine->GenerateRawLiveSceneFromView(&scene1, view, &trackingState, nullptr);


	int relevant_voxel_x_coords_mm[] = {-142, -160, -176, -191, -205, -217, -227, -236, -244, -250, -254,
	                                    -256, -258, -257, -255, -252, -247, -240, -232, -222, -211, -198,
	                                    -184, -168, -151, -132, -111, -89, -66, -41, -14, 14, 44,
	                                    75, 108, 143, 179, 216, 255, 296, 338, 382, 427, 474,
	                                    522, 572, 624, 677, 731, 787, 845, 904, 965, 1027, 1091,
	                                    1156, 1223, 1292, 1362, 1433, 1506, 1581};
	int relevant_voxel_z_coords_mm[] = {240, 280, 320, 360, 400, 440, 480, 520, 560, 600, 640,
	                                    680, 720, 760, 800, 840, 880, 920, 960, 1000, 1040, 1080,
	                                    1120, 1160, 1200, 1240, 1280, 1320, 1360, 1400, 1440, 1480, 1520,
	                                    1560, 1600, 1640, 1680, 1720, 1760, 1800, 1840, 1880, 1920, 1960,
	                                    2000, 2040, 2080, 2120, 2160, 2200, 2240, 2280, 2320, 2360, 2400,
	                                    2440, 2480, 2520, 2560, 2600, 2640, 2680};

	delete view;
	delete reconstructionEngine;
	delete viewBuilder;
	delete rgb;
	delete depth;
}


//TODO: restore
//BOOST_AUTO_TEST_CASE(testSceneSaveLoadCompact) {
//	ITMLibSettings* settings = &ITMLibSettings::Instance();
//
//	auto scene1 = new ITMVoxelVolume<ITMVoxel, ITMVoxelIndex>(
//			&settings->sceneParams, settings->swappingMode ==
//			                        ITMLibSettings::SWAPPINGMODE_ENABLED, settings->GetMemoryType());
//
//	auto scene2 = new ITMVoxelVolume<ITMVoxel, ITMVoxelIndex>(
//			&settings->sceneParams, settings->swappingMode ==
//			                        ITMLibSettings::SWAPPINGMODE_ENABLED, settings->GetMemoryType());
//	typedef ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelIndex> CanonicalSceneManipulationEngine;
//
//	GenerateTestScene01(scene1);
//	std::string path = "test_";
//	ITMSceneFileIOEngine<ITMVoxel, ITMVoxelIndex>::SaveToDirectoryCompact(scene1, path);
//
//	CanonicalSceneManipulationEngine::ResetScene(scene2);
//
//	ITMSceneFileIOEngine<ITMVoxel, ITMVoxelIndex>::LoadFromDirectoryCompact(scene2, path);
//
//	ITMSceneStatisticsCalculator<ITMVoxel, ITMVoxelIndex>& calc =
//			ITMSceneStatisticsCalculator<ITMVoxel, ITMVoxelIndex>::Instance();
//	std::vector<int> hashes1 = calc.GetFilledHashBlockIds(scene1);
//	std::vector<int> hashes2 = calc.GetFilledHashBlockIds(scene2);
//
//	BOOST_REQUIRE(hashes1.size() == hashes2.size());
//	for (int iHash = 0; iHash < hashes1.size(); iHash++) {
//		BOOST_REQUIRE(hashes1[iHash] == hashes2[iHash]);
//	}
//
//	delete settings;
//	delete scene1;
//	delete scene2;
//}
