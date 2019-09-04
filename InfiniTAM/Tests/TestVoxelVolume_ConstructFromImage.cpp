//  ================================================================
//  Created by Gregory Kramida on 9/4/19.
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
#include "../ITMLib/Engines/Reconstruction/Interface/ITMDynamicSceneReconstructionEngine.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"
#include "../ITMLib/Engines/ViewBuilding/Interface/ITMViewBuilder.h"
#include "../ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../ORUtils/FileUtils.h"

using namespace ITMLib;

BOOST_AUTO_TEST_CASE(testConstructVoxelVolumeFromImage_CPU_PlainVoxelArray) {
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

	auto getVoxelCoord = [](Vector3f coordinateMeters, float voxelSize) {
		return (coordinateMeters / voxelSize).toInt();
	};
	const int num_stripes = 62;
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

	std::vector<Vector3i> relevantVoxelCoords;
	for (int iVoxel = 0; iVoxel < num_stripes; iVoxel++) {
		Vector3f coordinateMeters(relevant_voxel_x_coords_mm[iVoxel], 0.0f, relevant_voxel_z_coords_mm[iVoxel]);
		relevantVoxelCoords.push_back(getVoxelCoord(coordinateMeters, settings->sceneParams.voxelSize));
	}




	delete view;
	delete reconstructionEngine;
	delete viewBuilder;
	delete rgb;
	delete depth;
}