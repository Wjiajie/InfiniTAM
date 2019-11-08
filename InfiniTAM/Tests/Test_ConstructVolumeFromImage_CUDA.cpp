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
#include "../ITMLib/Engines/Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"
#include "../ITMLib/Engines/ViewBuilding/Interface/ITMViewBuilder.h"
#include "../ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Utils/Analytics/ITMAlmostEqual.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#include "../ORUtils/FileUtils.h"
#include "../ITMLib/Objects/RenderStates/ITMRenderStateFactory.h"
#include "../ITMLib/Engines/SceneFileIO/ITMSceneFileIOEngine.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"

using namespace ITMLib;

typedef ITMSceneFileIOEngine<ITMVoxel, ITMPlainVoxelArray> SceneFileIOEngine_PVA;
typedef ITMSceneFileIOEngine<ITMVoxel, ITMVoxelBlockHash> SceneFileIOEngine_VBH;
typedef ITMSceneStatisticsCalculator_CUDA<ITMVoxel, ITMPlainVoxelArray> SceneStatisticsCalculator_PVA;
typedef ITMSceneStatisticsCalculator_CUDA<ITMVoxel, ITMVoxelBlockHash> SceneStatisticsCalculator_VBH;

BOOST_AUTO_TEST_CASE(testConstructVoxelVolumeFromImage_CUDA) {
	Configuration* settings = &Configuration::Instance();

	// region ================================= CONSTRUCT VIEW =========================================================

	settings->deviceType = MEMORYDEVICE_CUDA;
	settings->useBilateralFilter = false;
	settings->useThresholdFilter = false;
	settings->sceneParams.mu = 0.04; // non-truncated narrow-band half-width, in meters
	settings->sceneParams.voxelSize = 0.004; // m

	ITMRGBDCalib calibrationData;
	readRGBDCalib("TestData/snoopy_calib.txt", calibrationData);

	ITMViewBuilder* viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calibrationData, settings->deviceType);
	Vector2i imageSize(640, 480);
	ITMView* view = nullptr;

	auto* rgb = new ITMUChar4Image(true, true);
	auto* depth = new ITMShortImage(true, true);
	BOOST_REQUIRE(ReadImageFromFile(rgb, "TestData/stripes_color.png"));
	BOOST_REQUIRE(ReadImageFromFile(depth, "TestData/stripes_depth.png"));
	rgb->UpdateDeviceFromHost();
	depth->UpdateDeviceFromHost();

	viewBuilder->UpdateView(&view, rgb, depth, settings->useThresholdFilter,
	                        settings->useBilateralFilter, false, true);

	// endregion =======================================================================================================

	Vector3i volumeSize(1024, 32, 1024), volumeOffset(-volumeSize.x / 2, -volumeSize.y / 2, 0);

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> scene1(&settings->sceneParams,
	                                                    settings->swappingMode == Configuration::SWAPPINGMODE_ENABLED,
	                                                    settings->GetMemoryType(),
	                                                    {volumeSize, volumeOffset});
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetScene(&scene1);
	ITMTrackingState trackingState(imageSize, settings->GetMemoryType());

	ITMDynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMPlainVoxelArray>* reconstructionEngine_PVA =
			ITMDynamicSceneReconstructionEngineFactory
			::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMPlainVoxelArray>(settings->deviceType);
	reconstructionEngine_PVA->GenerateRawLiveSceneFromView(&scene1, view, &trackingState, nullptr);

	const int num_stripes = 62;

	//These hardcoded values were precomputed mathematically based on the generated stripe images
	int zero_level_set_voxel_x_coords_mm[] = {-142, -160, -176, -191, -205, -217, -227, -236, -244, -250, -254,
	                                          -256, -258, -257, -255, -252, -247, -240, -232, -222, -211, -198,
	                                          -184, -168, -151, -132, -111, -89, -66, -41, -14, 14, 44,
	                                          75, 108, 143, 179, 216, 255, 296, 338, 382, 427, 474,
	                                          522, 572, 624, 677, 731, 787, 845, 904, 965, 1027, 1091,
	                                          1156, 1223, 1292, 1362, 1433, 1506, 1581};
	int zero_level_set_voxel_z_coords_mm[] = {240, 280, 320, 360, 400, 440, 480, 520, 560, 600, 640,
	                                          680, 720, 760, 800, 840, 880, 920, 960, 1000, 1040, 1080,
	                                          1120, 1160, 1200, 1240, 1280, 1320, 1360, 1400, 1440, 1480, 1520,
	                                          1560, 1600, 1640, 1680, 1720, 1760, 1800, 1840, 1880, 1920, 1960,
	                                          2000, 2040, 2080, 2120, 2160, 2200, 2240, 2280, 2320, 2360, 2400,
	                                          2440, 2480, 2520, 2560, 2600, 2640, 2680};

	std::vector<Vector3i> zeroLevelSetCoords;
	auto getVoxelCoord = [](Vector3f coordinateMeters, float voxelSize) {
		return TO_INT_ROUND3(coordinateMeters / voxelSize);
	};
	for (int iVoxel = 0; iVoxel < num_stripes; iVoxel++) {
		Vector3f coordinateMeters(
				static_cast<float>(zero_level_set_voxel_x_coords_mm[iVoxel]) / 1000.0f,
				0.0f,
				static_cast<float>(zero_level_set_voxel_z_coords_mm[iVoxel]) / 1000.0f
		);
		zeroLevelSetCoords.push_back(getVoxelCoord(coordinateMeters, settings->sceneParams.voxelSize));
	}

	float tolerance = 1e-4;
	int narrowBandHalfwidthVoxels = static_cast<int>(std::round(
			scene1.sceneParams->mu / scene1.sceneParams->voxelSize));
	float maxSdfStep = 1.0f / narrowBandHalfwidthVoxels;

	// check constructed scene integrity
	for (int iCoord = 0; iCoord < zeroLevelSetCoords.size(); iCoord++) {
		Vector3i coord = zeroLevelSetCoords[iCoord];
		ITMVoxel voxel = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&scene1, coord);
		float sdf = ITMVoxel::valueToFloat(voxel.sdf);
		BOOST_REQUIRE(almostEqual(sdf, 0.0f, tolerance));

		// for extremely lateral points close to the camera, the rays are highly skewed,
		// so the value progression won't hold. Skip those.
		if (iCoord > 17) {
			// don't go into low negative sdf values, since those will be overwritten by positive values
			// during sdf construction in certain cases
			for (int iLevelSet = -narrowBandHalfwidthVoxels; iLevelSet < (narrowBandHalfwidthVoxels / 2); iLevelSet++) {
				Vector3i augmentedCoord(coord.x, coord.y, coord.z + iLevelSet);
				float expectedSdf = -static_cast<float>(iLevelSet) * maxSdfStep;
				voxel = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&scene1, augmentedCoord);
				sdf = ITMVoxel::valueToFloat(voxel.sdf);
				BOOST_REQUIRE(almostEqual(sdf, expectedSdf, tolerance));
			}
		}
	}

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> scene2(&settings->sceneParams,
	                                                   settings->swappingMode == Configuration::SWAPPINGMODE_ENABLED,
	                                                   settings->GetMemoryType(), {0x800, 0x20000});
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetScene(&scene2);

	ITMDynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMVoxelBlockHash>* reconstructionEngine_VBH =
			ITMDynamicSceneReconstructionEngineFactory
			::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMVoxelBlockHash>(settings->deviceType);

	ITMRenderState* renderState = ITMRenderStateFactory<ITMVoxelBlockHash>::CreateRenderState(imageSize,
	                                                                                          &settings->sceneParams,
	                                                                                          settings->GetMemoryType(),
	                                                                                          scene2.index);
	reconstructionEngine_VBH->GenerateRawLiveSceneFromView(&scene2, view, &trackingState, renderState);

	tolerance = 1e-5;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(&scene1, &scene2, tolerance));
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> scene3(&settings->sceneParams,
	                                                    settings->swappingMode == Configuration::SWAPPINGMODE_ENABLED,
	                                                    settings->GetMemoryType(),
	                                                    {volumeSize, volumeOffset});
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetScene(&scene3);
	reconstructionEngine_PVA->GenerateRawLiveSceneFromView(&scene3, view, &trackingState, nullptr);
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene1, &scene3, tolerance));
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> scene4(&settings->sceneParams,
	                                                   settings->swappingMode == Configuration::SWAPPINGMODE_ENABLED,
	                                                   settings->GetMemoryType(), {0x800, 0x20000});
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetScene(&scene4);
	reconstructionEngine_VBH->GenerateRawLiveSceneFromView(&scene4, view, &trackingState, renderState);
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene2, &scene4, tolerance));

	Vector3i coordinate = zeroLevelSetCoords[0];
	ITMVoxel voxel = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&scene3, coordinate);
	voxel.sdf = ITMVoxel::floatToValue(ITMVoxel::valueToFloat(voxel.sdf) + 0.05f);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene3, coordinate, voxel);
	ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&scene2, coordinate, voxel);

	BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene1, &scene3, tolerance));
	BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene2, &scene4, tolerance));
	BOOST_REQUIRE(!allocatedContentAlmostEqual_CUDA(&scene1, &scene2, tolerance));

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> scene5(
			&settings->sceneParams, settings->swappingMode == Configuration::SWAPPINGMODE_ENABLED,
			settings->GetMemoryType(), {0x800, 0x20000});
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetScene(&scene5);
	std::string path = "TestData/test_VBH_ConstructFromImage_";
	SceneFileIOEngine_VBH::SaveToDirectoryCompact(&scene4, path);
	SceneFileIOEngine_VBH::LoadFromDirectoryCompact(&scene5, path);
	BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(&scene1, &scene5, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene4, &scene5, tolerance));

	delete view;
	delete reconstructionEngine_PVA;
	delete reconstructionEngine_VBH;
	delete viewBuilder;
	delete rgb;
	delete depth;
}

BOOST_AUTO_TEST_CASE(testConstructVoxelVolumeFromImage2_CUDA) {
	Configuration* settings = &Configuration::Instance();

	// region ================================= CONSTRUCT VIEW =========================================================

	settings->deviceType = MEMORYDEVICE_CUDA;
	settings->useBilateralFilter = false;
	settings->useThresholdFilter = false;
	settings->sceneParams.mu = 0.04; // non-truncated narrow-band half-width, in meters
	settings->sceneParams.voxelSize = 0.004; // m

	ITMRGBDCalib calibrationData;
	readRGBDCalib("TestData/snoopy_calib.txt", calibrationData);

	ITMViewBuilder* viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calibrationData, settings->deviceType);
	Vector2i imageSize(640, 480);
	ITMView* view = nullptr;

	auto* rgb = new ITMUChar4Image(true, false);
	auto* depth = new ITMShortImage(true, false);
	BOOST_REQUIRE(ReadImageFromFile(rgb, "TestData/snoopy_color_000000.png"));
	BOOST_REQUIRE(ReadImageFromFile(depth, "TestData/snoopy_depth_000000.png"));

	viewBuilder->UpdateView(&view, rgb, depth, settings->useThresholdFilter,
	                        settings->useBilateralFilter, false, true);

	// endregion =======================================================================================================

	Vector3i volumeSize(512, 512, 512), volumeOffset(-volumeSize.x / 2, -volumeSize.y / 2, 0);
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> scene1(&settings->sceneParams,
	                                                    settings->swappingMode == Configuration::SWAPPINGMODE_ENABLED,
	                                                    settings->GetMemoryType(),
	                                                    {volumeSize, volumeOffset});

	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetScene(&scene1);
	ITMTrackingState trackingState(imageSize, settings->GetMemoryType());

	ITMDynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMPlainVoxelArray>* reconstructionEngine_PVA =
			ITMDynamicSceneReconstructionEngineFactory
			::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMPlainVoxelArray>(settings->deviceType);
	reconstructionEngine_PVA->GenerateRawLiveSceneFromView(&scene1, view, &trackingState, nullptr);

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> scene2(&settings->sceneParams,
	                                                    settings->swappingMode == Configuration::SWAPPINGMODE_ENABLED,
	                                                    settings->GetMemoryType(),
	                                                    {volumeSize, volumeOffset});

	std::string path = "TestData/test_PVA_ConstructFromImage2_";
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetScene(&scene2);
	SceneFileIOEngine_PVA::LoadFromDirectoryCompact(&scene2, path);

	float tolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene1, &scene2, tolerance));

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> scene3(&settings->sceneParams,
	                                                   settings->swappingMode == Configuration::SWAPPINGMODE_ENABLED,
	                                                   settings->GetMemoryType());
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetScene(&scene3);

	ITMDynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMVoxelBlockHash>* reconstructionEngine_VBH =
			ITMDynamicSceneReconstructionEngineFactory
			::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, ITMVoxelBlockHash>(settings->deviceType);

	ITMRenderState* renderState = ITMRenderStateFactory<ITMVoxelBlockHash>::CreateRenderState(imageSize,
	                                                                                          &settings->sceneParams,
	                                                                                          settings->GetMemoryType(),
	                                                                                          scene3.index);
	reconstructionEngine_VBH->GenerateRawLiveSceneFromView(&scene3, view, &trackingState, renderState);

	BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(&scene2, &scene3, tolerance));

	delete rgb;
	delete depth;
}