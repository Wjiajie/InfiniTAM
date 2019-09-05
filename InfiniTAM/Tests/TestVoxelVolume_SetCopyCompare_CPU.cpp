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
#include <random>
#include <vector>


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



BOOST_AUTO_TEST_CASE(testCompareVoxelVolumes_CPU_ITMVoxel) {
	typedef ITMSceneManipulationEngine_CPU<ITMVoxel, ITMPlainVoxelArray> PVA_ManipulationEngine;
	typedef ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash> VBH_ManipulationEngine;
	float tolerance = 1e-6;

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

	auto singleVoxelTests = [&]() {
		std::uniform_int_distribution<int> coordinate_distribution2(volumeOffset.x, 0);
		ITMVoxel voxel;
		voxel.sdf = ITMVoxel::floatToValue(-0.1f);
		simulateVoxelAlteration(voxel);

		Vector3i coordinate(coordinate_distribution2(generator), coordinate_distribution2(generator), 0);

		PVA_ManipulationEngine::SetVoxel(&scene2, coordinate, voxel);
		VBH_ManipulationEngine::SetVoxel(&scene4, coordinate, voxel);
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene3, &scene4, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene4, tolerance));

		ITMVoxel defaultVoxel;
		PVA_ManipulationEngine::SetVoxel(&scene2, coordinate, defaultVoxel);
		VBH_ManipulationEngine::SetVoxel(&scene4, coordinate, defaultVoxel);
		BOOST_REQUIRE(contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
		BOOST_REQUIRE(contentAlmostEqual_CPU(&scene3, &scene4, tolerance));

		coordinate = volumeOffset + volumeSize - Vector3i(1);
		voxel = PVA_ManipulationEngine::ReadVoxel(&scene2, coordinate);
		voxel.sdf = fmod((voxel.sdf + 0.1f), 1.0f);
		simulateVoxelAlteration(voxel);
		PVA_ManipulationEngine::SetVoxel(&scene2, coordinate, voxel);
		VBH_ManipulationEngine::SetVoxel(&scene4, coordinate, voxel);
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene3, &scene4, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene4, tolerance));

		PVA_ManipulationEngine::SetVoxel(&scene2, coordinate, defaultVoxel);
		VBH_ManipulationEngine::SetVoxel(&scene4, coordinate, defaultVoxel);
	};

	std::uniform_real_distribution<float> sdf_distribution(-1.0f, 1.0f);
	std::uniform_int_distribution<int> coordinate_distribution(0, extentEndVoxel.x);

	const int modifiedVoxelCount = 120;

	singleVoxelTests();

//	generate only in the positive coordinates' volume, to make sure that the unneeded voxel hash blocks are properly dismissed
	for (int iVoxel = 0; iVoxel < modifiedVoxelCount; iVoxel++) {
		ITMVoxel voxel;
		voxel.sdf = sdf_distribution(generator);
		simulateVoxelAlteration(voxel);
		Vector3i coordinate(coordinate_distribution(generator),
		                    coordinate_distribution(generator),
		                    coordinate_distribution(generator));

		PVA_ManipulationEngine::SetVoxel(&scene1, coordinate, voxel);
		PVA_ManipulationEngine::SetVoxel(&scene2, coordinate, voxel);
		VBH_ManipulationEngine::SetVoxel(&scene3, coordinate, voxel);
		VBH_ManipulationEngine::SetVoxel(&scene4, coordinate, voxel);
	}

	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene3, &scene4, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene1, &scene3, tolerance));

	singleVoxelTests();
}

BOOST_AUTO_TEST_CASE(testCompareVoxelVolumes_CPU_ITMWarp) {
	typedef ITMSceneManipulationEngine_CPU<ITMWarp, ITMPlainVoxelArray> PVA_ManipulationEngine;
	typedef ITMSceneManipulationEngine_CPU<ITMWarp, ITMVoxelBlockHash> VBH_ManipulationEngine;
	float tolerance = 1e-6;

	ITMLibSettings* settings = &ITMLibSettings::Instance();
	settings->deviceType = ITMLibSettings::DEVICE_CPU;

	Vector3i volumeSize(40);
	Vector3i volumeOffset(-20, -20, 0);
	Vector3i extentEndVoxel = volumeOffset + volumeSize;

	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> scene1(&settings->sceneParams,
	                                                   settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                   settings->GetMemoryType(),
	                                                   volumeSize,
	                                                   volumeOffset);
	PVA_ManipulationEngine::ResetScene(&scene1);
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> scene2(&settings->sceneParams,
	                                                   settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                   settings->GetMemoryType(),
	                                                   volumeSize,
	                                                   volumeOffset);
	PVA_ManipulationEngine::ResetScene(&scene2);
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> scene3(&settings->sceneParams,
	                                                  settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                  settings->GetMemoryType());
	VBH_ManipulationEngine::ResetScene(&scene3);
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> scene4(&settings->sceneParams,
	                                                  settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                  settings->GetMemoryType());
	VBH_ManipulationEngine::ResetScene(&scene4);

	std::random_device random_device;
	std::mt19937 generator(random_device());

	auto singleVoxelTests = [&]() {
		std::uniform_int_distribution<int> coordinate_distribution2(volumeOffset.x, 0);
		ITMWarp warp;
		warp.flow_warp = Vector3f(-0.1);

		Vector3i coordinate(coordinate_distribution2(generator), coordinate_distribution2(generator), 0);

		PVA_ManipulationEngine::SetVoxel(&scene2, coordinate, warp);
		VBH_ManipulationEngine::SetVoxel(&scene4, coordinate, warp);
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene3, &scene4, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene4, tolerance));

		ITMWarp defaultVoxel;
		PVA_ManipulationEngine::SetVoxel(&scene2, coordinate, defaultVoxel);
		VBH_ManipulationEngine::SetVoxel(&scene4, coordinate, defaultVoxel);
		BOOST_REQUIRE(contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
		BOOST_REQUIRE(contentAlmostEqual_CPU(&scene3, &scene4, tolerance));

		coordinate = volumeOffset + volumeSize - Vector3i(1);
		warp = PVA_ManipulationEngine::ReadVoxel(&scene2, coordinate);
		warp.warp_update += Vector3f(0.1);
		PVA_ManipulationEngine::SetVoxel(&scene2, coordinate, warp);
		VBH_ManipulationEngine::SetVoxel(&scene4, coordinate, warp);
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene3, &scene4, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene4, tolerance));

		PVA_ManipulationEngine::SetVoxel(&scene2, coordinate, defaultVoxel);
		VBH_ManipulationEngine::SetVoxel(&scene4, coordinate, defaultVoxel);
	};

	std::uniform_real_distribution<float> warp_distribution(-1.0f, 1.0f);
	std::uniform_int_distribution<int> coordinate_distribution(0, extentEndVoxel.x);

	const int modifiedWarpCount = 120;

	singleVoxelTests();

//	generate only in the positive coordinates' volume, to make sure that the unneeded voxel hash blocks are properly dismissed
	for (int iWarp = 0; iWarp < modifiedWarpCount; iWarp++) {
		ITMWarp warp;
		Vector3f flow_warp(warp_distribution(generator), warp_distribution(generator), warp_distribution(generator));
		warp.flow_warp = flow_warp;

		Vector3i coordinate(coordinate_distribution(generator),
		                    coordinate_distribution(generator),
		                    coordinate_distribution(generator));

		PVA_ManipulationEngine::SetVoxel(&scene1, coordinate, warp);
		PVA_ManipulationEngine::SetVoxel(&scene2, coordinate, warp);
		VBH_ManipulationEngine::SetVoxel(&scene3, coordinate, warp);
		VBH_ManipulationEngine::SetVoxel(&scene4, coordinate, warp);
	}

	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene3, &scene4, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene1, &scene3, tolerance));

	singleVoxelTests();
}


