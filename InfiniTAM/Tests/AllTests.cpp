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

#define BOOST_TEST_MODULE AllTests
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>


//ITMlib
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Scene/ITMScene.h"
#include "../ITMLib/Objects/Scene/ITMRepresentationAccess.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "../ITMLib/Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"

//local
#include "TestUtils.h"
#include "../ITMLib/Utils/Analytics/ITMSceneStatisticsCalculator.h"
#include "../ORUtils/FileUtils.h"
#include "../InputSource/ImageSourceEngine.h"
#include "../ITMLib/Utils/Collections/ITM3DNestedMapOfArrays.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Engines/Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"
#include "../ITMLib/Engines/SceneFileIO/ITMSceneFileIOEngine.h"

using namespace ITMLib;


BOOST_AUTO_TEST_CASE(testSetVoxelAndCopyScene_PlainVoxelArray) {
	ITMLibSettings* settings = &ITMLibSettings::Instance();
	settings->deviceType = ITMLibSettings::DEVICE_CPU;
	ITMScene<ITMVoxelCanonical, ITMPlainVoxelArray> scene1(&settings->sceneParams,
	                                                       settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                       settings->GetMemoryType());

	typedef ITMSceneManipulationEngine_CPU<ITMVoxelCanonical, ITMPlainVoxelArray> CanonicalSceneManipulationEngine;
	typedef ITMSceneManipulationEngine_CPU<ITMVoxelLive, ITMPlainVoxelArray> LiveSceneManipulationEngine;


	CanonicalSceneManipulationEngine::ResetScene(&scene1);

	ITMVoxelCanonical voxelZero;
	voxelZero.sdf = 0.0f;
	CanonicalSceneManipulationEngine::SetVoxel(&scene1, Vector3i(0, 0, 0), voxelZero);

	ITMVoxelCanonical out;
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	ITMVoxelCanonical voxelHalf;
	voxelHalf.sdf = 0.5f;

	CanonicalSceneManipulationEngine::SetVoxel(&scene1, Vector3i(1, 1, 1), voxelHalf);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene1, Vector3i(1, 1, 1));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	CanonicalSceneManipulationEngine::SetVoxel(&scene1, Vector3i(9, 9, 9), voxelZero);
	CanonicalSceneManipulationEngine::SetVoxel(&scene1, Vector3i(9, 9, 9), voxelHalf);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene1, Vector3i(9, 9, 9));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	Vector3i voxelPos(232, 125, 62);
	CanonicalSceneManipulationEngine::SetVoxel(&scene1, voxelPos, voxelZero);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene1, voxelPos);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);

	Vector3i offset(-34, 6, 9);
	ITMScene<ITMVoxelCanonical, ITMPlainVoxelArray> scene2(&settings->sceneParams,
	                                                       settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                       settings->GetMemoryType());
	CanonicalSceneManipulationEngine::ResetScene(&scene2);

	CanonicalSceneManipulationEngine::CopyScene(&scene2, &scene1, offset);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene2, voxelPos + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene2, Vector3i(0, 0, 0) + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene2, Vector3i(9, 9, 9) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene2, Vector3i(1, 1, 1) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
}

BOOST_AUTO_TEST_CASE(testSetVoxelAndCopyScene_VoxelBlockHash) {
	ITMLibSettings* settings = &ITMLibSettings::Instance();
	settings->deviceType = ITMLibSettings::DEVICE_CPU;
	ITMScene<ITMVoxelCanonical, ITMVoxelBlockHash> scene1(&settings->sceneParams,
	                                                      settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                      settings->GetMemoryType());

	typedef ITMSceneManipulationEngine_CPU<ITMVoxelCanonical, ITMVoxelBlockHash> CanonicalSceneManipulationEngine;
	typedef ITMSceneManipulationEngine_CPU<ITMVoxelLive, ITMVoxelBlockHash> LiveSceneManipulationEngine;


	CanonicalSceneManipulationEngine::ResetScene(&scene1);

	ITMVoxelCanonical voxelZero;
	voxelZero.sdf = 0.0f;
	CanonicalSceneManipulationEngine::SetVoxel(&scene1, Vector3i(0, 0, 0), voxelZero);

	ITMVoxelCanonical out;
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	ITMVoxelCanonical voxelHalf;
	voxelHalf.sdf = 0.5f;

	CanonicalSceneManipulationEngine::SetVoxel(&scene1, Vector3i(1, 1, 1), voxelHalf);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene1, Vector3i(1, 1, 1));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	CanonicalSceneManipulationEngine::SetVoxel(&scene1, Vector3i(9, 9, 9), voxelZero);
	CanonicalSceneManipulationEngine::SetVoxel(&scene1, Vector3i(9, 9, 9), voxelHalf);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene1, Vector3i(9, 9, 9));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	Vector3i voxelPos(232, 125, 62);
	CanonicalSceneManipulationEngine::SetVoxel(&scene1, voxelPos, voxelZero);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene1, voxelPos);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);

	Vector3i offset(-34, 6, 9);
	ITMScene<ITMVoxelCanonical, ITMVoxelBlockHash> scene2(&settings->sceneParams,
	                                                      settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                      settings->GetMemoryType());
	CanonicalSceneManipulationEngine::ResetScene(&scene2);

	CanonicalSceneManipulationEngine::CopyScene(&scene2, &scene1, offset);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene2, voxelPos + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene2, Vector3i(0, 0, 0) + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene2, Vector3i(9, 9, 9) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	out = CanonicalSceneManipulationEngine::ReadVoxel(&scene2, Vector3i(1, 1, 1) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
}

BOOST_AUTO_TEST_CASE(testITMIntArrayMap3D) {
	ITM3DNestedMapOfArrays<int> map("one", "two", "three", "four");
	const int maxElementsOnEachLevel = 3;

	for (int keyLevel3 = 0; keyLevel3 < maxElementsOnEachLevel; keyLevel3++) {
		for (int keyLevel2 = 0; keyLevel2 < maxElementsOnEachLevel; keyLevel2++) {
			for (int keyLevel1 = 0; keyLevel1 < maxElementsOnEachLevel; keyLevel1++) {
				for (int valueLevel0 = 0; valueLevel0 < maxElementsOnEachLevel; valueLevel0++) {
					map.InsertOrdered(keyLevel3, keyLevel2, keyLevel1, valueLevel0);
				}
			}
		}
	}
//	std::cout << __FILE__ << ": " << __LINE__ << ": map to save." << std::endl;
//	std::cout << map << std::endl;
	const char* testFilename = "int_array_map_test.dat";
	map.SaveToFile(testFilename);
	ITM3DNestedMapOfArrays<int> map2("one", "two", "three", "four");
	map2.LoadFromFile(testFilename);
	BOOST_REQUIRE(map == map2);


	ITM3DNestedMapOfArrays<int> map3("one", "two", "three", "four");
	map3.InsertOrdered(84651, 358, 1, 5);
	map3.InsertOrdered(84651, 358, 1, 6);
	map3.InsertOrdered(102821, 436, 1, 1);
	map3.InsertOrdered(155667, 495, 1, 2);
	map3.InsertOrdered(179874, 446, 1, 28);
	map3.InsertOrdered(179874, 446, 1, 30);
	map3.SaveToFile(testFilename);
	ITM3DNestedMapOfArrays<int> map4("one", "two", "three", "four");
	map4.LoadFromFile(testFilename);
	BOOST_REQUIRE(map3 == map4);
}

//BOOST_AUTO_TEST_CASE(testSceneSaveLoadCompact) {
//	ITMLibSettings* settings = &ITMLibSettings::Instance();
//
//	auto scene1 = new ITMScene<ITMVoxelCanonical, ITMVoxelIndex>(
//			&settings->sceneParams, settings->swappingMode ==
//			                        ITMLibSettings::SWAPPINGMODE_ENABLED, settings->GetMemoryType());
//
//	auto scene2 = new ITMScene<ITMVoxelCanonical, ITMVoxelIndex>(
//			&settings->sceneParams, settings->swappingMode ==
//			                        ITMLibSettings::SWAPPINGMODE_ENABLED, settings->GetMemoryType());
//	typedef ITMSceneManipulationEngine_CPU<ITMVoxelCanonical, ITMVoxelIndex> CanonicalSceneManipulationEngine;
//
//	GenerateTestScene01(scene1);
//	std::string path = "test_";
//	ITMSceneFileIOEngine<ITMVoxelCanonical, ITMVoxelIndex>::SaveToDirectoryCompact(scene1, path);
//
//	CanonicalSceneManipulationEngine::ResetScene(scene2);
//
//	ITMSceneFileIOEngine<ITMVoxelCanonical, ITMVoxelIndex>::LoadFromDirectoryCompact(scene2, path);
//
//	ITMSceneStatisticsCalculator<ITMVoxelCanonical, ITMVoxelIndex>& calc =
//			ITMSceneStatisticsCalculator<ITMVoxelCanonical, ITMVoxelIndex>::Instance();
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

//TODO: provide test data (put in Tests/Data directory, copy from there to tests runtime dir via CMake), then uncomment
//BOOST_AUTO_TEST_CASE(testImageMaskReader) {
//
//	using namespace InputSource;
//	auto* rgb = new ITMUChar4Image(true, false);
//	auto* depth = new ITMShortImage(true, false);
//	auto* gtMaskedRgb = new ITMUChar4Image(true, false);
//	auto* gtMaskedDepth = new ITMShortImage(true, false);
//
//	ITMUCharImage* mask = new ITMUCharImage(true, false);
//
//	InputSource::ImageMaskPathGenerator pathGenerator("frames/color_%06i.png", "frames/depth_%06i.png",
//	                                                  "frames/omask_%06i.png");
//	InputSource::ImageSourceEngine* imageSource = new InputSource::ImageFileReader<InputSource::ImageMaskPathGenerator>(
//			"frames/snoopy_calib.txt", pathGenerator);
//	imageSource->getImages(rgb, depth);
//
////	BOOST_REQUIRE(ReadImageFromFile(rgb, "frames/color_000000.png"));
////	BOOST_REQUIRE(ReadImageFromFile(depth, "frames/depth_000000.png"));
//	BOOST_REQUIRE(ReadImageFromFile(mask, "frames/omask_000000.png"));
//
////	rgb->ApplyMask(*mask,Vector4u((unsigned char)0));
////	depth->ApplyMask(*mask,0);
//
////	SaveImageToFile(rgb, "frames/color_000000_masked2.pnm");
////	SaveImageToFile(depth, "frames/depth_000000_masked2.pnm");
//
//	ReadImageFromFile(gtMaskedRgb, "frames/color_000000.png");
//	gtMaskedRgb->ApplyMask(*mask, Vector4u((unsigned char) 0));
//	ReadImageFromFile(gtMaskedDepth, "frames/depth_000000_masked.pnm");
//
//	BOOST_REQUIRE(*rgb == *gtMaskedRgb);
//	BOOST_REQUIRE(*depth == *gtMaskedDepth);
//
//	delete rgb;
//	delete depth;
//	//delete mask;
//	delete imageSource;
//	delete gtMaskedDepth;
//	delete gtMaskedRgb;
//}