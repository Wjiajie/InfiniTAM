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
#include "../ITMLib/Trackers/Shared/ITMSceneMotionTracker_Shared.h"
#include "../ITMLib/Objects/Scene/ITMSceneManipulation.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "../ITMLib/Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"

//local
#include "TestUtils.h"
#include "../ITMLib/Utils/ITMIntArrayMap3D.h"
#include "../ITMLib/Utils/ITMSceneStatisticsCalculator.h"
#include "../ORUtils/FileUtils.h"

using namespace ITMLib;


BOOST_AUTO_TEST_CASE( testSetVoxelAndCopyScene )
{
	ITMLibSettings *settings = new ITMLibSettings();
	settings->deviceType = ITMLibSettings::DEVICE_CPU;
	ITMScene<ITMVoxelCanonical, ITMVoxelIndex> scene(&settings->sceneParams,
	                                        settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                        settings->GetMemoryType());

	ITMSceneReconstructionEngine<ITMVoxelCanonical,ITMVoxelIndex> * sceneRecoEngine = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxelCanonical,ITMVoxelIndex>(settings->deviceType);
	sceneRecoEngine->ResetScene(&scene);
	ITMSceneReconstructionEngine<ITMVoxelLive,ITMVoxelIndex> * sceneRecoEngineAux = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxelLive,ITMVoxelIndex>(settings->deviceType);



	ITMVoxelCanonical voxelZero;
	voxelZero.sdf = 0.0f;
	SetVoxel_CPU(scene,Vector3i(0,0,0),voxelZero);
	ITMVoxelCanonical out;
	out = ReadVoxel(scene,Vector3i(0,0,0));
	BOOST_ASSERT(out.sdf == voxelZero.sdf);
	ITMVoxelCanonical voxelHalf;
	voxelHalf.sdf = 0.5f;

	SetVoxel_CPU(scene, Vector3i(1,1,1), voxelHalf);
	out = ReadVoxel(scene, Vector3i(1,1,1));
	BOOST_ASSERT(out.sdf == voxelHalf.sdf);
	SetVoxel_CPU(scene, Vector3i(9,9,9), voxelZero);
	SetVoxel_CPU(scene, Vector3i(9,9,9), voxelHalf);
	out = ReadVoxel(scene, Vector3i(9,9,9));
	BOOST_ASSERT(out.sdf == voxelHalf.sdf);
	Vector3i voxelPos(232,125,62);
	SetVoxel_CPU(scene, voxelPos, voxelZero);
	out = ReadVoxel(scene, voxelPos);
	BOOST_ASSERT(out.sdf == voxelZero.sdf);
	out = ReadVoxel(scene, Vector3i(0,0,0));
	BOOST_ASSERT(out.sdf == voxelZero.sdf);

	Vector3i offset(34,6,-9);
	ITMScene<ITMVoxelCanonical, ITMVoxelIndex> scene2(&settings->sceneParams,
	                                        settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                        settings->GetMemoryType());
	sceneRecoEngine->ResetScene(&scene2);
	CopySceneWithOffset_CPU(scene2,scene,offset);
	out = ReadVoxel(scene2, voxelPos+offset);
	BOOST_ASSERT(out.sdf == voxelZero.sdf);
	out = ReadVoxel(scene2, Vector3i(0,0,0)+offset);
	BOOST_ASSERT(out.sdf == voxelZero.sdf);
	out = ReadVoxel(scene2, Vector3i(9,9,9)+offset);
	BOOST_ASSERT(out.sdf == voxelHalf.sdf);
	out = ReadVoxel(scene2, Vector3i(1,1,1)+offset);
	BOOST_ASSERT(out.sdf == voxelHalf.sdf);

	ITMScene<ITMVoxelLive, ITMVoxelIndex> scene3(&settings->sceneParams,
	                                         settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                         settings->GetMemoryType());
	sceneRecoEngineAux->ResetScene(&scene3);
	//TODO: trans-voxel-type-copying re-implement -Greg (GitHub: Algomorph)
	CopySceneWithOffset_CPU(scene3,scene,offset);
	out = ReadVoxel(scene2, voxelPos+offset);
	BOOST_ASSERT(out.sdf == voxelZero.sdf);
	out = ReadVoxel(scene2, Vector3i(0,0,0)+offset);
	BOOST_ASSERT(out.sdf == voxelZero.sdf);
	out = ReadVoxel(scene2, Vector3i(9,9,9)+offset);
	BOOST_ASSERT(out.sdf == voxelHalf.sdf);
	out = ReadVoxel(scene2, Vector3i(1,1,1)+offset);
	BOOST_ASSERT(out.sdf == voxelHalf.sdf);
	BOOST_ASSERT(out.confidence == voxelHalf.confidence);


	delete settings;
	delete sceneRecoEngine;
}

BOOST_AUTO_TEST_CASE( testITMIntArrayMap3D )
{
	ITMIntArrayMap3D map ("one", "two", "three", "four");
	const int maxElementsOnEachLevel = 3;

	for(int keyLevel3 = 0; keyLevel3 < maxElementsOnEachLevel; keyLevel3++){
		for(int keyLevel2 = 0; keyLevel2 < maxElementsOnEachLevel; keyLevel2++){
			for(int keyLevel1 = 0; keyLevel1 <maxElementsOnEachLevel; keyLevel1++){
				for(int valueLevel0 =0; valueLevel0 < maxElementsOnEachLevel; valueLevel0++){
					map.InsertOrdered(keyLevel3,keyLevel2,keyLevel1,valueLevel0);
				}
			}
		}
	}
	std::cout << __FILE__ << ": " << __LINE__ << ": map to save." << std::endl;
	std::cout << map << std::endl;
	const char* testFilename = "int_array_map_test.dat";
	map.SaveToFile(testFilename);
	ITMIntArrayMap3D map2 ("one", "two", "three", "four");
	map2.LoadFromFile(testFilename);
	BOOST_ASSERT(map == map2);


	ITMIntArrayMap3D map3 ("one", "two", "three", "four");
	map3.InsertOrdered(84651,358,1,5);
	map3.InsertOrdered(84651,358,1,6);
	map3.InsertOrdered(102821,436,1,1);
	map3.InsertOrdered(155667, 495,1,2);
	map3.InsertOrdered(179874, 446,1,28);
	map3.InsertOrdered(179874, 446,1,30);
	map3.SaveToFile(testFilename);
	ITMIntArrayMap3D map4 ("one", "two", "three", "four");
	map4.LoadFromFile(testFilename);
	BOOST_ASSERT(map3 == map4);
}


BOOST_AUTO_TEST_CASE( testLogTestScene){
	GenerateAndLogKillingScene01();

}

BOOST_AUTO_TEST_CASE( testSceneSaveLoadCompact){
	auto settings = new ITMLibSettings();

	auto scene1 = new ITMScene<ITMVoxelCanonical, ITMVoxelIndex>(
			&settings->sceneParams, settings->swappingMode ==
			                        ITMLibSettings::SWAPPINGMODE_ENABLED, settings->GetMemoryType());

	auto scene2 = new ITMScene<ITMVoxelCanonical, ITMVoxelIndex>(
			&settings->sceneParams, settings->swappingMode ==
			                        ITMLibSettings::SWAPPINGMODE_ENABLED, settings->GetMemoryType());


	GenerateTestScene01(*scene1);
	std::string path = "test_";
	scene1->SaveToDirectoryCompact_CPU(path);


	ITMSceneReconstructionEngine<ITMVoxelCanonical, ITMVoxelIndex>* reconstructionEngine =
			ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxelCanonical, ITMVoxelIndex>(
					ITMLibSettings::DEVICE_CPU);

	reconstructionEngine->ResetScene(scene2);

	scene2->LoadFromDirectoryCompact_CPU(path);

	ITMSceneStatisticsCalculator<ITMVoxelCanonical, ITMVoxelIndex> calc;
	std::vector<int> hashes1 = calc.GetFilledHashBlockIds(scene1);
	std::vector<int> hashes2 = calc.GetFilledHashBlockIds(scene2);

	BOOST_ASSERT(hashes1.size() == hashes2.size());
	for (int iHash = 0; iHash < hashes1.size(); iHash++){
		BOOST_ASSERT(hashes1[iHash] == hashes2[iHash]);
	}

	delete settings;
	delete scene1;
	delete scene2;
}


BOOST_AUTO_TEST_CASE( testImageMaskReader){

	ITMUChar4Image* rgb = new ITMUChar4Image(true, false);
	ITMShortImage* depth = new ITMShortImage(true, false);
	ITMUCharImage* mask = new ITMUCharImage(true, false);

	BOOST_ASSERT(ReadImageFromFile(rgb, "frames/color_000000.png"));
	BOOST_ASSERT(ReadImageFromFile(depth, "frames/depth_000000.png"));
	BOOST_ASSERT(ReadImageFromFile(mask, "frames/omask_000000.png"));

	rgb->ApplyMask(*mask,Vector4u((unsigned char)0));
	depth->ApplyMask(*mask,0);
	SaveImageToFile(rgb, "frames/color_000000_masked.pnm");
	SaveImageToFile(depth, "frames/depth_000000_masked.pnm");

	delete rgb;
	delete depth;
	delete mask;
}