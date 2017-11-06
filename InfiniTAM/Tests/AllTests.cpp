//  ================================================================
//  Created by Gregory Kramida on 10/23/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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

using namespace ITMLib;

int i = 1;
BOOST_AUTO_TEST_CASE( testSetVoxelAndCopyScene )
{
	ITMLibSettings *settings = new ITMLibSettings();
	settings->deviceType = ITMLibSettings::DEVICE_CPU;
	ITMScene<ITMVoxel, ITMVoxelIndex> scene(&settings->sceneParams,
	                                        settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                        settings->GetMemoryType());

	ITMSceneReconstructionEngine<ITMVoxel,ITMVoxelIndex> * sceneRecoEngine = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxel,ITMVoxelIndex>(settings->deviceType);
	sceneRecoEngine->ResetScene(&scene);
	ITMSceneReconstructionEngine<ITMVoxelAux,ITMVoxelIndex> * sceneRecoEngineAux = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxelAux,ITMVoxelIndex>(settings->deviceType);



	ITMVoxel voxelZero;
	voxelZero.sdf = 0.0f;
	SetVoxel_CPU(scene,Vector3i(0,0,0),voxelZero);
	ITMVoxel out;
	out = ReadVoxel(scene,Vector3i(0,0,0));
	BOOST_ASSERT(out.sdf == voxelZero.sdf);
	ITMVoxel voxelHalf;
	voxelHalf.sdf = 0.5f;
	voxelHalf.confidence = 12.0f;
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
	ITMScene<ITMVoxel, ITMVoxelIndex> scene2(&settings->sceneParams,
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

	ITMScene<ITMVoxelAux, ITMVoxelIndex> scene3(&settings->sceneParams,
	                                         settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                         settings->GetMemoryType());
	sceneRecoEngineAux->ResetScene(&scene3);
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
