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
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "../ITMLib/Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"

//local
#include "TestUtils.h"

using namespace ITMLib;

int i = 1;
BOOST_AUTO_TEST_CASE( test1 )
{
	ITMLibSettings *settings = new ITMLibSettings();
	settings->deviceType = ITMLibSettings::DEVICE_CUDA;
	ITMScene<ITMVoxel, ITMVoxelIndex> *scene = new ITMScene<ITMVoxel,ITMVoxelIndex>(&settings->sceneParams,
	                                                                                settings->swappingMode ==    ITMLibSettings::SWAPPINGMODE_ENABLED, settings->GetMemoryType());

	ITMSceneReconstructionEngine<ITMVoxel,ITMVoxelIndex> * sceneRecoEngine = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxel,ITMVoxelIndex>(settings->deviceType);
	sceneRecoEngine->ResetScene(scene);

	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	int* excessAllocationList = scene->index.GetExcessAllocationList();
	Vector3i voxel1Position(0,0,0);
	ITMHashEntry* hashTable = scene->index.GetEntries();
	//TODO
	scene->localVBA.lastFreeBlockId = lastFreeExcessListId;
	scene->index.SetLastFreeExcessListId(lastFreeExcessListId);

	delete scene;
	delete settings;
	delete sceneRecoEngine;
}
