//  ================================================================
//  Created by Gregory Kramida on 10/17/19.
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


//stdlib
#include <random>
#include <vector>
#include <chrono>
#include <atomic>

//boost
#include <boost/test/unit_test.hpp>

//local
#include "TestUtils.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/SceneFileIO/ITMSceneFileIOEngine.h"
#include "../ITMLib/Engines/Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"
#include "../ITMLib/SceneMotionTrackers/Interface/ITMSceneMotionTracker.h"
#include "../ITMLib/SceneMotionTrackers/CUDA/ITMSceneMotionTracker_CUDA.h"
#include "../ITMLib/SceneMotionTrackers/Shared/ITMCalculateWarpGradientFunctor.h"
#include "../ITMLib/SceneMotionTrackers/Shared/ITMLegacyCalculateWarpGradientFunctor.h"
//#include "../ITMLib/SceneMotionTrackers/Shared/ITMWarpGradientTestCUDAFunctors.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"
#include "../ITMLib/Engines/Traversal/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h"


using namespace ITMLib;

typedef ITMSceneFileIOEngine<ITMVoxel, ITMPlainVoxelArray> SceneFileIOEngine_PVA;
typedef ITMSceneFileIOEngine<ITMVoxel, ITMVoxelBlockHash> SceneFileIOEngine_VBH;

template <typename TVoxel>
struct AlteredGradientCountFunctor{
	AlteredGradientCountFunctor() : count(0){};
	void operator()(const TVoxel& voxel) {
		if(voxel.gradient0 != Vector3f(0.0f)){
			count.fetch_add(1u);
		}
	}

	std::atomic<unsigned int> count;
};

BOOST_AUTO_TEST_CASE(testDataTerm_CUDA) {
	ITMLibSettings* settings = &ITMLibSettings::Instance();
	settings->deviceType = ITMLibSettings::DEVICE_CUDA;
	settings->enableKillingTerm = false;
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = false;
	settings->enableGradientSmoothing = true;
	settings->enableLevelSetTerm = false;


	Vector3i offsetSlice(-64, -24, 168);
	//64+16=80; -24+72=96; 312-168=300-156=304-160=144
	Vector3i sizeSlice(80, 96, 144);

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> canonical_scene_CUDA(&settings->sceneParams,
	                                                                 settings->swappingMode ==
	                                                                 ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                                 settings->GetMemoryType(),
	                                                                 sizeSlice, offsetSlice);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetScene(&canonical_scene_CUDA);

	canonical_scene_CUDA.LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/canonical");


	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> live_scene_CUDA(&settings->sceneParams,
	                                                            settings->swappingMode ==
	                                                            ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                            settings->GetMemoryType(),
	                                                            sizeSlice, offsetSlice);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetScene(&live_scene_CUDA);
	live_scene_CUDA.LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/live");


	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CUDA1(&settings->sceneParams,
	                                                            settings->swappingMode ==
	                                                            ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                            settings->GetMemoryType(),
	                                                            sizeSlice, offsetSlice);
	ManipulationEngine_CUDA_PVA_Warp::Inst().ResetScene(&warp_field_CUDA1);

	warp_field_CUDA1.LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/gradient0");


	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CUDA2(&settings->sceneParams,
	                                                            settings->swappingMode ==
	                                                            ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                            settings->GetMemoryType(),
	                                                            sizeSlice, offsetSlice);
	ManipulationEngine_CUDA_PVA_Warp::Inst().ResetScene(&warp_field_CUDA2);

	auto motionTracker_PVA_CUDA = new ITMSceneMotionTracker_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray>();


	TimeIt([&](){
		motionTracker_PVA_CUDA->CalculateWarpGradient(&canonical_scene_CUDA, &live_scene_CUDA, &warp_field_CUDA2, false);
	}, "Calculate Warp Gradient - Basic");


	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CPU(warp_field_CUDA2, MEMORYDEVICE_CPU);

	AlteredGradientCountFunctor<ITMWarp> functor;
	functor.count.store(0u);
	ITMSceneTraversalEngine<ITMWarp, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU, functor);
	std::cout << "Count altered warps: " << functor.count.load() << std::endl;
	BOOST_REQUIRE_EQUAL(functor.count.load(), 36627u);

	float tolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, &warp_field_CUDA2, tolerance));


}