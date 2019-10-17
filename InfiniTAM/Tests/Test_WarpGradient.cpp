//  ================================================================
//  Created by Gregory Kramida on 10/15/19.
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

//boost
#include <boost/test/unit_test.hpp>

//local
#include "TestUtils.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/SceneFileIO/ITMSceneFileIOEngine.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Engines/Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"
#include "../ITMLib/SceneMotionTrackers/Interface/ITMSceneMotionTracker.h"
#include "../ITMLib/SceneMotionTrackers/CPU/ITMSceneMotionTracker_CPU.h"
#include "../ITMLib/SceneMotionTrackers/CUDA/ITMSceneMotionTracker_CUDA.h"
#include "../ITMLib/SceneMotionTrackers/Shared/ITMCalculateWarpGradientFunctor.h"
#include "../ITMLib/SceneMotionTrackers/Shared/ITMWarpGradientFunctors.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"

using namespace ITMLib;

typedef ITMSceneFileIOEngine<ITMVoxel, ITMPlainVoxelArray> SceneFileIOEngine_PVA;
typedef ITMSceneFileIOEngine<ITMVoxel, ITMVoxelBlockHash> SceneFileIOEngine_VBH;

BOOST_AUTO_TEST_CASE(testDataTerm_CPU) {
	ITMLibSettings* settings = &ITMLibSettings::Instance();
	settings->deviceType = ITMLibSettings::DEVICE_CPU;
	settings->enableKillingTerm = false;
	settings->enableDataTerm = true;
	settings->enableSmoothingTerm = false;
	settings->enableGradientSmoothing = true;
	settings->enableLevelSetTerm = false;



	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> canonical_scene_CPU(&settings->sceneParams,
	                                                              settings->swappingMode ==
	                                                              ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                                 settings->GetMemoryType());
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetScene(&canonical_scene_CPU);
	std::cout << "Scene size: " << canonical_scene_CPU.index.getVolumeSize() << std::endl;
	std::cout << "Scene offset: " << canonical_scene_CPU.index.getVolumeOffset() << std::endl;
	canonical_scene_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_PVA/canonical");
	std::cout << "Orig canonical nontruncated:" << SceneStatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelCount(&canonical_scene_CPU) << std::endl;

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> live_scene_CPU(&settings->sceneParams,
	                                                         settings->swappingMode ==
	                                                         ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                            settings->GetMemoryType());
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetScene(&live_scene_CPU);
	live_scene_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_PVA/live");
	std::cout << "Orig live nontruncated:" << SceneStatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelCount(&live_scene_CPU) << std::endl;

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> canonical_scene_slice_CPU(&settings->sceneParams,
	                                                                 settings->swappingMode ==
	                                                                 ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                                 settings->GetMemoryType());
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetScene(&canonical_scene_slice_CPU);
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> live_scene_slice_CPU(&settings->sceneParams,
	                                                                       settings->swappingMode ==
	                                                                       ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                                       settings->GetMemoryType());
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetScene(&live_scene_slice_CPU);
	Vector6i bounds(0, 0, 0, 256, 256, 256);
	ManipulationEngine_CPU_PVA_Voxel::Inst().CopySceneSlice(&canonical_scene_slice_CPU, &canonical_scene_CPU, bounds);


	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CPU(&settings->sceneParams,
	                                                        settings->swappingMode ==
	                                                        ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                           settings->GetMemoryType());
	ManipulationEngine_CPU_PVA_Warp::Inst().ResetScene(&warp_field_CPU);

	auto motionTracker_PVA_CPU = new ITMSceneMotionTracker_CPU<ITMVoxel, ITMWarp,
			ITMCalculateWarpGradientFunctor<ITMVoxel, ITMWarp, ITMPlainVoxelArray::IndexData, ITMPlainVoxelArray::IndexCache>,
			ITMPlainVoxelArray>();


	std::chrono::duration<double> elapsed;
//	std::cout << "Basic functor gradient computation (PVA, CPU): " << std::endl;
	auto start = std::chrono::high_resolution_clock::now();
//	motionTracker_PVA_CPU->CalculateWarpGradient(&canonical_scene_CPU, &live_scene_CPU, &warp_field_CPU, false);
	auto finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
//	std::cout << "Elapsed time: " << elapsed.count() << " s\n";
//
//	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> canonical_scene_CUDA(canonical_scene_CPU,
//	                                                                  MEMORYDEVICE_CUDA);
//
//	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> live_scene_CUDA(live_scene_CPU,MEMORYDEVICE_CUDA);
//
//	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_CUDA(&settings->sceneParams,
//	                                                        settings->swappingMode ==
//	                                                        ITMLibSettings::SWAPPINGMODE_ENABLED,
//	                                                            MEMORYDEVICE_CUDA);
//	ManipulationEngine_CUDA_PVA_Warp::Inst().ResetScene(&warp_field_CUDA);
//
//	auto motionTracker_PVA_CUDA = new ITMSceneMotionTracker_CUDA<ITMVoxel, ITMWarp,
//		ITMCalculateWarpGradientFunctor<ITMVoxel, ITMWarp, ITMPlainVoxelArray::IndexData, ITMPlainVoxelArray::IndexCache>,
//		        ITMPlainVoxelArray>();
//
//
//	std::cout << "Basic functor gradient computation (PVA, CUDA): " << std::endl;
//	start = std::chrono::high_resolution_clock::now();
//	motionTracker_PVA_CUDA->CalculateWarpGradient(&canonical_scene_CUDA, &live_scene_CUDA, &warp_field_CUDA, false);
//	finish = std::chrono::high_resolution_clock::now();
//	elapsed = finish - start;
//	std::cout << "Elapsed time: " << elapsed.count() << " s\n";
//
//	float tolerance = 1e-5;
//
//	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> warp_field_comparison(warp_field_CUDA,MEMORYDEVICE_CPU);
//	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU, &warp_field_comparison, tolerance));


	//motionTracker_PVA_CPU->CalculateWarpGradient(&canonical_scene2, &live_scene2, &warp_field2, false);

	//std::string path = "TestData/snoopy_result_frame_17_VBH/canonical";
	//scene1.LoadFromDirectory(path);



}