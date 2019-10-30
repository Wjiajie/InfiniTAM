//  ================================================================
//  Created by Gregory Kramida on 10/30/19.
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
#define BOOST_TEST_MODULE SurfaceTrackingAuxiliaryFunctions
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//local
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"

#include "TestUtils.h"
#include "../ITMLib/SceneMotionTrackers/CPU/ITMSceneMotionTracker_CPU.h"

using namespace ITMLib;

BOOST_AUTO_TEST_CASE(Test_ClearOutFlowWarpCPU){
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warps_PVA;
	loadSdfVolume(&warps_PVA, "TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_data_flow_warps_", MEMORYDEVICE_CPU);
	auto motionTracker_PVA_CPU = new ITMSceneMotionTracker_CPU<ITMVoxel, ITMWarp, ITMPlainVoxelArray>();



	delete warps_PVA;
}

#ifndef COMPILE_WITHOUT_CUDA

#endif