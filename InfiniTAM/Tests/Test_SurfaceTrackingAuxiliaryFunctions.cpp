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
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"
#endif

using namespace ITMLib;

BOOST_AUTO_TEST_CASE(Test_ClearOutFlowWarp_CPU_PVA){
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warps_PVA;
	loadSdfVolume(&warps_PVA, "TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_data_flow_warps_", MEMORYDEVICE_CPU);
	float relativeTolerance = 0.1f;//percent
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_PVA_Warp::Instance().ComputeFlowWarpMax(warps_PVA), 0.0870865062f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_PVA_Warp::Instance().ComputeFlowWarpMin(warps_PVA), 0.0f, relativeTolerance);

	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>();

	motionTracker_PVA_CPU->ClearOutFlowWarp(warps_PVA);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_PVA_Warp::Instance().ComputeFlowWarpMax(warps_PVA), 0.0f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_PVA_Warp::Instance().ComputeFlowWarpMin(warps_PVA), 0.0f, relativeTolerance);

	delete warps_PVA;
}

BOOST_AUTO_TEST_CASE(Test_ClearOutFlowWarp_CPU_VBH){
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warps_VBH;
	loadSdfVolume(&warps_VBH, "TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_data_flow_warps_", MEMORYDEVICE_CPU);
	float relativeTolerance = 0.1f;//percent
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeFlowWarpMax(warps_VBH), 0.021316606551f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeFlowWarpMin(warps_VBH), 0.0f, relativeTolerance);

	auto motionTracker_VBH_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU>();

	motionTracker_VBH_CPU->ClearOutFlowWarp(warps_VBH);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeFlowWarpMax(warps_VBH), 0.0f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeFlowWarpMin(warps_VBH), 0.0f, relativeTolerance);

	delete warps_VBH;
}

#ifndef COMPILE_WITHOUT_CUDA

BOOST_AUTO_TEST_CASE(Test_ClearOutFlowWarp_CUDA_PVA){
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warps_PVA;
	loadSdfVolume(&warps_PVA, "TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_data_flow_warps_", MEMORYDEVICE_CUDA);
	float relativeTolerance = 0.1f;//percent
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_PVA_Warp::Instance().ComputeFlowWarpMax(warps_PVA), 0.0870865062f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_PVA_Warp::Instance().ComputeFlowWarpMin(warps_PVA), 0.0f, relativeTolerance);

	auto motionTracker_PVA_CUDA = new SurfaceTracker<ITMVoxel, ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CUDA>();

	motionTracker_PVA_CUDA->ClearOutFlowWarp(warps_PVA);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_PVA_Warp::Instance().ComputeFlowWarpMax(warps_PVA), 0.0f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_PVA_Warp::Instance().ComputeFlowWarpMin(warps_PVA), 0.0f, relativeTolerance);

	delete warps_PVA;
}

BOOST_AUTO_TEST_CASE(Test_ClearOutFlowWarp_CUDA_VBH){
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warps_VBH;
	loadSdfVolume(&warps_VBH, "TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_data_flow_warps_", MEMORYDEVICE_CUDA);
	float relativeTolerance = 0.1f;//percent
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeFlowWarpMax(warps_VBH), 0.021316606551f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeFlowWarpMin(warps_VBH), 0.0f, relativeTolerance);

	auto motionTracker_VBH_CUDA = new SurfaceTracker<ITMVoxel, ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CUDA>();

	motionTracker_VBH_CUDA->ClearOutFlowWarp(warps_VBH);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeFlowWarpMax(warps_VBH), 0.0f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeFlowWarpMin(warps_VBH), 0.0f, relativeTolerance);

	delete warps_VBH;
}


#endif