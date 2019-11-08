//  ================================================================
//  Created by Gregory Kramida on 11/7/19.
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

#define BOOST_TEST_MODULE WarpScene
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//local
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"
#include "../ITMLib/Engines/Reconstruction/CUDA/ITMDynamicSceneReconstructionEngine_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"

#include "TestUtils.h"

using namespace ITMLib;

typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray> RecoEngine_CUDA_PVA;
typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash> RecoEngine_CUDA_VBH;


BOOST_AUTO_TEST_CASE(Test_WarpScene_PVA_VBH) {
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume_PVA;
	buildSdfVolumeFromImage(&volume_PVA, "TestData/snoopy_depth_000016.png",
			"TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
			"TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA);

	std::cout << SceneStatCalc_CUDA_PVA_Voxel::Instance().ComputeAlteredVoxelCount(volume_PVA) << std::endl;

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume_VBH;
	buildSdfVolumeFromImage(&volume_VBH, "TestData/snoopy_depth_000016.png",
	                        "TestData/snoopy_color_000016.png", "TestData/snoopy_omask_000016.png",
	                        "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA);

	std::cout << SceneStatCalc_CUDA_VBH_Voxel::Instance().ComputeAlteredVoxelCount(volume_VBH) << std::endl;

	delete volume_PVA;
}