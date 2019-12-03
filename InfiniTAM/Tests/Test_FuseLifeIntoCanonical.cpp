//  ================================================================
//  Created by Gregory Kramida on 11/4/19.
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
#define BOOST_TEST_MODULE FuseLifeIntoCanonical
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//local
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"
#include "../ITMLib/Engines/Reconstruction/CPU/ITMDynamicSceneReconstructionEngine_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Engines/Reconstruction/CUDA/ITMDynamicSceneReconstructionEngine_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#endif

#include "TestUtils.h"

using namespace ITMLib;

typedef ITMDynamicSceneReconstructionEngine_CPU<ITMVoxel, ITMWarp, ITMPlainVoxelArray> RecoEngine_CPU_PVA;
typedef ITMDynamicSceneReconstructionEngine_CPU<ITMVoxel, ITMWarp, ITMVoxelBlockHash> RecoEngine_CPU_VBH;

BOOST_AUTO_TEST_CASE(Test_FuseLifeIntoCanonical_CPU_PVA) {
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* warped_live_volume;
	loadVolume(&warped_live_volume, "TestData/snoopy_result_fr16-17_partial_PVA/warped_live_",
	           MEMORYDEVICE_CPU);
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* canonical_volume;
	loadVolume(&canonical_volume, "TestData/snoopy_result_fr16-17_partial_PVA/canonical",
	           MEMORYDEVICE_CPU);

	RecoEngine_CPU_PVA recoEngine;
	recoEngine.FuseLiveIntoCanonicalSdf(canonical_volume, warped_live_volume);
	//canonical_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/fused_canonical_");

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* fused_canonical_volume_gt;
	loadVolume(&fused_canonical_volume_gt, "TestData/snoopy_result_fr16-17_partial_PVA/fused_canonical_",
	           MEMORYDEVICE_CPU);

	float absoluteTolerance = 1e-7;

	BOOST_REQUIRE(contentAlmostEqual_CPU(fused_canonical_volume_gt, canonical_volume, absoluteTolerance));
}

BOOST_AUTO_TEST_CASE(Test_FuseLifeIntoCanonical_CPU_VBH) {
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* warped_live_volume;
	loadVolume(&warped_live_volume, "TestData/snoopy_result_fr16-17_partial_VBH/warped_live_",
	           MEMORYDEVICE_CPU);
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* canonical_volume;
	loadVolume(&canonical_volume, "TestData/snoopy_result_fr16-17_partial_VBH/canonical",
	           MEMORYDEVICE_CPU);

	RecoEngine_CPU_VBH recoEngine;
	recoEngine.FuseLiveIntoCanonicalSdf(canonical_volume, warped_live_volume);
//	canonical_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/fused_canonical_");

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* fused_canonical_volume_gt;
	loadVolume(&fused_canonical_volume_gt, "TestData/snoopy_result_fr16-17_partial_VBH/fused_canonical_",
	           MEMORYDEVICE_CPU);

	float absoluteTolerance = 1e-7;

	BOOST_REQUIRE(contentAlmostEqual_CPU(fused_canonical_volume_gt, canonical_volume, absoluteTolerance));
}

#ifndef COMPILE_WITHOUT_CUDA
typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray> RecoEngine_CUDA_PVA;
typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash> RecoEngine_CUDA_VBH;
BOOST_AUTO_TEST_CASE(Test_FuseLifeIntoCanonical_CUDA_PVA) {
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* warped_live_volume;
	loadVolume(&warped_live_volume, "TestData/snoopy_result_fr16-17_partial_PVA/warped_live_",
	           MEMORYDEVICE_CUDA);
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* canonical_volume;
	loadVolume(&canonical_volume, "TestData/snoopy_result_fr16-17_partial_PVA/canonical",
	           MEMORYDEVICE_CUDA);

	RecoEngine_CUDA_PVA recoEngine;
	recoEngine.FuseLiveIntoCanonicalSdf(canonical_volume, warped_live_volume);
	//canonical_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/fused_canonical_");

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* fused_canonical_volume_gt;
	loadVolume(&fused_canonical_volume_gt, "TestData/snoopy_result_fr16-17_partial_PVA/fused_canonical_",
	           MEMORYDEVICE_CUDA);

	float absoluteTolerance = 1e-7;

	BOOST_REQUIRE(contentAlmostEqual_CUDA(fused_canonical_volume_gt, canonical_volume, absoluteTolerance));
}

BOOST_AUTO_TEST_CASE(Test_FuseLifeIntoCanonical_CUDA_VBH) {
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* warped_live_volume;
	loadVolume(&warped_live_volume, "TestData/snoopy_result_fr16-17_partial_VBH/warped_live_",
	           MEMORYDEVICE_CUDA);
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* canonical_volume;
	loadVolume(&canonical_volume, "TestData/snoopy_result_fr16-17_partial_VBH/canonical",
	           MEMORYDEVICE_CUDA);

	RecoEngine_CUDA_VBH recoEngine;
	recoEngine.FuseLiveIntoCanonicalSdf(canonical_volume, warped_live_volume);
//	canonical_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/fused_canonical_");

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* fused_canonical_volume_gt;
	loadVolume(&fused_canonical_volume_gt, "TestData/snoopy_result_fr16-17_partial_VBH/fused_canonical_",
	           MEMORYDEVICE_CUDA);

	float absoluteTolerance = 1e-7;

	BOOST_REQUIRE(contentAlmostEqual_CUDA(fused_canonical_volume_gt, canonical_volume, absoluteTolerance));
}

#endif