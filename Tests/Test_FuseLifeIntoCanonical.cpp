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
#define BOOST_TEST_MODULE FuseLiveIntoCanonical
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//local
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/VolumeFusion/VolumeFusionEngine.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#endif
#include "TestUtils.h"
#include "TestUtilsForSnoopyFrames16And17.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../ITMLib/Engines/VolumeFusion/VolumeFusionEngineFactory.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison.h"

using namespace ITMLib;

typedef VolumeFusionEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> VolumeFusionEngine_CPU_PVA;
typedef VolumeFusionEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> VolumeFusionEngine_CPU_VBH;

BOOST_FIXTURE_TEST_CASE(Test_FuseLifeIntoCanonical_CPU_PVA, Frame16And17Fixture) {
	const int iteration = 4;
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* warped_live_volume;
	loadVolume(&warped_live_volume, "TestData/snoopy_result_fr16-17_warps/data_tikhonov_sobolev_iter_"
	                                + std::to_string(iteration) + "_warped_live_",
	           MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* canonical_volume;
	loadVolume(&canonical_volume, "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_16_",
	           MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());

	VolumeFusionEngine_CPU_PVA volumeFusionEngine;
	volumeFusionEngine.FuseOneTsdfVolumeIntoAnother(canonical_volume, warped_live_volume);
	//canonical_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/fused_canonical_");

	VoxelVolume<TSDFVoxel, PlainVoxelArray>* fused_canonical_volume_gt;
	loadVolume(&fused_canonical_volume_gt, "TestData/snoopy_result_fr16-17_partial_PVA/fused_canonical_",
	           MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());

	float absoluteTolerance = 1e-7;

	BOOST_REQUIRE(contentAlmostEqual_CPU(fused_canonical_volume_gt, canonical_volume, absoluteTolerance));
}

BOOST_FIXTURE_TEST_CASE(Test_FuseLifeIntoCanonical_CPU_VBH, Frame16And17Fixture) {
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* warped_live_volume;
	const int iteration = 4;
	loadVolume(&warped_live_volume, "TestData/snoopy_result_fr16-17_warps/data_tikhonov_sobolev_iter_"
	                                + std::to_string(iteration) + "_warped_live_",
	           MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* canonical_volume;
	loadVolume(&canonical_volume, "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_16_",
	           MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());

	VolumeFusionEngine_CPU_VBH volume_fusion_engine;
	volume_fusion_engine.FuseOneTsdfVolumeIntoAnother(canonical_volume, warped_live_volume);
	//canonical_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/fused_canonical_");

	VoxelVolume<TSDFVoxel, VoxelBlockHash>* fused_canonical_volume_gt;
	loadVolume(&fused_canonical_volume_gt, "TestData/snoopy_result_fr16-17_partial_VBH/fused_canonical_",
	           MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());

	float absoluteTolerance = 1e-7;

	BOOST_REQUIRE(contentAlmostEqual_CPU(fused_canonical_volume_gt, canonical_volume, absoluteTolerance));
}

#ifndef COMPILE_WITHOUT_CUDA
typedef VolumeFusionEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA> VolumeFusionEngine_CUDA_PVA;
typedef VolumeFusionEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> VolumeFusionEngine_CUDA_VBH;
BOOST_FIXTURE_TEST_CASE(Test_FuseLifeIntoCanonical_CUDA_PVA, Frame16And17Fixture) {
	const int iteration = 4;
	ITMVoxelVolume<TSDFVoxel, PlainVoxelArray>* warped_live_volume;
	loadVolume(&warped_live_volume, "TestData/snoopy_result_fr16-17_warps/data_tikhonov_sobolev_iter_"
	                                + std::to_string(iteration) + "_warped_live_",
	           MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());
	ITMVoxelVolume<TSDFVoxel, PlainVoxelArray>* canonical_volume;
	loadVolume(&canonical_volume, "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_16_",
	           MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());

	VolumeFusionEngine_CUDA_PVA volume_fusion_engine;
	volume_fusion_engine.FuseOneTsdfVolumeIntoAnother(canonical_volume, warped_live_volume);
//	canonical_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/fused_canonical_");

	ITMVoxelVolume<TSDFVoxel, PlainVoxelArray>* fused_canonical_volume_gt;
	loadVolume(&fused_canonical_volume_gt, "TestData/snoopy_result_fr16-17_partial_PVA/fused_canonical_",
	           MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());

	float absoluteTolerance = 1e-7;

	BOOST_REQUIRE(contentAlmostEqual_CUDA(fused_canonical_volume_gt, canonical_volume, absoluteTolerance));
}

BOOST_FIXTURE_TEST_CASE(Test_FuseLifeIntoCanonical_CUDA_VBH, Frame16And17Fixture) {
	ITMVoxelVolume<TSDFVoxel, VoxelBlockHash>* warped_live_volume;
	const int iteration = 4;
	loadVolume(&warped_live_volume, "TestData/snoopy_result_fr16-17_warps/data_tikhonov_sobolev_iter_"
	                                + std::to_string(iteration) + "_warped_live_",
	           MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());
	ITMVoxelVolume<TSDFVoxel, VoxelBlockHash>* canonical_volume;
	loadVolume(&canonical_volume, "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_16_",
	           MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());

	VolumeFusionEngine_CUDA_VBH volume_fusion_engine;
	volume_fusion_engine.FuseOneTsdfVolumeIntoAnother(canonical_volume, warped_live_volume);
	//canonical_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/fused_canonical_");

	ITMVoxelVolume<TSDFVoxel, VoxelBlockHash>* fused_canonical_volume_gt;
	loadVolume(&fused_canonical_volume_gt, "TestData/snoopy_result_fr16-17_partial_VBH/fused_canonical_",
	           MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());

	float absoluteTolerance = 1e-7;

	BOOST_REQUIRE(contentAlmostEqual_CUDA(fused_canonical_volume_gt, canonical_volume, absoluteTolerance));
}


template<MemoryDeviceType TMemoryDeviceType>
void Generic_Fusion_PVA_to_VBH_test(int iteration){

	// *** load PVA stuff
	ITMVoxelVolume<TSDFVoxel, PlainVoxelArray>* warped_live_volume_PVA;
	loadVolume(&warped_live_volume_PVA, "TestData/snoopy_result_fr16-17_warps/data_tikhonov_sobolev_iter_"
	                                    + std::to_string(iteration) + "_warped_live_",
	           TMemoryDeviceType, Frame16And17Fixture::InitParams<PlainVoxelArray>());
	ITMVoxelVolume<TSDFVoxel, PlainVoxelArray>* canonical_volume_PVA;
	loadVolume(&canonical_volume_PVA, "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_16_",
	           TMemoryDeviceType, Frame16And17Fixture::InitParams<PlainVoxelArray>());
	
	// *** load VBH stuff
	ITMVoxelVolume<TSDFVoxel, VoxelBlockHash>* warped_live_volume_VBH;
	loadVolume(&warped_live_volume_VBH, "TestData/snoopy_result_fr16-17_warps/data_tikhonov_sobolev_iter_"
	                                    + std::to_string(iteration) + "_warped_live_",
	           TMemoryDeviceType, Frame16And17Fixture::InitParams<VoxelBlockHash>());
	ITMVoxelVolume<TSDFVoxel, VoxelBlockHash>* canonical_volume_VBH;
	loadVolume(&canonical_volume_VBH, "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_16_",
	           TMemoryDeviceType, Frame16And17Fixture::InitParams<VoxelBlockHash>());

	VolumeFusionEngineInterface<TSDFVoxel, WarpVoxel, PlainVoxelArray>* volume_fusion_engine_PVA =
			VolumeFusionEngineFactory::MakeVolumeFusionEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray>(TMemoryDeviceType);
	VolumeFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* volume_fusion_engine_VBH =
			VolumeFusionEngineFactory::MakeVolumeFusionEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash>(TMemoryDeviceType);

	volume_fusion_engine_PVA->FuseOneTsdfVolumeIntoAnother(canonical_volume_PVA, warped_live_volume_PVA);
	volume_fusion_engine_VBH->FuseOneTsdfVolumeIntoAnother(canonical_volume_VBH, warped_live_volume_VBH);
	float absoluteTolerance = 1e-7;
	//BOOST_REQUIRE( allocatedContentAlmostEqual_Verbose(canonical_volume_PVA, canonical_volume_VBH, absoluteTolerance, TMemoryDeviceType));
	BOOST_REQUIRE(contentForFlagsAlmostEqual(canonical_volume_PVA, canonical_volume_VBH, VOXEL_NONTRUNCATED, absoluteTolerance, TMemoryDeviceType));
}

BOOST_AUTO_TEST_CASE(Test_FuseLifeIntoCanonical_PVA_to_VBH_CPU){
	Generic_Fusion_PVA_to_VBH_test<MEMORYDEVICE_CPU>(4);
}
BOOST_AUTO_TEST_CASE(Test_FuseLifeIntoCanonical_PVA_to_VBH_CUDA){
	Generic_Fusion_PVA_to_VBH_test<MEMORYDEVICE_CUDA>(4);
}



#endif