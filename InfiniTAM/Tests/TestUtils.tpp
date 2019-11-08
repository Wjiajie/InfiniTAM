//  ================================================================
//  Created by Gregory Kramida on 11/3/17.
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
#include <random>
#include <boost/test/test_tools.hpp>

#include "TestUtils.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Engines/Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"
#include "../ITMLib/Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"
#include "../ITMLib/Engines/Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"
#include "../ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../ITMLib/Engines/Reconstruction/Interface/ITMDynamicSceneReconstructionEngine.h"
#include "../ORUtils/FileUtils.h"
#include "../ITMLib/Objects/RenderStates/ITMRenderStateFactory.h"


using namespace ITMLib;

template<class TVoxel, class TIndex>
void GenerateTestScene_CPU(ITMVoxelVolume<TVoxel, TIndex>* scene) {
	ITMSceneManipulationEngine_CPU<TVoxel, TIndex>::Inst().ResetScene(scene);
	const int narrowBandThicknessVoxels = 10;
	int xOffset = 8;
	int surfaceSizeVoxelsZ = 16;
	int surfaceSizeVoxelsY = 64;

	for (int iVoxelAcrossBand = 0; iVoxelAcrossBand < narrowBandThicknessVoxels + 1; iVoxelAcrossBand++) {
		float sdfMagnitude = 0.0f + static_cast<float>(iVoxelAcrossBand) * (1.0f / narrowBandThicknessVoxels);
		int xPos = xOffset + iVoxelAcrossBand;
		int xNeg = xOffset - iVoxelAcrossBand;
		TVoxel voxelPos, voxelNeg;
		simulateVoxelAlteration(voxelNeg, sdfMagnitude);
		simulateVoxelAlteration(voxelPos, -sdfMagnitude);

		for (int z = 0; z < surfaceSizeVoxelsZ; z++) {
			for (int y = 0; y < surfaceSizeVoxelsY; y++) {
				ITMSceneManipulationEngine_CPU<TVoxel, TIndex>::Inst().SetVoxel(scene, Vector3i(xPos, y, z), voxelPos);
				ITMSceneManipulationEngine_CPU<TVoxel, TIndex>::Inst().SetVoxel(scene, Vector3i(xNeg, y, z), voxelNeg);
			}
		}
	}

}

template<class TVoxel, class TIndex>
void GenerateTestScene_CUDA(ITMVoxelVolume<TVoxel, TIndex>* scene) {
	ITMSceneManipulationEngine_CUDA<TVoxel, TIndex>::Inst().ResetScene(scene);
	const int narrowBandThicknessVoxels = 10;
	int xOffset = 8;
	int surfaceSizeVoxelsZ = 16;
	int surfaceSizeVoxelsY = 64;

	for (int iVoxelAcrossBand = 0; iVoxelAcrossBand < narrowBandThicknessVoxels + 1; iVoxelAcrossBand++) {
		float sdfMagnitude = 0.0f + static_cast<float>(iVoxelAcrossBand) * (1.0f / narrowBandThicknessVoxels);
		int xPos = xOffset + iVoxelAcrossBand;
		int xNeg = xOffset - iVoxelAcrossBand;
		TVoxel voxelPos, voxelNeg;
		simulateVoxelAlteration(voxelNeg, sdfMagnitude);
		simulateVoxelAlteration(voxelPos, -sdfMagnitude);

		for (int z = 0; z < surfaceSizeVoxelsZ; z++) {
			for (int y = 0; y < surfaceSizeVoxelsY; y++) {
				ITMSceneManipulationEngine_CUDA<TVoxel, TIndex>::Inst().SetVoxel(scene, Vector3i(xPos, y, z), voxelPos);
				ITMSceneManipulationEngine_CUDA<TVoxel, TIndex>::Inst().SetVoxel(scene, Vector3i(xNeg, y, z), voxelNeg);
			}
		}
	}

}


template<bool hasSemanticInformation, typename TVoxel>
struct HandleSDFBasedFlagsAlterationFunctor;

template<typename TVoxel>
struct HandleSDFBasedFlagsAlterationFunctor<true, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline static
	void run(TVoxel& voxel) {
		if (voxel.sdf > -1.0f && voxel.sdf < 1.0f) {
			voxel.flags = VoxelFlags::VOXEL_NONTRUNCATED;
		} else {
			voxel.flags = VoxelFlags::VOXEL_TRUNCATED;
		}
	}
};

template<typename TVoxel>
struct HandleSDFBasedFlagsAlterationFunctor<false, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline static void run(TVoxel& voxel) {}
};

//judiciously ignore clang warnings about something from stdlib throwing uncatchable exceptions in default constructors
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
namespace internal {
std::random_device random_device;
std::mt19937 generator(random_device());
std::uniform_real_distribution<float> floatDistribution(-1.0f, 1.0f);
}
#pragma clang diagnostic pop

template<bool hasSDFInformation, typename TVoxel>
struct HandleSDFAlterationFunctor;

template<typename TVoxel>
struct HandleSDFAlterationFunctor<true, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline static void setValue(TVoxel& voxel, float value) {
		voxel.sdf = TVoxel::floatToValue(value);
		voxel.w_depth += 1;
		HandleSDFBasedFlagsAlterationFunctor<TVoxel::hasSemanticInformation, TVoxel>::run(voxel);
	}

	_CPU_AND_GPU_CODE_
	inline static void setRandom(TVoxel& voxel) {
		float value = internal::floatDistribution(internal::generator);
	}
};

template<typename TVoxel>
struct HandleSDFAlterationFunctor<false, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline static void setValue(TVoxel& voxel, float value) {}

	_CPU_AND_GPU_CODE_
	inline static void setRandom(TVoxel& voxel) {}
};

template<bool hasFlowWarp, typename TVoxel>
struct HandleFlowWarpAlterationFunctor;

template<typename TVoxel>
struct HandleFlowWarpAlterationFunctor<true, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline static void setValue(TVoxel& voxel, Vector3f value) {
		voxel.flow_warp = value;
	}

	_CPU_AND_GPU_CODE_
	inline static void setRandom(TVoxel& voxel) {
		Vector3f value(internal::floatDistribution(internal::generator),
		               internal::floatDistribution(internal::generator),
		               internal::floatDistribution(internal::generator));
	}
};

template<typename TVoxel>
struct HandleFlowWarpAlterationFunctor<false, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline static void setValue(TVoxel& voxel, float value) {}

	_CPU_AND_GPU_CODE_
	inline static void setRandom(TVoxel& voxel) {}
};


template<typename TVoxel>
void simulateVoxelAlteration(TVoxel& voxel, float newSdfValue) {
	HandleSDFAlterationFunctor<TVoxel::hasSDFInformation, TVoxel>::setValue(voxel, newSdfValue);
}

template<typename TVoxel>
void simulateRandomVoxelAlteration(TVoxel& voxel) {
	HandleSDFAlterationFunctor<TVoxel::hasSDFInformation, TVoxel>::setRandom(voxel);
	HandleFlowWarpAlterationFunctor<TVoxel::hasFlowWarp, TVoxel>::setRandom(voxel);
}

// FIXME: see TODO in header
//template<typename TVoxelA, typename TIndex>
//ITMVoxelVolume<TVoxelA, TIndex> loadSdfVolume (const std::string& path, MemoryDeviceType memoryDeviceType,
//                    typename TIndex::InitializationParameters initializationParameters, ITMLibSettings::SwappingMode swappingMode){
//	ITMLibSettings& settings = ITMLibSettings::Instance();
//	ITMVoxelVolume<TVoxelA, TIndex> scene(&settings.sceneParams,
//	                                              swappingMode,
//	                                              memoryDeviceType,initializationParameters);
//	PrepareVoxelVolumeForLoading(&scene, memoryDeviceType);
//	scene.LoadFromDirectory(path);
//	return scene;
//};

template<typename TVoxel, typename TIndex>
void loadSdfVolume(ITMVoxelVolume<TVoxel, TIndex>** volume, const std::string& path, MemoryDeviceType memoryDeviceType,
                   typename TIndex::InitializationParameters initializationParameters,
                   ITMLibSettings::SwappingMode swappingMode) {
	ITMLibSettings& settings = ITMLibSettings::Instance();
	(*volume) = new ITMVoxelVolume<TVoxel, TIndex>(&settings.sceneParams,
	                                               swappingMode,
	                                               memoryDeviceType, initializationParameters);
	PrepareVoxelVolumeForLoading(*volume, memoryDeviceType);
	(*volume)->LoadFromDirectory(path);
}

template<typename TVoxel, typename TIndex>
void buildSdfVolumeFromImage(ITMVoxelVolume<TVoxel, TIndex>** volume,
                             const std::string& depth_path, const std::string& color_path, const std::string& mask_path,
                             const std::string& calibration_path,
                             MemoryDeviceType memoryDevice,
                             typename TIndex::InitializationParameters initializationParameters,
                             ITMLibSettings::SwappingMode swappingMode,
                             bool useBilateralFilter) {
	ITMLibSettings* settings = &ITMLibSettings::Instance();

	// region ================================= CONSTRUCT VIEW =========================================================

	settings->deviceType = memoryDevice;
	settings->useBilateralFilter = useBilateralFilter;
	settings->useThresholdFilter = false;
	settings->sceneParams.mu = 0.04; // non-truncated narrow-band half-width, in meters
	settings->sceneParams.voxelSize = 0.004; // m

	ITMRGBDCalib calibrationData;
	readRGBDCalib(calibration_path.c_str(), calibrationData);

	ITMViewBuilder* viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calibrationData, settings->deviceType);
	Vector2i imageSize(640, 480);
	ITMView* view = nullptr;

	auto* rgb = new ITMUChar4Image(true, false);
	auto* depth = new ITMShortImage(true, false);
	auto* mask = new ITMUCharImage(true, false);
	BOOST_REQUIRE(ReadImageFromFile(rgb, color_path.c_str()));
	BOOST_REQUIRE(ReadImageFromFile(depth, depth_path.c_str()));
	BOOST_REQUIRE(ReadImageFromFile(mask, mask_path.c_str()));
	rgb->ApplyMask(*mask, Vector4u((unsigned char) 0));
	depth->ApplyMask(*mask, 0);

	viewBuilder->UpdateView(&view, rgb, depth, settings->useThresholdFilter,
	                        settings->useBilateralFilter, false, true);

	(*volume) = new ITMVoxelVolume<TVoxel, TIndex>(&settings->sceneParams, swappingMode,
	                                               memoryDevice, initializationParameters);
	switch (memoryDevice) {
		case MEMORYDEVICE_CUDA:
			ITMSceneManipulationEngine_CUDA<TVoxel, TIndex>::Inst().ResetScene(*volume);
			break;
		case MEMORYDEVICE_CPU:
			ITMSceneManipulationEngine_CPU<TVoxel, TIndex>::Inst().ResetScene(*volume);
			break;
	}
	ITMRenderState* renderState = ITMRenderStateFactory<TIndex>::CreateRenderState(imageSize,
	                                                                               &settings->sceneParams,
	                                                                               settings->GetMemoryType(),
	                                                                               (*volume)->index);
	ITMTrackingState trackingState(imageSize, settings->GetMemoryType());

	ITMDynamicSceneReconstructionEngine<TVoxel, ITMWarp, TIndex>* reconstructionEngine_PVA =
			ITMDynamicSceneReconstructionEngineFactory
			::MakeSceneReconstructionEngine<TVoxel, ITMWarp, TIndex>(settings->deviceType);

	reconstructionEngine_PVA->GenerateRawLiveSceneFromView(*volume, view, &trackingState, renderState);

	delete reconstructionEngine_PVA;
	delete rgb;
	delete depth;
	delete mask;
}

