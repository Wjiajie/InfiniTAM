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
#include <boost/test/test_tools.hpp>
#include "TestUtils.h"
#include "TestUtils.tpp"


#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/VolumeEditAndCopy/CPU/VolumeEditAndCopyEngine_CPU.h"
#include "../ITMLib/Utils/FileIO/ITMSceneLogger.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
#include "../ITMLib/Engines/ViewBuilding/Interface/ITMViewBuilder.h"
#include "../ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h"

using namespace ITMLib;

template void GenerateTestScene_CPU<ITMVoxel, ITMVoxelBlockHash>(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* scene);
template void GenerateTestScene_CPU<ITMVoxel, ITMPlainVoxelArray>(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* scene);
#ifndef COMPILE_WITHOUT_CUDA
template void GenerateTestScene_CUDA<ITMVoxel, ITMVoxelBlockHash>(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* scene);
template void GenerateTestScene_CUDA<ITMVoxel, ITMPlainVoxelArray>(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* scene);
#endif

template void simulateVoxelAlteration<ITMVoxel>(ITMVoxel& voxel, float newSdfValue);
template void simulateRandomVoxelAlteration<ITMVoxel>(ITMVoxel& voxel);
template void simulateRandomVoxelAlteration<ITMWarp>(ITMWarp& voxel);


//have nothing to prep for PVA -- everything gets copied off the disk exactly
template<>
void PrepareVoxelVolumeForLoading(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume) {}

template<>
void PrepareVoxelVolumeForLoading(ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* volume) {}

//for VBH, the scene has to be reset before loading
template<>
void PrepareVoxelVolumeForLoading(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume) {
	volume->Reset();
}
template<>
void PrepareVoxelVolumeForLoading(ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* volume) {
	volume->Reset();
}

template<>
ITMPlainVoxelArray::InitializationParameters GetFrame17PartialIndexParameters<ITMPlainVoxelArray>() {
	return {Vector3i(80, 96, 144), Vector3i(-64, -24, 168)};
}

template<>
ITMVoxelBlockHash::InitializationParameters GetFrame17PartialIndexParameters<ITMVoxelBlockHash>() {
	return {0x800, 0x20000};
}

template<>
typename ITMPlainVoxelArray::InitializationParameters GetStandard512IndexParameters<ITMPlainVoxelArray>() {
	return {Vector3i(512), Vector3i(-256, -256, 0)};
}

template<>
typename ITMVoxelBlockHash::InitializationParameters GetStandard512IndexParameters<ITMVoxelBlockHash>() {
	return {0x40000, 0x20000};
}


template<>
typename ITMPlainVoxelArray::InitializationParameters GetStandard128IndexParameters<ITMPlainVoxelArray>() {
	return {Vector3i(128), Vector3i(-64, -64, 0)};
}

template<>
typename ITMVoxelBlockHash::InitializationParameters GetStandard128IndexParameters<ITMVoxelBlockHash>() {
	return {0x2000, 0x20000};
}

// FIXME: see TODO in header
//template ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> loadVolume<ITMVoxel, ITMPlainVoxelArray>(
//                                                 const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                 ITMPlainVoxelArray::InitializationParameters initializationParameters,
//                                                 Configuration::SwappingMode swapping_mode);
//template ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> loadVolume<ITMVoxel, ITMVoxelBlockHash>(const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                 ITMVoxelBlockHash::InitializationParameters initializationParameters,
//                                                 Configuration::SwappingMode swapping_mode);
//template ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> loadVolume<ITMWarp, ITMPlainVoxelArray>(const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                          ITMPlainVoxelArray::InitializationParameters initializationParameters,
//                                                          Configuration::SwappingMode swapping_mode);
//template ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> loadVolume<ITMWarp, ITMVoxelBlockHash>(
//                                                         const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                         ITMVoxelBlockHash::InitializationParameters initializationParameters,
//                                                         Configuration::SwappingMode swapping_mode);

template void loadVolume<ITMVoxel, ITMPlainVoxelArray>(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>** volume,
                                                       const std::string& path, MemoryDeviceType memoryDeviceType,
                                                       ITMPlainVoxelArray::InitializationParameters initializationParameters,
                                                       Configuration::SwappingMode swappingMode);
template void loadVolume<ITMVoxel, ITMVoxelBlockHash>(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>** volume,
                                                      const std::string& path, MemoryDeviceType memoryDeviceType,
                                                      ITMVoxelBlockHash::InitializationParameters initializationParameters,
                                                      Configuration::SwappingMode swappingMode);
template void loadVolume<ITMWarp, ITMPlainVoxelArray>(ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>** volume,
                                                      const std::string& path, MemoryDeviceType memoryDeviceType,
                                                      ITMPlainVoxelArray::InitializationParameters initializationParameters,
                                                      Configuration::SwappingMode swappingMode);
template void loadVolume<ITMWarp, ITMVoxelBlockHash>(ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>** volume,
                                                     const std::string& path, MemoryDeviceType memoryDeviceType,
                                                     ITMVoxelBlockHash::InitializationParameters initializationParameters,
                                                     Configuration::SwappingMode swappingMode);

void
updateView(ITMView** view, const std::string& depth_path, const std::string& color_path, const std::string& mask_path,
           const std::string& calibration_path, MemoryDeviceType memoryDevice) {
	static ITMViewBuilder* viewBuilder_CPU = nullptr;
	static ITMViewBuilder* viewBuilder_CUDA = nullptr;
	ITMViewBuilder* viewBuilderToUse;
	switch (memoryDevice) {
		case MEMORYDEVICE_CPU:
			if (viewBuilder_CPU == nullptr)
				viewBuilder_CPU = ITMViewBuilderFactory::MakeViewBuilder(calibration_path, memoryDevice);
			viewBuilderToUse = viewBuilder_CPU;
			break;
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			if (viewBuilder_CUDA == nullptr)
				viewBuilder_CUDA = ITMViewBuilderFactory::MakeViewBuilder(calibration_path, memoryDevice);
			viewBuilderToUse = viewBuilder_CUDA;
#else
			DIEWITHEXCEPTION_REPORTLOCATION("Attmpted to update CUDA view while build without CUDA support, aborting.");
#endif
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("unsupported memory device type!");
	}

	auto* rgb = new ITMUChar4Image(true, false);
	auto* depth = new ITMShortImage(true, false);
	auto* mask = new ITMUCharImage(true, false);
	BOOST_REQUIRE(ReadImageFromFile(rgb, color_path.c_str()));
	BOOST_REQUIRE(ReadImageFromFile(depth, depth_path.c_str()));
	BOOST_REQUIRE(ReadImageFromFile(mask, mask_path.c_str()));
	rgb->ApplyMask(*mask, Vector4u((unsigned char) 0));
	depth->ApplyMask(*mask, 0);
	viewBuilderToUse->UpdateView(view, rgb, depth, false, false, false, true);
	delete rgb;
	delete depth;
	delete mask;
}

template
void buildSdfVolumeFromImage<ITMVoxel, ITMPlainVoxelArray>(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>** volume,
                                                           ITMView** view,
                                                           const std::string& depth_path, const std::string& color_path,
                                                           const std::string& mask_path,
                                                           const std::string& calibration_path,
                                                           MemoryDeviceType memoryDevice,
                                                           ITMPlainVoxelArray::InitializationParameters initializationParameters,
                                                           Configuration::SwappingMode swappingMode,
                                                           bool useBilateralFilter);

template
void buildSdfVolumeFromImage<ITMVoxel, ITMPlainVoxelArray>(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>** volume,
                                                           const std::string& depth_path, const std::string& color_path,
                                                           const std::string& mask_path,
                                                           const std::string& calibration_path,
                                                           MemoryDeviceType memoryDevice,
                                                           ITMPlainVoxelArray::InitializationParameters initializationParameters,
                                                           Configuration::SwappingMode swappingMode,
                                                           bool useBilateralFilter);

template
void buildSdfVolumeFromImage<ITMVoxel, ITMVoxelBlockHash>(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>** volume,
                                                          ITMView** view,
                                                          const std::string& depth_path, const std::string& color_path,
                                                          const std::string& mask_path,
                                                          const std::string& calibration_path,
                                                          MemoryDeviceType memoryDevice,
                                                          ITMVoxelBlockHash::InitializationParameters initializationParameters,
                                                          Configuration::SwappingMode swappingMode,
                                                          bool useBilateralFilter);

template
void buildSdfVolumeFromImage<ITMVoxel, ITMVoxelBlockHash>(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>** volume,
                                                          const std::string& depth_path, const std::string& color_path,
                                                          const std::string& mask_path,
                                                          const std::string& calibration_path,
                                                          MemoryDeviceType memoryDevice,
                                                          ITMVoxelBlockHash::InitializationParameters initializationParameters,
                                                          Configuration::SwappingMode swappingMode,
                                                          bool useBilateralFilter);

template
void initializeVolume<ITMVoxel, ITMVoxelBlockHash>(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>** volume,
                                                   ITMVoxelBlockHash::InitializationParameters initializationParameters,
                                                   MemoryDeviceType memoryDevice,
                                                   Configuration::SwappingMode swappingMode);
template
void initializeVolume<ITMVoxel, ITMPlainVoxelArray>(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>** volume,
                                                    ITMPlainVoxelArray::InitializationParameters initializationParameters,
                                                    MemoryDeviceType memoryDevice,
                                                    Configuration::SwappingMode swappingMode);
template
void initializeVolume<ITMWarp, ITMVoxelBlockHash>(ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>** volume,
                                                  ITMVoxelBlockHash::InitializationParameters initializationParameters,
                                                  MemoryDeviceType memoryDevice,
                                                  Configuration::SwappingMode swappingMode);
template
void initializeVolume<ITMWarp, ITMPlainVoxelArray>(ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>** volume,
                                                   ITMPlainVoxelArray::InitializationParameters initializationParameters,
                                                   MemoryDeviceType memoryDevice,
                                                   Configuration::SwappingMode swappingMode);
