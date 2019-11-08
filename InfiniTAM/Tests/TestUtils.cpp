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
#include "TestUtils.h"
#include "TestUtils.tpp"


#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Utils/FileIO/ITMSceneLogger.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"

using namespace ITMLib;

template void GenerateTestScene_CPU<ITMVoxel, ITMVoxelBlockHash>(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* scene);
template void GenerateTestScene_CPU<ITMVoxel, ITMPlainVoxelArray>(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* scene);
template void GenerateTestScene_CUDA<ITMVoxel, ITMVoxelBlockHash>(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* scene);
template void GenerateTestScene_CUDA<ITMVoxel, ITMPlainVoxelArray>(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* scene);

template void simulateVoxelAlteration<ITMVoxel>(ITMVoxel& voxel, float newSdfValue);
template void simulateRandomVoxelAlteration<ITMVoxel>(ITMVoxel& voxel);
template void simulateRandomVoxelAlteration<ITMWarp>(ITMWarp& voxel);


//have nothing to prep for PVA -- everything gets copied off the disk exactly
template<>
void PrepareVoxelVolumeForLoading(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume, MemoryDeviceType deviceType) {}

template<>
void PrepareVoxelVolumeForLoading(ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* volume, MemoryDeviceType deviceType) {}

//for VBH, the scene has to be reset -- there may be
template<>
void PrepareVoxelVolumeForLoading(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume, MemoryDeviceType deviceType) {
	switch (deviceType) {
		case MEMORYDEVICE_CPU:
			ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash>::Inst().ResetScene(volume);
			break;
		case MEMORYDEVICE_CUDA:
			ITMSceneManipulationEngine_CUDA<ITMVoxel, ITMVoxelBlockHash>::Inst().ResetScene(volume);
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Memory/Device type not supported");
	}
}

template<>
void PrepareVoxelVolumeForLoading(ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* volume, MemoryDeviceType deviceType) {
	switch (deviceType) {
		case MEMORYDEVICE_CPU:
			ITMSceneManipulationEngine_CPU<ITMWarp, ITMVoxelBlockHash>::Inst().ResetScene(volume);
			break;
		case MEMORYDEVICE_CUDA:
			ITMSceneManipulationEngine_CUDA<ITMWarp, ITMVoxelBlockHash>::Inst().ResetScene(volume);
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Memory/Device type not supported");
	}
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
typename ITMPlainVoxelArray::InitializationParameters GetStandard512IndexParameters<ITMPlainVoxelArray>(){
	return {Vector3i(512), Vector3i(-256, -256, 0)};
}

template<>
typename ITMVoxelBlockHash::InitializationParameters GetStandard512IndexParameters<ITMVoxelBlockHash>(){
	return {0x40000, 0x20000};
}


template<>
typename ITMPlainVoxelArray::InitializationParameters GetStandard128IndexParameters<ITMPlainVoxelArray>(){
	return {Vector3i(128), Vector3i(-64, -64, 0)};
}

template<>
typename ITMVoxelBlockHash::InitializationParameters GetStandard128IndexParameters<ITMVoxelBlockHash>(){
	return {0x2000, 0x20000};
}

// FIXME: see TODO in header
//template ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray> loadSdfVolume<ITMVoxel, ITMPlainVoxelArray>(
//                                                 const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                 ITMPlainVoxelArray::InitializationParameters initializationParameters,
//                                                 Configuration::SwappingMode swappingMode);
//template ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> loadSdfVolume<ITMVoxel, ITMVoxelBlockHash>(const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                 ITMVoxelBlockHash::InitializationParameters initializationParameters,
//                                                 Configuration::SwappingMode swappingMode);
//template ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray> loadSdfVolume<ITMWarp, ITMPlainVoxelArray>(const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                          ITMPlainVoxelArray::InitializationParameters initializationParameters,
//                                                          Configuration::SwappingMode swappingMode);
//template ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash> loadSdfVolume<ITMWarp, ITMVoxelBlockHash>(
//                                                         const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                         ITMVoxelBlockHash::InitializationParameters initializationParameters,
//                                                         Configuration::SwappingMode swappingMode);

template void loadSdfVolume<ITMVoxel, ITMPlainVoxelArray>(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>** volume,
                                                          const std::string& path, MemoryDeviceType memoryDeviceType,
                                                          ITMPlainVoxelArray::InitializationParameters initializationParameters,
                                                          Configuration::SwappingMode swappingMode);
template void loadSdfVolume<ITMVoxel, ITMVoxelBlockHash>(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>** volume,
                                                         const std::string& path, MemoryDeviceType memoryDeviceType,
                                                         ITMVoxelBlockHash::InitializationParameters initializationParameters,
                                                         Configuration::SwappingMode swappingMode);
template void loadSdfVolume<ITMWarp, ITMPlainVoxelArray>(ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>** volume,
                                                         const std::string& path, MemoryDeviceType memoryDeviceType,
                                                         ITMPlainVoxelArray::InitializationParameters initializationParameters,
                                                         Configuration::SwappingMode swappingMode);
template void loadSdfVolume<ITMWarp, ITMVoxelBlockHash>(ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>** volume,
                                                        const std::string& path, MemoryDeviceType memoryDeviceType,
                                                        ITMVoxelBlockHash::InitializationParameters initializationParameters,
                                                        Configuration::SwappingMode swappingMode);

template
void buildSdfVolumeFromImage<ITMVoxel, ITMPlainVoxelArray>(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>** volume,
                                                           const std::string& depth_path, const std::string& color_path, 
                                                           const std::string& mask_path, const std::string& calibration_path,
                                                           MemoryDeviceType memoryDevice,
                                                           ITMPlainVoxelArray::InitializationParameters initializationParameters,
                                                           Configuration::SwappingMode swappingMode,
                                                           bool useBilateralFilter);

template
void buildSdfVolumeFromImage<ITMVoxel, ITMVoxelBlockHash>(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>** volume,
                             const std::string& depth_path, const std::string& color_path, const std::string& mask_path,
                             const std::string& calibration_path,
                             MemoryDeviceType memoryDevice,
                             ITMVoxelBlockHash::InitializationParameters initializationParameters,
                             Configuration::SwappingMode swappingMode,
                             bool useBilateralFilter);

