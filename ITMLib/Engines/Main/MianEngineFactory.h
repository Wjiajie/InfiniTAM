//  ================================================================
//  Created by Gregory Kramida on 12/27/19.
//  Copyright (c)  2019 Gregory Kramida
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

#pragma once

#include "../../../ORUtils/MemoryDeviceType.h"
#include "../../../InputSource/ImageSourceEngine.h"
#include "ITMBasicEngine.h"
#include "ITMMultiEngine.h"
#include "ITMDynamicEngine.h"
#include "ITMBasicSurfelEngine.h"
#include "ITMMainEngine.h"

namespace ITMLib{

ITMMainEngine* BuildMainEngine(const ITMRGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d, bool fix_camera = false){
	auto& settings = configuration::get();
	configuration::IndexingMethod chosenIndexingMethod = settings.indexing_method;
	ITMMainEngine* mainEngine = nullptr;

	switch (settings.library_mode) {
		case configuration::LIBMODE_BASIC:
			switch (chosenIndexingMethod) {
				case configuration::INDEX_HASH:
					mainEngine = new ITMBasicEngine<ITMVoxel, VoxelBlockHash>(calib, imgSize_rgb, imgSize_d);
					break;
				case configuration::INDEX_ARRAY:
					mainEngine = new ITMBasicEngine<ITMVoxel, PlainVoxelArray>(calib, imgSize_rgb, imgSize_d);
					break;
			}
			break;
		case configuration::LIBMODE_BASIC_SURFELS:
			mainEngine = new ITMBasicSurfelEngine<ITMSurfelT>(calib, imgSize_rgb, imgSize_d);
			break;
		case configuration::LIBMODE_LOOPCLOSURE:
			switch (chosenIndexingMethod) {
				case configuration::INDEX_HASH:
					mainEngine = new ITMMultiEngine<ITMVoxel, VoxelBlockHash>(calib, imgSize_rgb, imgSize_d);
					break;
				case configuration::INDEX_ARRAY:
					mainEngine = new ITMMultiEngine<ITMVoxel, PlainVoxelArray>(calib, imgSize_rgb, imgSize_d);
					break;
			}
			break;
		case configuration::LIBMODE_DYNAMIC:
			switch (chosenIndexingMethod) {
				case configuration::INDEX_HASH:
					mainEngine = new ITMDynamicEngine<ITMVoxel, ITMWarp, VoxelBlockHash>(calib, imgSize_rgb, imgSize_d);
					break;
				case configuration::INDEX_ARRAY:
					mainEngine = new ITMDynamicEngine<ITMVoxel, ITMWarp, PlainVoxelArray>(calib, imgSize_rgb, imgSize_d);
					break;
			}
			break;
		default:
			throw std::runtime_error("Unsupported library mode!");
	}

	if (fix_camera) {
		std::cout << "fix_camera flag passed, automatically locking camera if possible "
		             "(attempting to disable tracking)." << std::endl;
		mainEngine->turnOffTracking();
	}
	return mainEngine;
}

} // namespace ITMLib

