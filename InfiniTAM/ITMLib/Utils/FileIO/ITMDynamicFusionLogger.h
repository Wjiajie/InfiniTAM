//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "../../Objects/Scene/ITMScene.h"
#include "ITMSceneLogger.h"
#include "ITMScene2DSliceLogger.h"

namespace ITMLib{
template<typename TVoxelLive, typename TVoxelCanonical, typename TIndex>
class ITMDynamicFusionLogger {
public:
	ITMDynamicFusionLogger();
	~ITMDynamicFusionLogger() = default;//not intended to be extended, so not virtual

	void InitializeRecording(
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene,
			bool saveLiveScene2DSlicesAsImages, bool saveCanonicalScene2DSlicesAsImages, bool recordWarp2DSlices,
			bool hasFocusCoordinates, Vector3i focusCoordinates, bool recordWarps, std::string outputDirectory);

	void InitializeWarp2DSliceRecording(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                    ITMScene<TVoxelLive, TIndex>* sourceLiveScene);
	void SaveWarp2DSlice(int iteration);
	void SaveWarps();
	void FinalizeRecording(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene);
private:
	// internal file intput/output
	ITMScene2DSliceLogger<TVoxelCanonical, TVoxelLive, TIndex>* rasterizer;
	ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>* sceneLogger;
	ITMScene <TVoxelCanonical, TIndex>* canonicalScene;
	ITMScene <TVoxelLive, TIndex>* liveScene;

	std::ofstream energy_stat_file;

	// templates //TODO outsource to ITMScene2DSliceLogger
	cv::Mat blank;
	cv::Mat liveImgTemplate;

	// flags
	bool recordWarp2DSlices;// = false; // CLI flag made in InfiniTAM_bpo
	bool recordWarps;
	bool hasFocusCoordinates;

	std::string outputDirectory;
	Vector3i focusCoordinates;
	const int focusSliceRadius;//=3;

};

} //namespace InfiniTAM


