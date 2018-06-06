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
#include "../Visualization/ITMScene2DSliceVisualizer.h"
#include "../Visualization/ITMScene1DSliceVisualizer.h"

namespace ITMLib {

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMDynamicFusionLogger {
public:
	ITMDynamicFusionLogger();
	~ITMDynamicFusionLogger() = default;//not intended to be extended, so not virtual

	void
	InitializeRecording(ITMScene <TVoxelCanonical, TIndex>* canonicalScene, ITMScene <TVoxelLive, TIndex>*& liveScene,
		                    std::string outputDirectory, bool hasFocusCoordinates, Vector3i focusCoordinates,
		                    bool saveLiveScene2DSlicesAsImages, bool saveCanonicalScene2DSlicesAsImages, bool recordWarps,
		                    bool recordScene1DSlicesWithUpdates, bool recordScene2DSlicesWithUpdates,
		                    vtkSmartPointer<vtkContextView> vtkView);

	void InitializeWarp2DSliceRecording(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                    ITMScene<TVoxelLive, TIndex>* sourceLiveScene);
	void SaveWarpSlices(int iteration);
	void SaveWarps();
	void FinalizeRecording(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene);
	void RecordStatistics(double totalDataEnergy,
	                      double totalLevelSetEnergy,
	                      double totalKillingEnergy,
	                      double totalSmoothnessEnergy,
	                      double totalEnergy);
	bool IsRecordingWarp2DSlices();
	bool IsRecordingWarps();
	void LogHighlight(int hash, int locId, ITMHighlightIterationInfo info);
private:
	// internal file intput/output
	ITMScene2DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>* scene2DSliceVisualizer;
	ITMScene1DSliceVisualizer* scene1DSliceVisualizer;
	ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>* scene3DLogger;
	ITMScene<TVoxelCanonical, TIndex>* canonicalScene;
	ITMScene<TVoxelLive, TIndex>* liveScene;


	std::ofstream energyStatisticsFile;

	// templates //TODO outsource to ITMScene2DSliceLogger
	cv::Mat blank;
	cv::Mat liveImgTemplate;

	// flags
	bool recordingScene2DSlicesWithUpdates;
	bool recordingScene1DSlicesWithUpdates;
	bool recordingWarps;
	bool hasFocusCoordinates;

	std::string outputDirectory;
	Vector3i focusCoordinates;
	const int focusSliceRadius;//=3;

};

} //namespace InfiniTAM