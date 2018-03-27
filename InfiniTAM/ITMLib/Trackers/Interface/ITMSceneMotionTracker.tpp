//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
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
//stdlib
#include <limits>
#include <iomanip>

#ifdef _DEBUG
//_DEBUG (opencv)
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>

#endif

//local
#include "ITMSceneMotionTracker.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"
#include "../../Utils/FileIO/ITMCombinedSceneLogger.h"
#include "../../Utils/ITMPrintHelpers.h"


using namespace ITMLib;


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneMotionTracker(const ITMSceneParams& params, std::string scenePath)
		:
		maxVectorUpdateThresholdVoxels(maxVectorUpdateThresholdMeters / params.voxelSize),
		sceneLogger(""),
		baseOutputDirectory(scenePath){}

/**
 * \brief Tracks motion of voxels from canonical frame to live frame.
 * \details The warp field representing motion of voxels in the canonical frame is updated such that the live frame maps
 * as closely as possible back to the canonical using the warp.
 * \tparam TVoxelCanonical type of canonical voxels
 * \tparam TVoxelLive type of live voxels
 * \tparam TIndex type of voxel index used
 * \param canonicalScene the canonical voxel grid
 * \param liveScene the live voxel grid (typcially obtained by integrating a single depth image into an empty TSDF grid)
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::TrackMotion(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene,
		bool recordWarpUpdates) {

	float maxVectorUpdate = std::numeric_limits<float>::infinity();
	AllocateNewCanonicalHashBlocks(canonicalScene, liveScene);
	//START _DEBUG
#ifdef RASTERIZE_CANONICAL_SCENE
	ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::RenderCanonicalSceneSlices(
			canonicalScene, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::AXIS_X, "_X");
	ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::RenderCanonicalSceneSlices(
			canonicalScene, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::AXIS_Y, "_Y");
	ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::RenderCanonicalSceneSlices(
			canonicalScene, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::AXIS_Z, "_Z");
#endif
#ifdef RASTERIZE_LIVE_SCENE
	ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::RenderLiveSceneSlices(
			liveScene, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::AXIS_X, "_X");
	ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::RenderLiveSceneSlices(
			liveScene, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::AXIS_Y, "_Y");
	ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::RenderLiveSceneSlices(
			liveScene, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::AXIS_Z, "_Z");
#endif
#ifdef DRAW_IMAGE
	std::cout << "Desired warp update (voxels) below " << maxVectorUpdateThresholdVoxels << std::endl;
	cv::Mat canonicalImg =
			ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawCanonicalSceneImageAroundPoint(
					canonicalScene) * 255.0f;
	cv::Mat canonicalImgOut;
	canonicalImg.convertTo(canonicalImgOut, CV_8UC1);
	cv::cvtColor(canonicalImgOut, canonicalImgOut, cv::COLOR_GRAY2BGR);
	cv::imwrite(ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolder + "canonical.png",
				canonicalImgOut);
	cv::Mat liveImg =
			ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawLiveSceneImageAroundPoint(liveScene) *
			255.0f;
	cv::Mat liveImgTemplate, liveImgOut;
	liveImg.convertTo(liveImgTemplate, CV_8UC1);
	cv::cvtColor(liveImgTemplate, liveImgOut, cv::COLOR_GRAY2BGR);
	cv::imwrite(ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolder + "live.png",
				liveImgOut);
	cv::Mat blank = cv::Mat::zeros(liveImg.rows, liveImg.cols, CV_8UC1);
#endif

	std::string currentFrameOutputPath = GenerateCurrentFrameOutputPath();
	sceneLogger.SetPath(currentFrameOutputPath);//makes the folder as well if it doesn't exist
	sceneLogger.SetScenes(canonicalScene, liveScene);

	const std::string energyStatFilePath = currentFrameOutputPath + "/energy.txt";
	energy_stat_file = std::ofstream(energyStatFilePath.c_str(), std::ios_base::out);
	energy_stat_file << "data" << "," << "level_set" << "," << "smoothness" << ","
	                 << "killing" << "," << "total" << std::endl;


	if(recordWarpUpdates){
		sceneLogger.SaveScenesCompact();
		sceneLogger.StartSavingWarpState(currentFrameIx);
	}

	for (iteration = 0; maxVectorUpdate > maxVectorUpdateThresholdVoxels && iteration < maxIterationCount;
	     iteration++) {
		//START _DEBUG
#ifdef DRAW_IMAGE
		cv::Mat warpImg = ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawWarpedSceneImageAroundPoint(
				canonicalScene) * 255.0f;
		cv::Mat warpImgChannel, warpImgOut, mask, liveImgChannel, markChannel;
		blank.copyTo(markChannel);
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::MarkWarpedSceneImageAroundPoint(
				canonicalScene, markChannel, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos1);
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::MarkWarpedSceneImageAroundPoint(
				canonicalScene, markChannel, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos2);
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::MarkWarpedSceneImageAroundPoint(
				canonicalScene, markChannel, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos3);
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::MarkWarpedSceneImageAroundPoint(
				canonicalScene, markChannel, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos4);
		liveImgChannel = cv::Mat::zeros(warpImg.rows, warpImg.cols, CV_8UC1);
		warpImg.convertTo(warpImgChannel, CV_8UC1);
		cv::threshold(warpImgChannel, mask, 1.0, 1.0, cv::THRESH_BINARY_INV);
		liveImgTemplate.copyTo(liveImgChannel, mask);

		cv::Mat channels[3] = {liveImgTemplate, warpImgChannel, markChannel};

		cv::merge(channels, 3, warpImgOut);
		std::stringstream numStringStream;
		numStringStream << std::setw(3) << std::setfill('0') << iteration;
		std::string image_name =
				ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolder + "warp" +
				numStringStream.str() + ".png";
		cv::imwrite(image_name, warpImgOut);
#endif
		//END _DEBUG
		std::cout << red << "Iteration: " << iteration << reset;// << std::endl;
		maxVectorUpdate = UpdateWarpField(canonicalScene, liveScene);
		//START _DEBUG

		if(recordWarpUpdates){
			sceneLogger.SaveCurrentWarpState();
		}

		//END _DEBUG
	}

	if(recordWarpUpdates) {
		sceneLogger.StopSavingWarpState();
	}
	currentFrameIx++;
	energy_stat_file.close();

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::GenerateCurrentFrameOutputPath() const {
	return baseOutputDirectory + "/Frame_" + std::to_string(currentFrameIx);
}

