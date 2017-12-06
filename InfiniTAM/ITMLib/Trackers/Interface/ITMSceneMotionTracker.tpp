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

//_DEBUG (opencv)
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
//_DEBUG (local)
#define DRAW_IMAGE
#ifdef DRAW_IMAGE
#include "../../Utils/ITMSceneSliceRasterizer.h"
#endif

//local
#include "ITMSceneMotionTracker.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"



using namespace ITMLib;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::ProcessFrame(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
                                                         ITMScene<TVoxelLive, TIndex>* liveScene) {

	float maxVectorUpdate = std::numeric_limits<float>::infinity();

	//START _DEBUG

#ifdef DRAW_IMAGE
	std::cout << "Desired warp update (voxels) below " << maxVectorUpdateThresholdVoxels << std::endl;
	cv::Mat canonicalImg = ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawCanonicalSceneImage(canonicalScene) * 255.0f;
	cv::Mat canonicalImgOut;
	canonicalImg.convertTo(canonicalImgOut, CV_8UC1);
	cv::cvtColor(canonicalImgOut, canonicalImgOut, cv::COLOR_GRAY2BGR);
	cv::imwrite(ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolder + "canonical.png", canonicalImgOut);
	cv::Mat liveImg = ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawLiveSceneImage(liveScene) * 255.0f;
	cv::Mat liveImgTemplate, liveImgOut;
	liveImg.convertTo(liveImgTemplate, CV_8UC1);
	cv::cvtColor(liveImgTemplate, liveImgOut, cv::COLOR_GRAY2BGR);
	cv::imwrite(ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolder + "live.png", liveImgOut);
	cv::Mat blank = cv::Mat::zeros(liveImg.rows, liveImg.cols, CV_8UC1);
#endif
	//END _DEBUG

	//_DEBUG
	//TVoxelCanonical vox = ReadVoxel(*canonicalScene,testPos1);
	//vox.warp_t = Vector3f(0,0,0);
	//SetVoxel_CPU(*canonicalScene,testPos1,vox);

	//AllocateBoundaryHashBlocks(canonicalScene);

	for (int iteration = 0;
	     maxVectorUpdate > maxVectorUpdateThresholdVoxels && iteration < maxIterationCount; iteration++) {
		const std::string red("\033[0;31m");
		const std::string reset("\033[0m");
		//START _DEBUG
#ifdef DRAW_IMAGE
		cv::Mat warpImg = ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawWarpedSceneImage(canonicalScene) * 255.0f;
		cv::Mat warpImgChannel, warpImgOut, mask, liveImgChannel, markChannel;
		blank.copyTo(markChannel);
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::MarkWarpedSceneImage(canonicalScene, markChannel, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos1);
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::MarkWarpedSceneImage(canonicalScene, markChannel, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos2);
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive,TIndex>::MarkWarpedSceneImage(canonicalScene, markChannel, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive,TIndex>::testPos3);
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive,TIndex>::MarkWarpedSceneImage(canonicalScene, markChannel, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive,TIndex>::testPos4);
		liveImgChannel = cv::Mat::zeros(warpImg.rows, warpImg.cols, CV_8UC1);
		warpImg.convertTo(warpImgChannel, CV_8UC1);
		cv::threshold(warpImgChannel, mask, 1.0, 1.0, cv::THRESH_BINARY_INV);
		liveImgTemplate.copyTo(liveImgChannel, mask);

		cv::Mat channels[3] = {liveImgTemplate, warpImgChannel, markChannel};

		cv::merge(channels, 3, warpImgOut);
		std::stringstream numStringStream;
		numStringStream << std::setw(3) << std::setfill('0') << iteration;
		std::string image_name = ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive,TIndex>::iterationFramesFolder + "warp" + numStringStream.str() + ".png";
		cv::imwrite(image_name, warpImgOut);
#endif
		//END _DEBUG

		std::cout << red << "Iteration: " << iteration << reset;// << std::endl;
		maxVectorUpdate = UpdateWarpField(canonicalScene, liveScene);


		std::cout << " Max update: " << maxVectorUpdate << std::endl;
	}

	this->FuseFrame(canonicalScene, liveScene);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneMotionTracker(const ITMSceneParams& params) :
		maxVectorUpdateThresholdVoxels(maxVectorUpdateThresholdMeters / params.voxelSize) {}
