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
#pragma once

#include <opencv2/core/mat.hpp>
#include "../Interface/ITMSceneMotionTracker.h"

namespace ITMLib {
	template<typename TVoxel, typename TIndex>
	class ITMSceneMotionTracker_CPU :
			public ITMSceneMotionTracker<TVoxel, TIndex> {
	public:

		explicit ITMSceneMotionTracker_CPU(const ITMSceneParams& params);

	protected:
		//START _DEBUG
		template<typename TTVoxel>
		cv::Mat DrawSceneImage(ITMScene <TTVoxel, TIndex>* scene);

		template<typename TTVoxel>
		cv::Mat DrawWarpedSceneImageTemplated(ITMScene <TTVoxel, TIndex>* scene);

		cv::Mat DrawCanonicalSceneImage(ITMScene <TVoxel, TIndex>* scene) override {
			return DrawSceneImage(scene);
		}

		cv::Mat DrawWarpedSceneImage(ITMScene <TVoxel, TIndex>* scene) override {
			return DrawWarpedSceneImageTemplated(scene);
		}

		cv::Mat DrawLiveSceneImage(ITMScene <ITMVoxelAux, TIndex>* scene) override {
			return DrawSceneImage(scene);
		}

		//timers
		double timeWarpUpdateCompute = 0.0;
		double timeDataJandHCompute = 0.0;
		double timeWarpJandHCompute = 0.0;
		double timeUpdateTermCompute = 0.0;
		double timeWarpUpdateApply = 0.0;

		//END _DEBUG

		float
		UpdateWarpField(ITMScene <TVoxel, TIndex>* canonicalScene, ITMScene <ITMVoxelAux, TIndex>* liveScene) override;

		void FuseFrame(ITMScene <TVoxel, TIndex>* canonicalScene, ITMScene <ITMVoxelAux, TIndex>* liveScene) override;

	private:
		//debug image stuff START _DEBUG
		const bool absFillingStrategy = false;
		const int imageSizeVoxels = 100;
		const int imageHalfSizeVoxels = imageSizeVoxels / 2;
		const int imgRangeStartX = (ITMSceneMotionTracker<TVoxel, TIndex>::testPos1).x - imageHalfSizeVoxels;
		const int imgRangeEndX = (ITMSceneMotionTracker<TVoxel, TIndex>::testPos1).x + imageHalfSizeVoxels;
		const int imgRangeStartY = (ITMSceneMotionTracker<TVoxel, TIndex>::testPos1).y - imageHalfSizeVoxels;
		const int imgRangeEndY = (ITMSceneMotionTracker<TVoxel, TIndex>::testPos1).y + imageHalfSizeVoxels;
		const int imgZSlice = (ITMSceneMotionTracker<TVoxel, TIndex>::testPos1).z;

		const int imgVoxelRangeX = imgRangeEndX - imgRangeStartX;
		const int imgVoxelRangeY = imgRangeEndY - imgRangeStartY;

		const float imgToVoxelScale = 16.0;

		const int imgPixelRangeX = static_cast<int>(imgToVoxelScale * imgVoxelRangeX);
		const int imgPixelRangeY = static_cast<int>(imgToVoxelScale * imgVoxelRangeY);

		bool isVoxelInImgRange(int x, int y, int z);

		bool isVoxelBlockInImgRange(Vector3i blockVoxelCoords);

		Vector2i getVoxelImgCoords(int x, int y);
		//END _DEBUG

		bool isVoxelBlockInImgRangeTolerance(Vector3i blockVoxelCoords, int tolerance);

		Vector2i getVoxelImgCoords(float x, float y);

		void MarkWarpedSceneImage(ITMScene <TVoxel, TIndex>* scene, cv::Mat& image, Vector3i position) override;
	};


}//namespace ITMLib


