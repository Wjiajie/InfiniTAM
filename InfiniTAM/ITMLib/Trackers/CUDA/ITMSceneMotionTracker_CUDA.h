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

#include "../Interface/ITMSceneMotionTracker.h"

namespace ITMLib {
	template<class TVoxel, class TIndex>
	class ITMSceneMotionTracker_CUDA :
			public ITMSceneMotionTracker<TVoxel, TIndex> {
	public:

		explicit ITMSceneMotionTracker_CUDA(const ITMSceneParams& params);

	protected:

		//START _DEBUG
		cv::Mat DrawCanonicalSceneImage(ITMScene <TVoxel, TIndex>* scene) override {
			return cv::Mat();
		};

		cv::Mat DrawLiveSceneImage(ITMScene <ITMVoxelAux, TIndex>* scene) override {
			return cv::Mat();
		}
		cv::Mat DrawWarpedSceneImage(ITMScene <TVoxel, TIndex>* scene) override{
			return cv::Mat();
		}

		void MarkWarpedSceneImage(ITMScene <TVoxel, TIndex>* scene, cv::Mat& image, Vector3i position) override {
		}
		//END _DEBUG

		void FuseFrame(ITMScene <TVoxel, TIndex>* canonicalScene, ITMScene <ITMVoxelAux, TIndex>* liveScene) override;
		float UpdateWarpField(ITMScene <TVoxel, TIndex>* canonicalScene, ITMScene <ITMVoxelAux, TIndex>* liveScene) override;
	};


}//namespace ITMLib


