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
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneMotionTracker_CUDA :
		public ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex> {
public:

	explicit ITMSceneMotionTracker_CUDA(const ITMSceneParams& params, std::string scenePath);
	void
	FuseFrame(ITMScene <TVoxelCanonical, TIndex>* canonicalScene, ITMScene <TVoxelLive, TIndex>* liveScene) override;
	void WarpCanonicalToLive(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                         ITMScene <TVoxelLive, TIndex>* liveScene) override;

protected:
	void AllocateNewCanonicalHashBlocks(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                                    ITMScene <TVoxelLive, TIndex>* liveScene) override;
	void ApplyWarpFieldToLive(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
		                          ITMScene <TVoxelLive, TIndex>* sourceLiveScene) override;
	float CalculateWarpUpdate(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                          ITMScene <TVoxelLive, TIndex>* liveScene) override;
	void ApplySmoothingToGradient(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                              ITMScene <TVoxelLive, TIndex>* liveScene) override;
	void ApplyWarpUpdateToLive(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
		                           ITMScene <TVoxelLive, TIndex>* sourceLiveScene) override;

	float ApplyWarpUpdateToWarp(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                            ITMScene <TVoxelLive, TIndex>* liveScene) override;

	void ClearOutFrameWarps(ITMScene <TVoxelCanonical, TIndex>* canonicalScene) override;
	void ApplyFrameWarpsToWarps(ITMScene<TVoxelCanonical, TIndex>* canonicalScene) override;
};


}//namespace ITMLib


