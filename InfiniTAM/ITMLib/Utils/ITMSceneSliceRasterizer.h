//  ================================================================
//  Created by Gregory Kramida on 12/5/17.
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

#include <opencv2/core/core.hpp>
#include "ITMMath.h"
#include "../Objects/Scene/ITMScene.h"

namespace ITMLib {

template <typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneSliceRasterizer {

	//TODO: positions of voxels to highlight / draw around should be defined extenally in the user code, not as class static members. -Greg (GitHub: Algomorph)
public:
	static cv::Mat DrawCanonicalSceneImageAroundPoint(ITMScene<TVoxelCanonical, TIndex>* scene);
	static cv::Mat DrawLiveSceneImageAroundPoint(ITMScene<TVoxelLive, TIndex>* scene);
	static cv::Mat DrawWarpedSceneImageAroundPoint(ITMScene<TVoxelCanonical, TIndex>* scene);
	static void MarkWarpedSceneImageAroundPoint(ITMScene<TVoxelCanonical, TIndex>* scene, cv::Mat& imageToMarkOn,
	                                            Vector3i positionOfVoxelToMark);

	// voxels to highlight and use as drawing canvas center
	static const Vector3i testPos1;
	static const Vector3i testPos2;
	static const Vector3i testPos3;
	static const Vector3i testPos4;
// where to save the images
	static const std::string iterationFramesFolder;

protected:
	template<typename TTVoxel>
	static cv::Mat DrawSceneImageAroundPoint(ITMScene<TTVoxel, TIndex>* scene);
	template<typename TTVoxel>
	static cv::Mat DrawWarpedSceneImageTemplated(ITMScene <TTVoxel, TIndex>* scene);


	static Vector2i GetVoxelImgCoords(int x, int y);
	static Vector2i GetVoxelImgCoords(float x, float y);
	static bool IsVoxelInImgRange(int x, int y, int z);
	static bool IsVoxelBlockInImgRange(Vector3i blockVoxelCoords);
	static bool IsVoxelBlockInImgRangeTolerance(Vector3i blockVoxelCoords, int tolerance);

	static const bool absFillingStrategy;
	static const int imageSizeVoxels;
	static const int imageHalfSizeVoxels;
	static const int imgRangeStartX;
	static const int imgRangeEndX;
	static const int imgRangeStartY;
	static const int imgRangeEndY;
	static const int imgZSlice;

	static const int imgVoxelRangeX;
	static const int imgVoxelRangeY;

	static const float imgToVoxelScale;

	static const int imgPixelRangeX;
	static const int imgPixelRangeY;
};


}//namespace ITMLib


