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
#include "../ITMMath.h"
#include "../../Objects/Scene/ITMScene.h"

namespace ITMLib {

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMScene2DSliceVisualizer {

	//TODO: positions of voxels to highlight / draw around should be defined extenally in the user code, not as class static members. Static functions should be probably changed to become member functions, with focus coordinates (testPos1, testPos2...) being set in user code during construction. -Greg (GitHub: Algomorph)
public:
	enum Axis {
		AXIS_X = 0,
		AXIS_Y = 1,
		AXIS_Z = 3
	};

	enum Plane{
		PLANE_XY = 0,
		PLANE_YZ = 1,
		PLANE_XZ = 2
	};

	static std::string PlaneToString(Plane plane);

	// where to save the images within the output directory
	static const std::string iterationFramesFolderName;
	static const std::string liveIterationFramesFolderName;
	static const std::string canonicalSceneRasterizedFolderName;
	static const std::string liveSceneRasterizedFolderName;

	static float SdfToValue(float sdf);

	explicit ITMScene2DSliceVisualizer(Vector3i focusCoordinate, std::string outputDirectory,
	                               unsigned int imageSizeVoxels = 100,
	                               float pixelsPerVoxel = 16.0, Plane plane = PLANE_XY);

	virtual ~ITMScene2DSliceVisualizer() = default;

	void MakeOrClearOutputDirectories() const;
	std::string GetOutputDirectoryForWarps() const;
	std::string GetOutputDirectoryForWarpedLiveScenes() const;
	void SaveLiveSceneSlicesAs2DImages_AllDirections(ITMScene <TVoxelLive, TIndex>* scene);
	void SaveLiveSceneSlicesAs2DImages_AllDirections(ITMScene <TVoxelCanonical, TIndex>* scene);
	void SaveCanonicalSceneSlicesAs2DImages(ITMScene <TVoxelCanonical, TIndex>* scene,
	                                        Axis axis, std::string pathPostfix);
	void SaveLiveSceneSlicesAs2DImages(ITMScene <TVoxelLive, TIndex>* scene,
	                                   Axis axis, std::string pathPostfix);

	cv::Mat DrawCanonicalSceneImageAroundPoint(ITMScene <TVoxelCanonical, TIndex>* scene);
	cv::Mat DrawLiveSceneImageAroundPoint(
			ITMScene <TVoxelLive, TIndex>* scene, int fieldIndex);
	cv::Mat DrawWarpedSceneImageAroundPoint(ITMScene <TVoxelCanonical, TIndex>* scene);

	void MarkWarpedSceneImageAroundFocusPoint(ITMScene <TVoxelCanonical, TIndex>* scene, cv::Mat& imageToMarkOn);
	void MarkWarpedSceneImageAroundPoint(ITMScene <TVoxelCanonical, TIndex>* scene, cv::Mat& imageToMarkOn, Vector3i positionOfVoxelToMark);




	const Vector3i focusCoordinate;
	const std::string outputDirectory;
	const float pixelsPerVoxel;

protected:
	template<typename TVoxel>
	void RenderSceneSlices(ITMScene <TVoxel, TIndex>* scene,
	                       Axis axis,
	                       const std::string& outputFolder,
	                       bool verbose = false);
	template<typename TVoxel>
	cv::Mat DrawSceneImageAroundPoint(ITMScene <TVoxel, TIndex>* scene);
	template<typename TVoxel>
	cv::Mat DrawWarpedSceneImageTemplated(ITMScene <TVoxel, TIndex>* scene);


	Vector2i GetVoxelImageCoordinates(Vector3i coordinates, Plane plane);
	Vector2i GetVoxelImageCoordinates(Vector3f coordinates, Plane plane);

	bool IsVoxelInImageRange(int x, int y, int z, Plane plane) const;
	bool IsVoxelBlockInImageRange(Vector3i blockVoxelCoords, Plane plane) const;
	bool IsVoxelBlockInImageRangeTolerance(Vector3i blockVoxelCoords, int tolerance, Plane plane) const;

	void SetPlane(Plane plane);

	static const bool absFillingStrategy;

	const int imageSizeVoxels;
	const int imageHalfSizeVoxels;

	const int imgRangeStartX;
	const int imgRangeEndX;
	const int imgRangeStartY;
	const int imgRangeEndY;
	const int imgRangeStartZ;
	const int imgRangeEndZ;

	const int imgXSlice;
	const int imgYSlice;
	const int imgZSlice;

	const int imgVoxelRangeX;
	const int imgVoxelRangeY;
	const int imgVoxelRangeZ;

	const int imgPixelRangeX;
	const int imgPixelRangeY;
	const int imgPixelRangeZ;

	Plane plane;

	template<typename TVoxel>
	cv::Mat DrawSceneImageAroundPointIndexedFields(ITMScene <TVoxel, TIndex>* scene, int fieldIndex);
};


}//namespace ITMLib


