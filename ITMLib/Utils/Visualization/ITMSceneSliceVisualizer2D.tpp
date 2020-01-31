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
//stdlib

#ifdef WITH_OPENCV
#include <unordered_set>
#include <iomanip>

//opencv
#include <opencv2/imgcodecs.hpp>

//boost
#include <boost/filesystem.hpp>

//local
#include "ITMSceneSliceVisualizer2D.h"
#include "ITMSceneSliceVisualizerCommon.h"
#include "../../Objects/Scene/RepresentationAccess.h"
#include "../Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
#include "../../Engines/Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h"
#include "../../Engines/Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"

using namespace ITMLib;
namespace fs = boost::filesystem;


template<typename TVoxel, typename TWarp, typename TIndex>
const bool ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::absFillingStrategy = false;

inline float SdfToShadeValue(float sdf, bool absFillingStrategy) {
	return absFillingStrategy ? std::abs(sdf) : (sdf + 1.0f) / 2.0f;
}

inline bool IsPointInImageRange(int x, int y, int z, const Vector6i bounds) {
	return bounds.min_x <= x && bounds.max_x > x && bounds.min_y <= y && bounds.max_y > y && bounds.min_z <= z &&
	       bounds.max_z > z;
}


// region ===================================== CONSTRUCTORS / DESTRUCTORS =============================================

template<typename TVoxel, typename TWarp, typename TIndex>
ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::ITMSceneSliceVisualizer2D(Vector3i focusCoordinates,
                                                                            unsigned int imageSizeVoxels,
                                                                            float pixelsPerVoxel,
                                                                            Plane plane)
		:
		focusCoordinates(focusCoordinates),
		imageSizeVoxels(imageSizeVoxels),
		imageHalfSizeVoxels(imageSizeVoxels / 2),
		bounds(ComputeBoundsAroundPoint(focusCoordinates, imageSizeVoxels / 2, 0, plane)),
		imgXSlice(focusCoordinates.x),
		imgYSlice(focusCoordinates.y),
		imgZSlice(focusCoordinates.z),
		pixelsPerVoxel(pixelsPerVoxel),
		imgPixelRangeX(static_cast<int>(pixelsPerVoxel * bounds.max_x - bounds.min_x)),
		imgPixelRangeY(static_cast<int>(pixelsPerVoxel * bounds.max_y - bounds.min_y)),
		imgPixelRangeZ(static_cast<int>(pixelsPerVoxel * bounds.max_z - bounds.min_z)),
		plane(plane) {

}

// endregion ===========================================================================================================


inline
Vector2i GetVoxelImageCoordinates(float pixelsPerVoxel, const Vector3i& coordinates, const Vector6i& bounds,
                                  Plane plane, int imgPixelRangeY, int imgPixelRangeZ) {
	switch (plane) {
		case PLANE_XY:
			return Vector2i(static_cast<int>(pixelsPerVoxel * (coordinates.x - bounds.min_x)),
			                imgPixelRangeY - static_cast<int>(pixelsPerVoxel * (coordinates.y - bounds.min_y)));
		case PLANE_YZ:
			return Vector2i(static_cast<int>(pixelsPerVoxel * (coordinates.y - bounds.min_y)),
			                imgPixelRangeZ - static_cast<int>(pixelsPerVoxel * (coordinates.z - bounds.min_z)));
		case PLANE_XZ:
			return Vector2i(static_cast<int>(pixelsPerVoxel * (coordinates.x - bounds.min_x)),
			                imgPixelRangeZ - static_cast<int>(pixelsPerVoxel * (coordinates.z - bounds.min_z)));
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Plane value support not implemented");
	}
}


template<typename TVoxel>
struct DrawSceneVoxelAtOriginalPositionFunctor {
	DrawSceneVoxelAtOriginalPositionFunctor(int imageRangeY, int imageRangeZ, const Vector6i& bounds, Plane plane,
	                                        float pixelsPerVoxel,
	                                        bool absFillingStrategy, cv::Mat& img) :
			imageRangeY(imageRangeY), imageRangeZ(imageRangeZ), bounds(bounds), plane(plane),
			pixelsPerVoxel(pixelsPerVoxel), absFillingStrategy(absFillingStrategy), img(img) {}

	void operator()(TVoxel& voxel, Vector3i& position) {
		Vector2i imgCoords = GetVoxelImageCoordinates(pixelsPerVoxel, position, bounds, plane, imageRangeY,
		                                              imageRangeZ);
		const int voxelOnImageSize = static_cast<int>(pixelsPerVoxel);
		for (int row = imgCoords.y; row < imgCoords.y + voxelOnImageSize; row++) {
			for (int col = imgCoords.x; col < imgCoords.x + voxelOnImageSize; col++) {
				float sdf = voxel.sdf;
				float value = SdfToShadeValue(sdf, absFillingStrategy);
				img.at<float>(row, col) = value;
			}
		}
	}

private:
	const bool absFillingStrategy;
	const float pixelsPerVoxel;
	const int imageRangeY;
	const int imageRangeZ;
	const Vector6i& bounds;
	const Plane plane;
	cv::Mat& img;
};


template<typename TVoxel, typename TWarp, typename TIndex>
cv::Mat ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::DrawSceneImageAroundPoint(
		VoxelVolume<TVoxel, TIndex>* scene) {
	//use if default voxel SDF value is 1.0
	cv::Mat img = cv::Mat::ones(imgPixelRangeX, imgPixelRangeY, CV_32F);

	std::unordered_set<float> valueSet = {};
	DrawSceneVoxelAtOriginalPositionFunctor<TVoxel> drawSceneVoxelFunctor(
			imgPixelRangeY, imgPixelRangeZ, bounds, plane,
			this->pixelsPerVoxel, absFillingStrategy, img);
	VolumeTraversalEngine<TVoxel, TIndex, MEMORYDEVICE_CPU>::
	        VoxelPositionTraversalWithinBounds(scene,drawSceneVoxelFunctor, bounds);
	return img;
}


template<typename TVoxel, typename TWarp>
struct DrawSceneVoxelAtWarpedPositionFunctor {
	DrawSceneVoxelAtWarpedPositionFunctor(int imageRangeY, int imageRangeZ, const Vector6i& bounds, Plane plane,
	                                      float pixelsPerVoxel, bool absFillingStrategy, cv::Mat& img) :
			imageRangeY(imageRangeY), imageRangeZ(imageRangeZ), bounds(bounds), plane(plane),
			pixelsPerVoxel(pixelsPerVoxel), absFillingStrategy(absFillingStrategy), img(img) {}

	void operator()(TVoxel& voxel, TWarp& warp, Vector3i& position) {

		Vector3f projectedPosition = position.toFloat() + warp.framewise_warp;
		Vector3i projectedPositionFloored = projectedPosition.toIntFloor();
		switch (plane) {
			case PLANE_XY:
				projectedPositionFloored.z = position.z;
				break;
			case PLANE_YZ:
				projectedPositionFloored.x = position.x;
				break;
			case PLANE_XZ:
				projectedPosition.y = position.y;
				break;
		}
		if (!IsPointInImageRange(projectedPositionFloored.x, projectedPositionFloored.y,
		                         projectedPositionFloored.z, bounds))
			return;

		Vector2i imgCoords = GetVoxelImageCoordinates(pixelsPerVoxel, projectedPositionFloored, bounds, plane,
		                                              imageRangeY,
		                                              imageRangeZ);
		const int voxelOnImageSize = static_cast<int>(pixelsPerVoxel);
		float value = 0.0;//SdfToShadeValue(TVoxelA::valueToFloat(voxel.sdf));
		//fill a pixel block with the source scene value
		for (int row = imgCoords.y; row < imgCoords.y + voxelOnImageSize / 2; row++) {
			for (int col = imgCoords.x; col < imgCoords.x + voxelOnImageSize / 2; col++) {
				img.at<float>(row, col) = value;
			}
		}
	}

private:
	const bool absFillingStrategy;
	const float pixelsPerVoxel;
	const int imageRangeY;
	const int imageRangeZ;
	const Vector6i& bounds;
	const Plane plane;
	cv::Mat& img;
};

template<typename TVoxel, typename TWarp, typename TIndex>
cv::Mat ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::DrawWarpedSceneImageAroundPoint(
		VoxelVolume<TVoxel, TIndex>* scene,
		VoxelVolume<TWarp, TIndex>* warpField) {
#ifdef IMAGE_BLACK_BACKGROUND
	//use if default voxel SDF value is -1.0
	cv::Mat img = cv::Mat::zeros(imgPixelRangeX, imgPixelRangeY, CV_32F);
#else
	//use if default voxel SDF value is 1.0
	cv::Mat img = cv::Mat::ones(imgPixelRangeX, imgPixelRangeY, CV_32F);
#endif
	const int margin = 5;
	Vector6i expandedBounds = Vector6i(
			bounds.min_x + margin,
			bounds.min_y + margin,
			bounds.min_z + margin,
			bounds.max_x + margin,
			bounds.max_y + margin,
			bounds.max_z + margin);
	DrawSceneVoxelAtWarpedPositionFunctor<TVoxel, TWarp> drawSceneVoxelFunctor(
			imgPixelRangeY, imgPixelRangeZ, bounds, plane,
			this->pixelsPerVoxel, absFillingStrategy, img);
	TwoVolumeTraversalEngine<TVoxel, TWarp, TIndex, TIndex, MEMORYDEVICE_CPU>::
	        template DualVoxelPositionTraversalWithinBounds(scene, warpField, drawSceneVoxelFunctor, expandedBounds);
	return img;
}


/**
 * \brief Mark/highlight the (warped) voxel on an image generated from the given scene's voxels
 * around a specific (potentially, different) point
 * \tparam TVoxelCanonical canonical voxel type
 * \tparam TVoxelLive live voxel type
 * \tparam TIndex index structure type
 * \param scene the voxel grid the image was generated from
 * \param imageToMarkOn the image that was generated from the warped vosel positions that should be marked
 * \param positionOfVoxelToMark voxel position (before warp application) of the voxel to highlight/mark
 */
template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::MarkWarpedSceneImageAroundPoint(
		VoxelVolume<TVoxel, TIndex>* scene, VoxelVolume<TWarp, TIndex>* warpField, cv::Mat& imageToMarkOn,
		Vector3i positionOfVoxelToMark) {
	TVoxel voxel = EditAndCopyEngine_CPU<TVoxel, TIndex>::Inst().ReadVoxel(scene, positionOfVoxelToMark);
	TWarp warp = EditAndCopyEngine_CPU<TWarp, TIndex>::Inst().ReadVoxel(warpField, positionOfVoxelToMark);

	Vector3f projectedPosition = positionOfVoxelToMark.toFloat() + warp.framewise_warp;
	Vector3i projectedPositionFloored = projectedPosition.toIntFloor();
	switch (plane) {
		case PLANE_XY:
			projectedPositionFloored.z = positionOfVoxelToMark.z;
			break;
		case PLANE_YZ:
			projectedPositionFloored.x = positionOfVoxelToMark.x;
			break;
		case PLANE_XZ:
			projectedPosition.y = positionOfVoxelToMark.y;
			break;
	}
	if (!IsPointInImageRange(projectedPositionFloored.x, projectedPositionFloored.y, positionOfVoxelToMark.z))
		return;

	Vector2i imageCoords = GetVoxelImageCoordinates(projectedPosition, this->plane);
	const int voxelOnImageSize = static_cast<int>(pixelsPerVoxel);

	//fill a pixel block with the source scene value
	for (int row = imageCoords.y; row < imageCoords.y + voxelOnImageSize / 2; row++) {
		for (int col = imageCoords.x; col < imageCoords.x + voxelOnImageSize / 2; col++) {
			imageToMarkOn.at<uchar>(row, col) = static_cast<uchar>(255.0f);
		}
	}

}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::IsPointInImageRange(int x, int y, int z) const {
	return bounds.min_x <= x && bounds.max_x > x && bounds.min_y <= y && bounds.max_y > y && bounds.min_z <= z &&
	       bounds.max_z > z;
}


template<typename TVoxel, typename TWarp, typename TIndex>
Vector2i
ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::GetVoxelImageCoordinates(
		Vector3i coordinates,
		Plane plane) {
	switch (plane) {
		case PLANE_XY:
			return Vector2i(static_cast<int>(pixelsPerVoxel * (coordinates.x - bounds.min_x)),
			                imgPixelRangeY - static_cast<int>(pixelsPerVoxel * (coordinates.y - bounds.min_y)));
		case PLANE_YZ:
			return Vector2i(static_cast<int>(pixelsPerVoxel * (coordinates.y - bounds.min_y)),
			                imgPixelRangeZ - static_cast<int>(pixelsPerVoxel * (coordinates.z - bounds.min_z)));
		case PLANE_XZ:
			return Vector2i(static_cast<int>(pixelsPerVoxel * (coordinates.x - bounds.min_x)),
			                imgPixelRangeZ - static_cast<int>(pixelsPerVoxel * (coordinates.z - bounds.min_z)));
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Plane value support not implemented");
	}
}


template<typename TVoxel, typename TWarp, typename TIndex>
Vector2i
ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::GetVoxelImageCoordinates(
		Vector3f coordinates,
		Plane plane) {
	switch (plane) {
		case PLANE_XY:
			return Vector2i(static_cast<int>(pixelsPerVoxel * (coordinates.x - bounds.min_x)),
			                imgPixelRangeY - static_cast<int>(pixelsPerVoxel * (coordinates.y - bounds.min_y)));
		case PLANE_YZ:
			return Vector2i(static_cast<int>(pixelsPerVoxel * (coordinates.y - bounds.min_y)),
			                imgPixelRangeZ - static_cast<int>(pixelsPerVoxel * (coordinates.z - bounds.min_z)));
		case PLANE_XZ:
			return Vector2i(static_cast<int>(pixelsPerVoxel * (coordinates.x - bounds.min_x)),
			                imgPixelRangeZ - static_cast<int>(pixelsPerVoxel * (coordinates.z - bounds.min_z)));
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Plane value support not implemented");
	}
}


template<typename TVoxel>
struct DrawSingleVoxelForSliceFunctor {
	DrawSingleVoxelForSliceFunctor(std::vector<cv::Mat>& images, Axis axis, Vector3i focusCoordinates,
	                               Vector3i minPoint, bool absFillingStrategy) :
			images(images), axis(axis), focusCoordinates(focusCoordinates), minPoint(minPoint),
			absFillingStrategy(absFillingStrategy) {}

	void operator()(TVoxel& voxel, Vector3i voxelPosition) {
		//determine pixel coordinate
		Vector2i pixelCoordinate;
		int iImage;
		Vector3i absoluteCoordinate = voxelPosition - minPoint; // move origin to minPoint
		switch (axis) {
			case AXIS_X:
				//Y-Z plane
				pixelCoordinate.x = absoluteCoordinate.z;
				pixelCoordinate.y = absoluteCoordinate.y;
				iImage = absoluteCoordinate.x;
				break;
			case AXIS_Y:
				//Z-X plane
				pixelCoordinate.x = absoluteCoordinate.x;
				pixelCoordinate.y = absoluteCoordinate.z;
				iImage = absoluteCoordinate.y;
				break;
			case AXIS_Z:
				//X-Y plane
				pixelCoordinate.x = absoluteCoordinate.x;
				pixelCoordinate.y = absoluteCoordinate.y;
				iImage = absoluteCoordinate.z;
				break;
		}
		float sdf = TVoxel::valueToFloat(voxel.sdf);
		float value = absFillingStrategy ? std::abs(sdf) : (sdf + 1.0f) / 2.0f; //TODO: DRY violation
		uchar colorChar = static_cast<uchar>(value * 255.0);
		cv::Vec3b color = cv::Vec3b::all(colorChar);
		if (voxelPosition == focusCoordinates) {
			color.val[0] = 0;
			color.val[1] = 0;
			color.val[2] = static_cast<uchar>(255.0);
		}
		images[iImage].at<cv::Vec3b>(pixelCoordinate.y, pixelCoordinate.x) = color;
	}

private:
	std::vector<cv::Mat>& images;
	Axis axis;
	Vector3i focusCoordinates;
	Vector3i minPoint;
	bool absFillingStrategy;
};

template<typename TVoxel, typename TWarp, typename TIndex>
void
ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::RenderSceneSlices(VoxelVolume<TVoxel, TIndex>* scene,
                                                                    Axis axis,
                                                                    const std::string& outputFolder,
                                                                    bool verbose) {

	Vector6i bounds = ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::Instance().ComputeVoxelBounds(scene);
	Vector3i minPoint(bounds.min_x, bounds.min_y, bounds.min_z);

	int imageSizeX, imageSizeY, imageSizeZ;

	imageSizeX = bounds.max_x - minPoint.x;
	imageSizeY = bounds.max_y - minPoint.y;
	imageSizeZ = bounds.max_z - minPoint.z;

	cv::Size imSize;
	int imageCount = 0;
	int indexOffset = 0;
	switch (axis) {
		case AXIS_X:
			imSize = cv::Size(imageSizeZ, imageSizeY);
			imageCount = imageSizeX;
			indexOffset = minPoint.x;
			break;
		case AXIS_Y:
			imSize = cv::Size(imageSizeX, imageSizeZ);
			imageCount = imageSizeY;
			indexOffset = minPoint.y;
			break;
		case AXIS_Z:
			imSize = cv::Size(imageSizeX, imageSizeY);
			imageCount = imageSizeZ;
			indexOffset = minPoint.z;
			break;
	}
	std::vector<cv::Mat> images;
	images.reserve(static_cast<unsigned long>(imageCount));

	//generate image stack
	for (int iImage = 0; iImage < imageCount; iImage++) {
		images.push_back(cv::Mat::zeros(imSize, CV_8UC3));
	}
	DrawSingleVoxelForSliceFunctor<TVoxel> drawSingleVoxelForSliceFunctor(images, axis, focusCoordinates, minPoint,
	                                                                      absFillingStrategy);
	VolumeTraversalEngine<TVoxel, TIndex, MEMORYDEVICE_CPU>::
	VoxelPositionTraversal(scene, drawSingleVoxelForSliceFunctor);

	for (int iImage = 0; iImage < imageCount; iImage++) {
		std::stringstream ss;
		ss << outputFolder << "/slice" << std::setfill('0') << std::setw(5) << iImage + indexOffset << ".png";
		cv::imwrite(ss.str(), images[iImage]);
		if (verbose) {
			std::cout << "Writing " << ss.str() << std::endl;
		}
	}
}


template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::SaveSceneSlicesAs2DImages_AllDirections(
		VoxelVolume<TVoxel, TIndex>* scene, std::string pathWithoutPostfix) {
	fs::create_directories(pathWithoutPostfix + "/X");
	fs::create_directories(pathWithoutPostfix + "/Y");
	fs::create_directories(pathWithoutPostfix + "/Z");
	ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::SaveSceneSlicesAs2DImages(
			scene, AXIS_X, pathWithoutPostfix + "/X");
	ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::SaveSceneSlicesAs2DImages(
			scene, AXIS_Y, pathWithoutPostfix + "/Y");
	ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::SaveSceneSlicesAs2DImages(
			scene, AXIS_Z, pathWithoutPostfix + "/Z");
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::SaveSceneSlicesAs2DImages(
		VoxelVolume<TVoxel, TIndex>* scene, Axis axis, std::string path) {
	RenderSceneSlices(scene, axis, path, false);
}


template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::MarkWarpedSceneImageAroundFocusPoint(
		VoxelVolume<TVoxel, TIndex>* scene,
		VoxelVolume<TWarp, TIndex>* warpField, cv::Mat& imageToMarkOn) {
	MarkWarpedSceneImageAroundPoint(scene, warpField, imageToMarkOn, focusCoordinates);
}

template<typename TVoxel, typename TWarp, typename TIndex>
float ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::SdfToShadeValue(float sdf) {
	return absFillingStrategy ? std::abs(sdf) : (sdf + 1.0f) / 2.0f;
}

template<typename TVoxel, typename TWarp, typename TIndex>
Plane ITMSceneSliceVisualizer2D<TVoxel, TWarp, TIndex>::GetPlane() const {
	return this->plane;
}

#endif // #ifdef WITH_OPENCV



