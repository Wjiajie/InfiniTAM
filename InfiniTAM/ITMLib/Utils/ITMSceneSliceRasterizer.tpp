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
#include <unordered_set>
#include "ITMSceneSliceRasterizer.h"
#include "../Objects/Scene/ITMRepresentationAccess.h"

using namespace ITMLib;

//====================================== DEFINE CONSTANTS ==============================================================
// voxels to highlight and use as drawing canvas center
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const Vector3i ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos1 = Vector3i(54, -21, 286);
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const Vector3i ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos2 = Vector3i(54, -20, 286);
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const Vector3i ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos3 = Vector3i(53, -21, 286);
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const Vector3i ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos4 = Vector3i(53, -20, 286);
// where to save the images
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolder =
		"/media/algomorph/Data/4dmseg/Killing/iteration_frames/";

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const bool ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::absFillingStrategy = false;
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::imageSizeVoxels = 100;
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::imageHalfSizeVoxels = imageSizeVoxels / 2;
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::imgRangeStartX = testPos1.x - imageHalfSizeVoxels;
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::imgRangeEndX = testPos1.x + imageHalfSizeVoxels;
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::imgRangeStartY = testPos1.y - imageHalfSizeVoxels;
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::imgRangeEndY = testPos1.y + imageHalfSizeVoxels;
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::imgZSlice = testPos1.z;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::imgVoxelRangeX = imgRangeEndX - imgRangeStartX;
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::imgVoxelRangeY = imgRangeEndY - imgRangeStartY;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const float ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::imgToVoxelScale = 16.0;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::imgPixelRangeX = static_cast<int>(imgToVoxelScale * imgVoxelRangeX);
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::imgPixelRangeY = static_cast<int>(imgToVoxelScale * imgVoxelRangeY);

// ============================================ END CONSTANT DEFINITIONS ===============================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
template <typename TVoxel>
cv::Mat ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawSceneImageAroundPoint(
		ITMScene<TVoxel, TIndex>* scene) {
#ifdef IMAGE_BLACK_BACKGROUND
	//use if default voxel SDF value is -1.0
	cv::Mat img = cv::Mat::zeros(imgPixelRangeX, imgPixelRangeY, CV_32F);
#else
	//use if default voxel SDF value is 1.0
	cv::Mat img = cv::Mat::ones(imgPixelRangeX, imgPixelRangeY, CV_32F);
#endif
	TVoxel* voxelBlocks = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
	typename TIndex::IndexCache canonicalCache;

	std::unordered_set<float> valueSet = {};

	int numPixelsFilled = 0;

#ifdef WITH_OPENMP
#pragma omp parallel for reduction(+:numPixelsFilled)
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		Vector3i currentBlockPositionVoxels;
		const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		currentBlockPositionVoxels = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		if (!IsVoxelBlockInImgRange(currentBlockPositionVoxels)) continue;

		TVoxel* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = currentBlockPositionVoxels + Vector3i(x, y, z);
					if (!IsVoxelInImgRange(originalPosition.x, originalPosition.y, originalPosition.z)) continue;

					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxel& voxel = localVoxelBlock[locId];
					Vector2i imgCoords = GetVoxelImgCoords(originalPosition.x, originalPosition.y);
					const int voxelOnImageSize = static_cast<int>(imgToVoxelScale);
					for (int row = imgCoords.y; row < imgCoords.y + voxelOnImageSize; row++) {
						for (int col = imgCoords.x; col < imgCoords.x + voxelOnImageSize; col++) {
							float sdfRepr = absFillingStrategy ? std::abs(voxel.sdf) : (voxel.sdf + 1.0) / 2.0;
							img.at<float>(row, col) = sdfRepr;
						}
					}
				}
			}
		}
	}
	return img;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
template<typename TVoxel>
cv::Mat ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawWarpedSceneImageTemplated(ITMScene<TVoxel, TIndex>* scene) {
#ifdef IMAGE_BLACK_BACKGROUND
	//use if default voxel SDF value is -1.0
	cv::Mat img = cv::Mat::zeros(imgPixelRangeX, imgPixelRangeY, CV_32F);
#else
	//use if default voxel SDF value is 1.0
	cv::Mat img = cv::Mat::ones(imgPixelRangeX, imgPixelRangeY, CV_32F);
#endif

	TVoxel* voxelBlocks = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
	typename TIndex::IndexCache canonicalCache;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		Vector3i currentBlockPositionVoxels;
		const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		currentBlockPositionVoxels = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		if (!IsVoxelBlockInImgRangeTolerance(currentBlockPositionVoxels, 5)) continue;

		TVoxel* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = currentBlockPositionVoxels + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxel& voxel = localVoxelBlock[locId];
					Vector3f projectedPosition = originalPosition.toFloat() + voxel.warp_t;
					Vector3i projectedPositionFloored = projectedPosition.toIntFloor();
					if (!IsVoxelInImgRange(projectedPositionFloored.x, projectedPositionFloored.y,
					                       originalPosition.z))
						continue;

					Vector2i imgCoords = GetVoxelImgCoords(projectedPosition.x, projectedPosition.y);
					const int voxelOnImageSize = static_cast<int>(imgToVoxelScale);
					float sdfRepr;
					sdfRepr = absFillingStrategy ? std::abs(voxel.sdf) : (voxel.sdf + 1.0) / 2.0;
					//fill a pixel block with the source scene value
					for (int row = imgCoords.y; row < imgCoords.y + voxelOnImageSize / 2; row++) {
						for (int col = imgCoords.x; col < imgCoords.x + voxelOnImageSize / 2; col++) {
							img.at<float>(row, col) = sdfRepr;
						}
					}
				}
			}
		}
	}
	return img;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
cv::Mat ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawCanonicalSceneImageAroundPoint(
		ITMScene<TVoxelCanonical, TIndex>* scene) {
	return DrawSceneImageAroundPoint(scene);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
cv::Mat ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawLiveSceneImageAroundPoint(
		ITMScene<TVoxelLive, TIndex>* scene) {
	return DrawSceneImageAroundPoint(scene);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
cv::Mat ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawWarpedSceneImageAroundPoint(
		ITMScene<TVoxelCanonical, TIndex>* scene) {
	return DrawWarpedSceneImageTemplated(scene);
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
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::MarkWarpedSceneImageAroundPoint(
		ITMScene<TVoxelCanonical, TIndex>* scene, cv::Mat& imageToMarkOn,
		Vector3i positionOfVoxelToMark) {
	bool vmIndex;
	TVoxelCanonical voxel = readVoxel(scene->localVBA.GetVoxelBlocks(), scene->index.GetEntries(), positionOfVoxelToMark, vmIndex);
	Vector3f projectedPosition = positionOfVoxelToMark.toFloat() + voxel.warp_t;
	Vector3i projectedPositionFloored = projectedPosition.toIntFloor();
	if (!IsVoxelInImgRange(projectedPositionFloored.x, projectedPositionFloored.y, positionOfVoxelToMark.z - 1) &&
	    !IsVoxelInImgRange(projectedPositionFloored.x, projectedPositionFloored.y, positionOfVoxelToMark.z) &&
	    !IsVoxelInImgRange(projectedPositionFloored.x, projectedPositionFloored.y, positionOfVoxelToMark.z + 1)) return;

	Vector2i imgCoords = GetVoxelImgCoords(projectedPosition.x, projectedPosition.y);
	const int voxelOnImageSize = static_cast<int>(imgToVoxelScale);
	float sdfRepr;
	//sdfRepr = std::abs(voxel.sdf);
	sdfRepr = 1.0f;// - sdfRepr*.6f;

	//fill a pixel block with the source scene value
	for (int row = imgCoords.y; row < imgCoords.y + voxelOnImageSize / 2; row++) {
		for (int col = imgCoords.x; col < imgCoords.x + voxelOnImageSize / 2; col++) {
			imageToMarkOn.at<uchar>(row, col) = static_cast<uchar>(sdfRepr * 255.0f);
		}
	}

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::IsVoxelInImgRange(int x, int y, int z) {
	return (z == imgZSlice && x >= imgRangeStartX && x < imgRangeEndX && y > imgRangeStartY && y < imgRangeEndY);
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
Vector2i ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::GetVoxelImgCoords(int x, int y) {
	return Vector2i(static_cast<int>(imgToVoxelScale * (x - imgRangeStartX)),
	                imgPixelRangeY - static_cast<int>(imgToVoxelScale * (y - imgRangeStartY)));
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
Vector2i ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::GetVoxelImgCoords(float x, float y) {
	return Vector2i(static_cast<int>(imgToVoxelScale * (x - imgRangeStartX)),
	                imgPixelRangeY - static_cast<int>(imgToVoxelScale * (y - imgRangeStartY)));
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::IsVoxelBlockInImgRange(Vector3i blockVoxelCoords) {
	Vector3i& bvc0 = blockVoxelCoords;
	Vector3i bvc1 = blockVoxelCoords + Vector3i(SDF_BLOCK_SIZE);
	return !(imgZSlice >= bvc1.z || imgZSlice < bvc0.z) &&
	       !(imgRangeStartX >= bvc1.x || imgRangeEndX < bvc0.x) &&
	       !(imgRangeStartY >= bvc1.y || imgRangeEndY < bvc0.y);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::IsVoxelBlockInImgRangeTolerance(Vector3i blockVoxelCoords, int tolerance) {
	Vector3i& bvc0 = blockVoxelCoords;
	Vector3i bvc1 = blockVoxelCoords + Vector3i(SDF_BLOCK_SIZE);
	return !(imgZSlice >= bvc1.z || imgZSlice < bvc0.z) &&
	       !(imgRangeStartX - tolerance >= bvc1.x || imgRangeEndX + tolerance < bvc0.x) &&
	       !(imgRangeStartY - tolerance >= bvc1.y || imgRangeEndY + tolerance < bvc0.y);
}


