// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Utils/ITMPixelUtils.h"
#include "../../../Utils/ITMVoxelFlags.h"


template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void updateSdfAndFlagsBasedOnDistanceSurfaceToVoxel(
		DEVICEPTR(TVoxel)& voxel, float signedDistanceSurfaceToVoxelAlongCameraRay, float narrowBandHalfWidth){
	if (signedDistanceSurfaceToVoxelAlongCameraRay < -narrowBandHalfWidth) {
		//the voxel is beyond the narrow band, on the other side of the surface. Set SDF to -1.0
		voxel.sdf = TVoxel::floatToValue(-1.0);
		voxel.flags = ITMLib::VOXEL_TRUNCATED;
	} else if (signedDistanceSurfaceToVoxelAlongCameraRay > narrowBandHalfWidth) {
		//the voxel is in front of the narrow band, between the surface and the camera. Set SDF to 1.0
		voxel.sdf = TVoxel::floatToValue(-1.0);
		voxel.flags = ITMLib::VOXEL_TRUNCATED;
	} else {
		// The voxel lies within the narrow band, between truncation boundaries.
		// Update SDF in proportion to the distance from surface.
		voxel.sdf = TVoxel::floatToValue(signedDistanceSurfaceToVoxelAlongCameraRay / narrowBandHalfWidth);
		voxel.flags = ITMLib::VOXEL_NONTRUNCATED;
	}
}

/**
 * \brief Voxel update without confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param voxelInSceneCoordinates
 * \param depthCameraSceneMatrix
 * \param depthCameraProjectionParameters
 * \param narrowBandHalfWidth
 * \param depthImage an array of float depths corresponding to the depth image
 * \param imageSize
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedLiveVoxelDepthInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& voxelInSceneCoordinates,
		const CONSTPTR(Matrix4f)& depthCameraSceneMatrix,
		const CONSTPTR(Vector4f)& depthCameraProjectionParameters,
		float narrowBandHalfWidth,
		const CONSTPTR(float)* depthImage,
		const CONSTPTR(Vector2i)& imageSize) {

	// project point into image (voxel point in camera coordinates)
	Vector4f voxelPointInCameraCoordinates = depthCameraSceneMatrix * voxelInSceneCoordinates;
	// if point is behind the camera, don't modify any voxels
	if (voxelPointInCameraCoordinates.z <= 0) {
		return -1.0f;
	}

	Vector2f voxelPointProjectedToImage;
	voxelPointProjectedToImage.x = depthCameraProjectionParameters.x * voxelPointInCameraCoordinates.x
	                               / voxelPointInCameraCoordinates.z + depthCameraProjectionParameters.z;
	voxelPointProjectedToImage.y = depthCameraProjectionParameters.y * voxelPointInCameraCoordinates.y
	                               / voxelPointInCameraCoordinates.z + depthCameraProjectionParameters.w;

	// point falls outside of the image bounds
	if ((voxelPointProjectedToImage.x < 1) || (voxelPointProjectedToImage.x > imageSize.x - 2)
	    || (voxelPointProjectedToImage.y < 1) || (voxelPointProjectedToImage.y > imageSize.y - 2)) {
		return -1.0f;
	}

	// get measured depthImage from image
	float depthMeasure = depthImage[static_cast<int>(voxelPointProjectedToImage.x + 0.5f) +
	                                static_cast<int>(voxelPointProjectedToImage.y + 0.5f) * imageSize.x];
	// if depthImage is "invalid", return "unknown"
	if (depthMeasure <= 0.0f) {
		//keep voxel flags at ITMLib::VOXEL_UNKNOWN
		return -1.0f;
	}

	// signedDistanceSurfaceToVoxelAlongCameraRay (i.e. eta) =
	// [distance from surface to camera, i.e. depthImage] - [distance from voxel to camera]
	// effectively, eta is the distance between measured surface & voxel point
	float signedDistanceSurfaceToVoxelAlongCameraRay = depthMeasure - voxelPointInCameraCoordinates.z;
	updateSdfAndFlagsBasedOnDistanceSurfaceToVoxel(voxel, signedDistanceSurfaceToVoxelAlongCameraRay,
	                                               narrowBandHalfWidth);
	return signedDistanceSurfaceToVoxelAlongCameraRay;
}

/**
 * \brief Voxel update with confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param voxelInSceneCoordinates
 * \param depthCameraSceneMatrix
 * \param depthCameraProjectionParameters
 * \param narrowBandHalfWidth
 * \param maxW
 * \param depthImage
 * \param confidencesAtPixels
 * \param imageSize
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedLiveVoxelDepthInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& voxelInSceneCoordinates,
		const CONSTPTR(Matrix4f)& depthCameraSceneMatrix,
		const CONSTPTR(Vector4f)& depthCameraProjectionParameters,
		float narrowBandHalfWidth,
		const CONSTPTR(float)* depthImage,
		const CONSTPTR(float)* confidencesAtPixels,
		const CONSTPTR(Vector2i)& imageSize) {


	// project point into image
	Vector4f voxelPointInCameraCoordinates = depthCameraSceneMatrix * voxelInSceneCoordinates;
	// if the point is behind the camera, don't make any changes to SDF and return -1 to short-circuit further updates
	if (voxelPointInCameraCoordinates.z <= 0) return -1;

	Vector2f voxelPointProjectedToImage;
	voxelPointProjectedToImage.x =
			depthCameraProjectionParameters.x * voxelPointInCameraCoordinates.x /
			voxelPointInCameraCoordinates.z + depthCameraProjectionParameters.z;
	voxelPointProjectedToImage.y =
			depthCameraProjectionParameters.y * voxelPointInCameraCoordinates.y /
			voxelPointInCameraCoordinates.z + depthCameraProjectionParameters.w;
	if ((voxelPointProjectedToImage.x < 1) || (voxelPointProjectedToImage.x > imageSize.x - 2) ||
	    (voxelPointProjectedToImage.y < 1) || (voxelPointProjectedToImage.y > imageSize.y - 2))
		return -1;

	// point falls outside of the image bounds
	int pixelIndex =
			static_cast<int>(voxelPointProjectedToImage.x + 0.5f) +
			static_cast<int>(voxelPointProjectedToImage.y + 0.5f) * imageSize.x;

	// get measured depth from image
	float depthMeasure = depthImage[pixelIndex];
	if (depthMeasure <= 0.0) return -1;

	// signedSurfaceToVoxelAlongCameraRay (i.e. eta) =
	// [distance from surface to camera, i.e. depthImage] - [distance from voxel to camera]
	// effectively, eta is the distance between measured surface & voxel point
	float signedSurfaceToVoxelAlongCameraRay = depthMeasure - voxelPointInCameraCoordinates.z;
	voxel.confidence = TVoxel::floatToValue(confidencesAtPixels[pixelIndex]);

	updateSdfAndFlagsBasedOnDistanceSurfaceToVoxel(voxel, signedSurfaceToVoxelAlongCameraRay, narrowBandHalfWidth);
	return signedSurfaceToVoxelAlongCameraRay;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void computeUpdatedLiveVoxelColorInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& voxelInSceneCoordinates,
		const CONSTPTR(Matrix4f)& rgbCameraSceneMatrix,
		const CONSTPTR(Vector4f)& rgbCameraProjectionParameters,
		float narrowBandHalfWidth, float signedDistanceSurfaceToVoxel,
		const CONSTPTR(Vector4u)* rgbImage,
		const CONSTPTR(Vector2i)& imageSize) {
	Vector4f voxelInCameraCoordinates;
	Vector2f voxelPointProjectedToImage;

	voxelInCameraCoordinates = rgbCameraSceneMatrix * voxelInSceneCoordinates;

	voxelPointProjectedToImage.x =
			rgbCameraProjectionParameters.x * voxelInCameraCoordinates.x
			/ voxelInCameraCoordinates.z + rgbCameraProjectionParameters.z;
	voxelPointProjectedToImage.y =
			rgbCameraProjectionParameters.y * voxelInCameraCoordinates.y
			/ voxelInCameraCoordinates.z + rgbCameraProjectionParameters.w;
//TODO: the magic value 0.25f used to determine the cutoff distance for color processing should be pre-defined as a parameter -Greg (GitHub:Algomorph)
	//cut off voxels that are too far from the surface
	if(std::abs(signedDistanceSurfaceToVoxel) < 0.25f * narrowBandHalfWidth) return;

	if ((voxelPointProjectedToImage.x < 1) || (voxelPointProjectedToImage.x > imageSize.x - 2) ||
	    (voxelPointProjectedToImage.y < 1) || (voxelPointProjectedToImage.y > imageSize.y - 2))
		return;

	voxel.clr = TO_UCHAR3(TO_VECTOR3(interpolateBilinear(rgbImage, voxelPointProjectedToImage, imageSize)));
}

template<bool hasColor, bool hasConfidence, bool hasSemanticInformation, typename TVoxel>
struct ComputeUpdatedLiveVoxelInfo;

//================= VOXEL UPDATES FOR VOXELS WITH NO SEMANTIC INFORMATION ==============================================
//arguments to the "compute" member function should always be the same
#define COMPUTE_VOXEL_UPDATE_PARAMETERS \
DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,\
const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,\
const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,\
float mu, int maxW,\
const CONSTPTR(float) *depth, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,\
const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb


// no color, no confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<false, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, imgSize_d);
	}
};
// with color, no confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<true, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, imgSize_d);
		computeUpdatedLiveVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, eta, rgb, imgSize_rgb);
	}
};
// no color, with confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<false, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, confidence, imgSize_d);
	}
};
// with color, with confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<true, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, confidence,
		                                             imgSize_d);
		computeUpdatedLiveVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, eta, rgb, imgSize_rgb);
	}
};
//================= VOXEL UPDATES FOR VOXELS WITH SEMANTIC INFORMATION =================================================

template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<false, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, imgSize_d);
	}
};
// with color, no confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<true, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, imgSize_d);
		computeUpdatedLiveVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, eta, rgb, imgSize_rgb);
	}
};
// no color, with confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<false, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, confidence,
		                                             imgSize_d);
	}
};
// with color, with confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<true, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, confidence,
		                                             imgSize_d);
		computeUpdatedLiveVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, eta, rgb, imgSize_rgb);
	}
};


#undef COMPUTE_VOXEL_UPDATE_PARAMETERS
//======================================================================================================================
_CPU_AND_GPU_CODE_ inline void
buildLiveHashAllocAndVisibleTypePP(DEVICEPTR(uchar)* entriesAllocType, DEVICEPTR(uchar)* entriesVisibleType, int x,
                                   int y,
                                   DEVICEPTR(Vector4s)* blockCoords, const CONSTPTR(float)* depth, Matrix4f invM_d,
                                   Vector4f projParams_d, float mu, Vector2i imgSize,
                                   float oneOverVoxelSize, const CONSTPTR(ITMHashEntry)* hashTable,
                                   float viewFrustum_min, float viewFrustum_max) {
	float depth_measure;
	int hashIdx;
	int noSteps;
	Vector4f voxelInCameraCoordinates_f;
	Vector3f bandEndHashEntryPosition, bandStartHashEntryPosition, direction;
	Vector3s hashEntryPosition;

	depth_measure = depth[x + y * imgSize.x];
	if (depth_measure <= 0 || (depth_measure - mu) < 0 || (depth_measure - mu) < viewFrustum_min ||
	    (depth_measure + mu) > viewFrustum_max)
		return;

	voxelInCameraCoordinates_f.z = depth_measure;
	voxelInCameraCoordinates_f.x = voxelInCameraCoordinates_f.z * ((float(x) - projParams_d.z) * projParams_d.x);
	voxelInCameraCoordinates_f.y = voxelInCameraCoordinates_f.z * ((float(y) - projParams_d.w) * projParams_d.y);

	float norm = sqrtf(voxelInCameraCoordinates_f.x * voxelInCameraCoordinates_f.x +
	                  voxelInCameraCoordinates_f.y * voxelInCameraCoordinates_f.y +
	                  voxelInCameraCoordinates_f.z * voxelInCameraCoordinates_f.z);

	Vector4f pt_buff;

	pt_buff = voxelInCameraCoordinates_f * (1.0f - mu / norm);
	pt_buff.w = 1.0f;
	bandStartHashEntryPosition = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;

	pt_buff = voxelInCameraCoordinates_f * (1.0f + mu / norm);
	pt_buff.w = 1.0f;
	bandEndHashEntryPosition = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;

	// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
	// end of the (truncated SDF) band, along the ray cast from the camera through the point, in camera space
	direction = bandEndHashEntryPosition - bandStartHashEntryPosition;

	norm = sqrtf(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
	// number of steps to take along the truncated SDF band
	noSteps = (int) ceilf(2.0f * norm);

	// a single stride along the sdf band segment from one step to the next
	direction /= (float) (noSteps - 1);

	//add neighbouring blocks
	for (int i = 0; i < noSteps; i++) {
		//find block position at current step
		hashEntryPosition = TO_SHORT_FLOOR3(bandStartHashEntryPosition);
		//compute index in hash table
		hashIdx = hashIndex(hashEntryPosition);
		//check if hash table contains entry
		bool isFound = false;
		ITMHashEntry hashEntry = hashTable[hashIdx];
		if (IS_EQUAL3(hashEntry.pos, hashEntryPosition) && hashEntry.ptr >= -1) {
			//entry has been streamed out but is visible or in memory and visible
			entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? (uchar)2 : (uchar)1;
			isFound = true;
		}
		if (!isFound) {
			bool isExcess = false;
			if (hashEntry.ptr >= -1){ //search excess list only if there is no room in ordered part
				while (hashEntry.offset >= 1) {
					hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
					hashEntry = hashTable[hashIdx];
					if (IS_EQUAL3(hashEntry.pos, hashEntryPosition) && hashEntry.ptr >= -1) {
						//entry has been streamed out but is visible or in memory and visible
						entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? (uchar)2 : (uchar)1;
						isFound = true;
						break;
					}
				}
				isExcess = true;
			}
			if (!isFound){ //still not found
				entriesAllocType[hashIdx] = isExcess ? (uchar)2 : (uchar)1; //needs allocation
				if (!isExcess) entriesVisibleType[hashIdx] = 1; //new entry is visible
				blockCoords[hashIdx] = Vector4s(hashEntryPosition.x, hashEntryPosition.y, hashEntryPosition.z, 1);
			}
		}
		bandStartHashEntryPosition += direction;
	}
}

template<bool useSwapping>
_CPU_AND_GPU_CODE_ inline void checkLivePointVisibility(THREADPTR(bool)& isVisible, THREADPTR(bool)& isVisibleEnlarged,
                                                        const THREADPTR(Vector4f)& voxelPointProjectedToImage,
                                                        const CONSTPTR(Matrix4f)& M_d,
                                                        const CONSTPTR(Vector4f)& projParams_d,
                                                        const CONSTPTR(Vector2i)& imgSize) {
	Vector4f pt_buff;

	pt_buff = M_d * voxelPointProjectedToImage;

	if (pt_buff.z < 1e-10f) return;

	pt_buff.x = projParams_d.x * pt_buff.x / pt_buff.z + projParams_d.z;
	pt_buff.y = projParams_d.y * pt_buff.y / pt_buff.z + projParams_d.w;

	if (pt_buff.x >= 0 && pt_buff.x < imgSize.x && pt_buff.y >= 0 && pt_buff.y < imgSize.y) {
		isVisible = true;
		isVisibleEnlarged = true;
	}
	else if (useSwapping) {
		Vector4i lims;
		lims.x = -imgSize.x / 8;
		lims.y = imgSize.x + imgSize.x / 8;
		lims.z = -imgSize.y / 8;
		lims.w = imgSize.y + imgSize.y / 8;

		if (pt_buff.x >= lims.x && pt_buff.x < lims.y && pt_buff.y >= lims.z && pt_buff.y < lims.w)
			isVisibleEnlarged = true;
	}
}

template<bool useSwapping>
_CPU_AND_GPU_CODE_ inline void checkLiveBlockVisibility(THREADPTR(bool)& isVisible, THREADPTR(bool)& isVisibleEnlarged,
                                                        const THREADPTR(Vector3s)& hashPos,
                                                        const CONSTPTR(Matrix4f)& M_d,
                                                        const CONSTPTR(Vector4f)& projParams_d,
                                                        const CONSTPTR(float)& voxelSize,
                                                        const CONSTPTR(Vector2i)& imgSize) {
	Vector4f voxelPointProjectedToImage;
	float factor = (float) SDF_BLOCK_SIZE * voxelSize;

	isVisible = false;
	isVisibleEnlarged = false;

	// 0 0 0
	voxelPointProjectedToImage.x = (float) hashPos.x * factor;
	voxelPointProjectedToImage.y = (float) hashPos.y * factor;
	voxelPointProjectedToImage.z = (float) hashPos.z * factor;
	voxelPointProjectedToImage.w = 1.0f;
	checkLivePointVisibility<useSwapping>(isVisible, isVisibleEnlarged, voxelPointProjectedToImage, M_d, projParams_d,
	                                      imgSize);
	if (isVisible) return;

	// 0 0 1
	voxelPointProjectedToImage.z += factor;
	checkLivePointVisibility<useSwapping>(isVisible, isVisibleEnlarged, voxelPointProjectedToImage, M_d, projParams_d,
	                                      imgSize);
	if (isVisible) return;

	// 0 1 1
	voxelPointProjectedToImage.y += factor;
	checkLivePointVisibility<useSwapping>(isVisible, isVisibleEnlarged, voxelPointProjectedToImage, M_d, projParams_d,
	                                      imgSize);
	if (isVisible) return;

	// 1 1 1
	voxelPointProjectedToImage.x += factor;
	checkLivePointVisibility<useSwapping>(isVisible, isVisibleEnlarged, voxelPointProjectedToImage, M_d, projParams_d,
	                                      imgSize);
	if (isVisible) return;

	// 1 1 0 
	voxelPointProjectedToImage.z -= factor;
	checkLivePointVisibility<useSwapping>(isVisible, isVisibleEnlarged, voxelPointProjectedToImage, M_d, projParams_d,
	                                      imgSize);
	if (isVisible) return;

	// 1 0 0 
	voxelPointProjectedToImage.y -= factor;
	checkLivePointVisibility<useSwapping>(isVisible, isVisibleEnlarged, voxelPointProjectedToImage, M_d, projParams_d,
	                                      imgSize);
	if (isVisible) return;

	// 0 1 0
	voxelPointProjectedToImage.x -= factor;
	voxelPointProjectedToImage.y += factor;
	checkLivePointVisibility<useSwapping>(isVisible, isVisibleEnlarged, voxelPointProjectedToImage, M_d, projParams_d,
	                                      imgSize);
	if (isVisible) return;

	// 1 0 1
	voxelPointProjectedToImage.x += factor;
	voxelPointProjectedToImage.y -= factor;
	voxelPointProjectedToImage.z += factor;
	checkLivePointVisibility<useSwapping>(isVisible, isVisibleEnlarged, voxelPointProjectedToImage, M_d, projParams_d,
	                                      imgSize);
	if (isVisible) return;
}
