// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Utils/ITMPixelUtils.h"
#include "../../../Utils/ITMVoxelFlags.h"

/**
 * \brief Voxel update without confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param pt_model
 * \param M_d
 * \param projParams_d
 * \param mu
 * \param maxW
 * \param depth
 * \param imgSize
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(
		DEVICEPTR(TVoxel) &voxel,
		const THREADPTR(Vector4f) & pt_model,
		const CONSTPTR(Matrix4f) & M_d,
		const CONSTPTR(Vector4f) & projParams_d,
		float mu, int maxW,
		const CONSTPTR(float) *depth,
		const CONSTPTR(Vector2i) & imgSize)
{
	Vector4f pt_camera; Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW;

	// project point into image (voxel point in camera coordinates)
	pt_camera = M_d * pt_model;
	// if point is behind the camera, don't modify any voxels
	if (pt_camera.z <= 0) return -1;

	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

	// get measured depth from image
	depth_measure = depth[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x];
	if (depth_measure <= 0.0f) return -1;

	// check whether voxel needs updating
	// eta = [depth at pixel corresp. to current ray] - [depth of voxel point along the ray]
	// effectively, eta is the distance between measured surface & voxel point
	eta = depth_measure - pt_camera.z;

	//the voxel is beyond the narrow band, on the other side of the surface. Don't make any updates to SDF.
	if (eta < -mu) return eta;

	// compute updated SDF value and reliability
	oldF = TVoxel::valueToFloat(voxel.sdf); oldW = voxel.w_depth;

	newF = MIN(1.0f, eta / mu);
	newW = 1;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = MIN(newW, maxW);

	// write back
	voxel.sdf = TVoxel::floatToValue(newF);
	voxel.w_depth = newW;

	return eta;
}
/**
 * \brief Voxel update with confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param pt_model
 * \param M_d
 * \param projParams_d
 * \param mu
 * \param maxW
 * \param depth
 * \param confidence
 * \param imgSize
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(
		DEVICEPTR(TVoxel) &voxel,
		const THREADPTR(Vector4f) & pt_model,
		const CONSTPTR(Matrix4f) & M_d,
		const CONSTPTR(Vector4f) & projParams_d,
		float mu, int maxW,
		const CONSTPTR(float) *depth,
		const CONSTPTR(float) *confidence,
		const CONSTPTR(Vector2i) & imgSize)
{
	Vector4f pt_camera; Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW, locId;

	// project point into image
	pt_camera = M_d * pt_model;
	// if the point is behind the camera, don't make any changes to SDF and return -1 to short-circuit further updates
	if (pt_camera.z <= 0) return -1;

	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

	locId = (int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x;
	// get measured depth from image
	depth_measure = depth[locId];
	if (depth_measure <= 0.0) return -1;

	// check whether voxel needs updating
	eta = depth_measure - pt_camera.z;

	//the voxel is beyond the narrow band, on the other side of the surface. Don't make any updates to SDF.
	if (eta < -mu) return eta;

	// compute updated SDF value and reliability
	oldF = TVoxel::valueToFloat(voxel.sdf); oldW = voxel.w_depth;
	newF = MIN(1.0f, eta / mu); newW = 1;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = MIN(newW, maxW);

	// write back^
	voxel.sdf = TVoxel::floatToValue(newF);
	voxel.w_depth = newW;
	voxel.confidence += TVoxel::floatToValue(confidence[locId]);

	return eta;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void computeUpdatedVoxelColorInfo(
		DEVICEPTR(TVoxel) &voxel,
		const THREADPTR(Vector4f) & pt_model,
		const CONSTPTR(Matrix4f) & M_rgb,
		const CONSTPTR(Vector4f) & projParams_rgb,
		float mu, uchar maxW, float eta,
		const CONSTPTR(Vector4u) *rgb,
		const CONSTPTR(Vector2i) & imgSize)
{
	Vector4f pt_camera; Vector2f pt_image;
	Vector3f rgb_measure, oldC, newC; Vector3u buffV3u;
	float newW, oldW;

	buffV3u = voxel.clr;
	oldW = (float)voxel.w_color;

	oldC = TO_FLOAT3(buffV3u) / 255.0f;
	newC = oldC;

	pt_camera = M_rgb * pt_model;

	pt_image.x = projParams_rgb.x * pt_camera.x / pt_camera.z + projParams_rgb.z;
	pt_image.y = projParams_rgb.y * pt_camera.y / pt_camera.z + projParams_rgb.w;

	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return;

	rgb_measure = TO_VECTOR3(interpolateBilinear(rgb, pt_image, imgSize)) / 255.0f;
	//rgb_measure = rgb[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x].toVector3().toFloat() / 255.0f;
	newW = 1;

	newC = oldC * oldW + rgb_measure * newW;
	newW = oldW + newW;
	newC /= newW;
	newW = MIN(newW, maxW);

	voxel.clr = TO_UCHAR3(newC * 255.0f);
	voxel.w_color = (uchar)newW;
}

template<bool hasColor, bool hasConfidence, bool hasSemanticInformation, class TVoxel> struct ComputeUpdatedVoxelInfo;

//================= VOXEL UPDATES FOR VOXELS WITH NO SEMANTIC INFORMATION ==============================================
//arguments to the "compute" member function should always be the same
#define COMPUTE_VOXEL_UPDATE_PARAMETERS \
DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,\
const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,\
const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,\
float mu, int maxW,\
const CONSTPTR(float) *depth, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,\
const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb
//TODO: the magic value 0.25f used to determine the cutoff distance for color processing should be pre-defined either as a constant or a preprocessor define -Greg (GitHub:Algomorph)
#define COMPUTE_COLOR_CHECK if ((eta > mu) || (fabs(eta / mu) > 0.25f)) return;

// no color, no confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS)
	{
		computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
	}
};
// with color, no confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS)
	{
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
		COMPUTE_COLOR_CHECK
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};
// no color, with confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS)
	{
		computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence, imgSize_d);
	}
};
// with color, with confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS)
	{
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence, imgSize_d);
		COMPUTE_COLOR_CHECK
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};
//================= VOXEL UPDATES FOR VOXELS WITH SEMANTIC INFORMATION =================================================
// no color, no confidence, with semantic info
#define FLAG_UPDATE_CHECK \
	if (eta == -1.0f){\
		voxel.flags = ITMLib::VoxelFlags::VOXEL_UNKNOWN;\
        return; \
	} else if(std::abs(eta) > mu){/*assumes narrow band half-thickness mu is smaller than 1 meter*/\
        voxel.flags = ITMLib::VoxelFlags::VOXEL_TRUNCATED;\
        return; \
    } else {\
		voxel.flags = ITMLib::VoxelFlags::VOXEL_NONTRUNCATED;\
	}
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS)
	{
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
		FLAG_UPDATE_CHECK
	}
};
// with color, no confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS)
	{
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
		FLAG_UPDATE_CHECK
		COMPUTE_COLOR_CHECK
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};
// no color, with confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS)
	{
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence, imgSize_d);
		FLAG_UPDATE_CHECK
	}
};
// with color, with confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS)
	{
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence, imgSize_d);
		FLAG_UPDATE_CHECK
		COMPUTE_COLOR_CHECK
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};

#undef COMPUTE_COLOR_CHECK
#undef FLAG_UPDATE_CHECK
#undef COMPUTE_VOXEL_UPDATE_PARAMETERS
//======================================================================================================================
_CPU_AND_GPU_CODE_ inline void buildHashAllocAndVisibleTypePP(DEVICEPTR(uchar) *entriesAllocType, DEVICEPTR(uchar) *entriesVisibleType, int x, int y,
	DEVICEPTR(Vector3s) *blockCoords, const CONSTPTR(float) *depth, Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i imgSize,
	float oneOverVoxelSize, const CONSTPTR(ITMHashEntry) *hashTable, float viewFrustum_min, float viewFrustum_max)
{
	float depth_measure; unsigned int hashIdx; int noSteps;
	Vector4f pt_camera_f; Vector3f bandEndHashEntryPosition, bandStartHashEntryPosition, direction; Vector3s hashEntryPosition;

	depth_measure = depth[x + y * imgSize.x];
	if (depth_measure <= 0 || (depth_measure - mu) < 0 || (depth_measure - mu) < viewFrustum_min || (depth_measure + mu) > viewFrustum_max) return;

	pt_camera_f.z = depth_measure;
	pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams_d.z) * projParams_d.x);
	pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams_d.w) * projParams_d.y);

	float norm = sqrt(pt_camera_f.x * pt_camera_f.x + pt_camera_f.y * pt_camera_f.y + pt_camera_f.z * pt_camera_f.z);

	Vector4f pt_buff;
	
	pt_buff = pt_camera_f * (1.0f - mu / norm); pt_buff.w = 1.0f;
	bandStartHashEntryPosition = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;

	pt_buff = pt_camera_f * (1.0f + mu / norm); pt_buff.w = 1.0f;
	bandEndHashEntryPosition = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;

	// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
	// end of the (truncated SDF) band, along the ray cast from the camera through the point, in camera space
	direction = bandEndHashEntryPosition - bandStartHashEntryPosition;

	norm = sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
	// number of steps to take along the truncated SDF band
	noSteps = (int)ceil(2.0f*norm);

	// a single stride along the sdf band segment from one step to the next
	direction /= (float)(noSteps - 1);

	//add neighbouring blocks
	for (int i = 0; i < noSteps; i++)
	{
		//find block position at current step
		hashEntryPosition = TO_SHORT_FLOOR3(bandStartHashEntryPosition);

		//compute index in hash table
		hashIdx = hashIndex(hashEntryPosition);

		//check if hash table contains entry
		bool isFound = false;

		ITMHashEntry hashEntry = hashTable[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, hashEntryPosition) && hashEntry.ptr >= -1)
		{
			//entry has been streamed out but is visible or in memory and visible
			entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

			isFound = true;
		}

		if (!isFound)
		{
			bool isExcess = false;
			if (hashEntry.ptr >= -1) //search excess list only if there is no room in ordered part
			{
				while (hashEntry.offset >= 1)
				{
					hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
					hashEntry = hashTable[hashIdx];

					if (IS_EQUAL3(hashEntry.pos, hashEntryPosition) && hashEntry.ptr >= -1)
					{
						//entry has been streamed out but is visible or in memory and visible
						entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

						isFound = true;
						break;
					}
				}

				isExcess = true;
			}

			if (!isFound) //still not found
			{
				entriesAllocType[hashIdx] = isExcess ? 2 : 1; //needs allocation 
				if (!isExcess) entriesVisibleType[hashIdx] = 1; //new entry is visible
				blockCoords[hashIdx] = Vector4s(hashEntryPosition.x, hashEntryPosition.y, hashEntryPosition.z, 1);
			}
		}

		bandStartHashEntryPosition += direction;
	}
}

template<bool useSwapping>
_CPU_AND_GPU_CODE_ inline void checkPointVisibility(THREADPTR(bool) &isVisible, THREADPTR(bool) &isVisibleEnlarged,
	const THREADPTR(Vector4f) &pt_image, const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) &projParams_d,
	const CONSTPTR(Vector2i) &imgSize)
{
	Vector4f pt_buff;

	pt_buff = M_d * pt_image;

	if (pt_buff.z < 1e-10f) return;

	pt_buff.x = projParams_d.x * pt_buff.x / pt_buff.z + projParams_d.z;
	pt_buff.y = projParams_d.y * pt_buff.y / pt_buff.z + projParams_d.w;

	if (pt_buff.x >= 0 && pt_buff.x < imgSize.x && pt_buff.y >= 0 && pt_buff.y < imgSize.y) { isVisible = true; isVisibleEnlarged = true; }
	else if (useSwapping)
	{
		Vector4i lims;
		lims.x = -imgSize.x / 8; lims.y = imgSize.x + imgSize.x / 8;
		lims.z = -imgSize.y / 8; lims.w = imgSize.y + imgSize.y / 8;

		if (pt_buff.x >= lims.x && pt_buff.x < lims.y && pt_buff.y >= lims.z && pt_buff.y < lims.w) isVisibleEnlarged = true;
	}
}

template<bool useSwapping>
_CPU_AND_GPU_CODE_ inline void checkBlockVisibility(THREADPTR(bool) &isVisible, THREADPTR(bool) &isVisibleEnlarged,
	const THREADPTR(Vector3s) &hashPos, const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) &projParams_d,
	const CONSTPTR(float) &voxelSize, const CONSTPTR(Vector2i) &imgSize)
{
	Vector4f pt_image;
	float factor = (float)SDF_BLOCK_SIZE * voxelSize;

	isVisible = false; isVisibleEnlarged = false;

	// 0 0 0
	pt_image.x = (float)hashPos.x * factor; pt_image.y = (float)hashPos.y * factor;
	pt_image.z = (float)hashPos.z * factor; pt_image.w = 1.0f;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 0 0 1
	pt_image.z += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 0 1 1
	pt_image.y += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 1 1
	pt_image.x += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 1 0 
	pt_image.z -= factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 0 0 
	pt_image.y -= factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 0 1 0
	pt_image.x -= factor; pt_image.y += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 0 1
	pt_image.x += factor; pt_image.y -= factor; pt_image.z += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;
}
