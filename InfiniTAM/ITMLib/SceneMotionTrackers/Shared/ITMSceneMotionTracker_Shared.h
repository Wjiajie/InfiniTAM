//  ================================================================
//  Created by Gregory Kramida on 4/26/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "../../Utils/ITMVoxelFlags.h"
#include "../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../ORUtils/PlatformIndependence.h"
#include "../../Utils/ITMPrintHelpers.h"




//_DEBUG
// this seems to give more noise in the results (visually)
//template<typename TVoxelCanonical, typename TVoxelLive>
//inline
//bool VoxelIsConsideredForTracking(TVoxelCanonical& voxelCanonical, TVoxelLive voxelLive, int sourceFieldIndex){
//	return voxelLive.flag_values[sourceFieldIndex] == VOXEL_NONTRUNCATED;
//};

template<typename TVoxelCanonical, typename TVoxelLive>
_CPU_AND_GPU_CODE_ inline
bool
VoxelIsConsideredForTracking(const TVoxelCanonical& voxelCanonical, const TVoxelLive& voxelLive, int sourceFieldIndex) {
	return voxelCanonical.flags == ITMLib::VOXEL_NONTRUNCATED ||
	       voxelLive.flag_values[sourceFieldIndex] == ITMLib::VOXEL_NONTRUNCATED;
};

template<typename TVoxelCanonical, typename TVoxelLive>
_CPU_AND_GPU_CODE_ inline
bool
VoxelIsConsideredForDataTerm(const TVoxelCanonical& canonicalVoxel, const TVoxelLive& liveVoxel, int sourceSdfIndex) {
	//_DEBUG
	//Data condition: ALWAYS
	//return true;
	//Data condition: IGNORE_UNKNOWN
	//return canonicalVoxel.flags != ITMLib::VOXEL_UNKNOWN && liveVoxel.flag_values[sourceSdfIndex] != ITMLib::VOXEL_UNKNOWN;
	//Data condition: ONLY_NONTRUNCATED
//	return liveVoxel.flag_values[sourceSdfIndex] == ITMLib::VOXEL_NONTRUNCATED &&
//	       canonicalVoxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	//Data condition: IGNORE_CANONICAL_UNKNOWN
	//return canonicalVoxel.flags != ITMLib::VOXEL_UNKNOWN;
	//Data condition: MIXED01
	return canonicalVoxel.flags != ITMLib::VOXEL_UNKNOWN && liveVoxel.flag_values[sourceSdfIndex] == ITMLib::VOXEL_NONTRUNCATED;
};


// region =================================EXPLORATION OF NEIGHBORHOOD AROUND CANONICAL VOXEL===========================


/**
 * \brief Finds neighbor voxel's warps in the order specified below.
 *     0        1        2          3         4         5           6         7         8
 *	(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
 * \tparam TVoxel
 * \tparam TCache
 * \param[out] neighborFlowWarps
 * \param[out] neighborKnown - current behavior is:
 * 1) record unallocated voxels as non-found
 * 2) truncated voxels marked unknown or known as found
 * 3) everything else (non-truncated), of course, as found
 * \param[in] voxelPosition exact position of voxel in the scene.
 * \param[in] voxels
 * \param[in] indexData
 * \param[in] cache
 */
template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPoint2ndDerivativeNeighborhoodFlowWarp(THREADPTR(Vector3f)* neighborFlowWarps, //x9, out
                                                       THREADPTR(bool)* neighborKnown, //x9, out
                                                       THREADPTR(bool)* neighborTruncated, //x9, out
                                                       THREADPTR(bool)* neighborAllocated, //x9, out
                                                       const CONSTPTR(Vector3i)& voxelPosition,
                                                       const CONSTPTR(TVoxel)* voxels,
                                                       const CONSTPTR(TIndexData)* indexData,
                                                       THREADPTR(TCache)& cache) {
	int vmIndex = 0;

	TVoxel voxel;
	//TODO: define inline function instead of macro
#define PROCESS_VOXEL(location, index)\
    voxel = readVoxel(voxels, indexData, voxelPosition + (location), vmIndex, cache);\
    neighborFlowWarps[index] = voxel.flow_warp;\
    neighborAllocated[index] = vmIndex != 0;\
    neighborKnown[index] = voxel.flags != ITMLib::VOXEL_UNKNOWN;\
    neighborTruncated[index] = voxel.flags == ITMLib::VOXEL_TRUNCATED;

	//necessary for 2nd derivatives in same direction, e.g. xx and zz
	PROCESS_VOXEL(Vector3i(-1, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, -1, 0), 1);
	PROCESS_VOXEL(Vector3i(0, 0, -1), 2);

	//necessary for 1st derivatives
	PROCESS_VOXEL(Vector3i(1, 0, 0), 3);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 4);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 5);

	//necessary for 2nd derivatives in mixed directions, e.g. xy and yz
	PROCESS_VOXEL(Vector3i(1, 1, 0), 6);//xy corner
	PROCESS_VOXEL(Vector3i(0, 1, 1), 7);//yz corner
	PROCESS_VOXEL(Vector3i(1, 0, 1), 8);//xz corner
#undef PROCESS_VOXEL
}


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPoint2ndDerivativeNeighborhoodFlowWarp_DEBUG(
		THREADPTR(Vector3f)* neighborFlowWarps, //x9, out
		THREADPTR(bool)* neighborKnown, //x9, out
		THREADPTR(bool)* neighborTruncated, //x9, out
		THREADPTR(bool)* neighborAllocated, //x9, out
		const CONSTPTR(Vector3i)& voxelPosition,
		const CONSTPTR(TVoxel)* voxels,
		const CONSTPTR(TIndexData)* indexData,
		THREADPTR(TCache)& cache) {
	int vmIndex = 0;

	TVoxel voxel;
	//TODO: define inline function instead of macro
	voxel = readVoxel(voxels, indexData, voxelPosition + Vector3i(0, 0, 0), vmIndex, cache);
	neighborFlowWarps[0] = voxel.flow_warp;
	neighborAllocated[0] = vmIndex != 0;
	neighborKnown[0] = voxel.flags != ITMLib::VOXEL_UNKNOWN;
	neighborTruncated[0] = voxel.flags == ITMLib::VOXEL_TRUNCATED;
}


/**
 * \brief Finds neighbor voxel's warps in the order specified below.
 *     0        1        2          3         4         5           6         7         8
 *	(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
 * \tparam TVoxel
 * \tparam TCache
 * \param[out] neighborUpdates
 * \param[out] neighborKnown - current behavior is:
 * 1) record unallocated voxels as non-found
 * 2) truncated voxels marked unknown or known as found
 * 3) everything else (non-truncated), of course, as found
 * \param[in] voxelPosition exact position of voxel in the scene.
 * \param[in] voxels
 * \param[in] indexData
 * \param[in] cache
 */
template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPoint2ndDerivativeNeighborhoodPreviousUpdate(THREADPTR(Vector3f)* neighborUpdates, //x9, out
                                                             THREADPTR(bool)* neighborKnown, //x9, out
                                                             THREADPTR(bool)* neighborTruncated, //x9, out
                                                             THREADPTR(bool)* neighborAllocated, //x9, out
                                                             const CONSTPTR(Vector3i)& voxelPosition,
                                                             const CONSTPTR(TVoxel)* voxels,
                                                             const CONSTPTR(TIndexData)* indexData,
                                                             THREADPTR(TCache)& cache) {
	int vmIndex = 0;

	TVoxel voxel;
	//TODO: define inline function instead of macro
#define PROCESS_VOXEL(location, index)\
    voxel = readVoxel(voxels, indexData, voxelPosition + (location), vmIndex, cache);\
    neighborUpdates[index] = voxel.warp_update;\
    neighborAllocated[index] = vmIndex != 0;\
    neighborKnown[index] = voxel.flags != ITMLib::VOXEL_UNKNOWN;\
    neighborTruncated[index] = voxel.flags == ITMLib::VOXEL_TRUNCATED;

	//necessary for 2nd derivatives in same direction, e.g. xx and zz
	PROCESS_VOXEL(Vector3i(-1, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, -1, 0), 1);
	PROCESS_VOXEL(Vector3i(0, 0, -1), 2);

	//necessary for 1st derivatives
	PROCESS_VOXEL(Vector3i(1, 0, 0), 3);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 4);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 5);

	//necessary for 2nd derivatives in mixed directions, e.g. xy and yz
	PROCESS_VOXEL(Vector3i(1, 1, 0), 6);//xy corner
	PROCESS_VOXEL(Vector3i(0, 1, 1), 7);//yz corner
	PROCESS_VOXEL(Vector3i(1, 0, 1), 8);//xz corner
#undef PROCESS_VOXEL
}


/**
 * \brief Finds neighbor voxel's warps in the order specified below.
 *     0        1        2          3         4         5           6         7         8
 *	(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
 * \tparam TVoxel
 * \tparam TCache
 * \param[out] neighborWarps
 * \param[out] neighborKnown - current behavior is:
 * 1) record unallocated voxels as non-found
 * 2) truncated voxels marked unknown or known as found
 * 3) everything else (non-truncated), of course, as found
 * \param[in] voxelPosition exact position of voxel in the scene.
 * \param[in] voxels
 * \param[in] indexData
 * \param[in] cache
 */
template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPoint2ndDerivativeNeighborhoodWarp(THREADPTR(Vector3f)* neighborWarps, //x9, out
                                                   THREADPTR(bool)* neighborKnown, //x9, out
                                                   THREADPTR(bool)* neighborTruncated, //x9, out
                                                   THREADPTR(bool)* neighborAllocated, //x9, out
                                                   const CONSTPTR(Vector3i)& voxelPosition,
                                                   const CONSTPTR(TVoxel)* voxels,
                                                   const CONSTPTR(TIndexData)* indexData,
                                                   THREADPTR(TCache)& cache) {
	int vmIndex = 0;

	TVoxel voxel;
	//TODO: define inline function instead of macro
#define PROCESS_VOXEL(location, index)\
    voxel = readVoxel(voxels, indexData, voxelPosition + (location), vmIndex, cache);\
    neighborWarps[index] = voxel.warp;\
    neighborAllocated[index] = vmIndex != 0;\
    neighborKnown[index] = voxel.flags != ITMLib::VOXEL_UNKNOWN;\
    neighborTruncated[index] = voxel.flags == ITMLib::VOXEL_TRUNCATED;

	//necessary for 2nd derivatives in same direction, e.g. xx and zz
	PROCESS_VOXEL(Vector3i(-1, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, -1, 0), 1);
	PROCESS_VOXEL(Vector3i(0, 0, -1), 2);

	//necessary for 1st derivatives
	PROCESS_VOXEL(Vector3i(1, 0, 0), 3);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 4);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 5);

	//necessary for 2nd derivatives in mixed directions, e.g. xy and yz
	PROCESS_VOXEL(Vector3i(1, 1, 0), 6);//xy corner
	PROCESS_VOXEL(Vector3i(0, 1, 1), 7);//yz corner
	PROCESS_VOXEL(Vector3i(1, 0, 1), 8);//xz corner
#undef PROCESS_VOXEL
}

//endregion




//region ================================= SDF JACOBIAN ================================================================

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobianForwardDifferences(THREADPTR(Vector3f)& jacobian,
                                           const CONSTPTR(Vector3i)& position,
                                           const CONSTPTR(float)& sdfAtPosition,
                                           const CONSTPTR(TVoxel)* voxels,
                                           const CONSTPTR(TIndexData)* indexData,
                                           THREADPTR(TCache) cache) {
	int vmIndex = 0;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, indexData, position + (offset), vmIndex, cache).sdf))
	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));
#undef sdf_at

	jacobian[0] = sdfAtXplusOne - sdfAtPosition;
	jacobian[1] = sdfAtYplusOne - sdfAtPosition;
	jacobian[2] = sdfAtZplusOne - sdfAtPosition;
};

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_ForwardDifferences(THREADPTR(Vector3f)& jacobian,
                                            const CONSTPTR(Vector3i)& position,
                                            const CONSTPTR(TVoxel)* voxels,
                                            const CONSTPTR(TIndexData)* indexData,
                                            THREADPTR(TCache)& cache) {
	int vmIndex = 0;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, indexData, position + (offset), vmIndex, cache).sdf))
	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));

	float sdfAtPosition = sdf_at(Vector3i(0, 0, 0));
#undef sdf_at
	jacobian[0] = sdfAtXplusOne - sdfAtPosition;
	jacobian[1] = sdfAtYplusOne - sdfAtPosition;
	jacobian[2] = sdfAtZplusOne - sdfAtPosition;
};


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences(THREADPTR(Vector3f)& jacobian,
                                            const CONSTPTR(Vector3i)& voxelPosition,
                                            const CONSTPTR(TVoxel)* voxels,
                                            const CONSTPTR(TIndexData)* indexData,
                                            THREADPTR(TCache)& cache) {
	int vmIndex = 0;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, indexData, voxelPosition + (offset), vmIndex, cache).sdf))

	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));
	float sdfAtXminusOne = sdf_at(Vector3i(-1, 0, 0));
	float sdfAtYminusOne = sdf_at(Vector3i(0, -1, 0));
	float sdfAtZminusOne = sdf_at(Vector3i(0, 0, -1));

#undef sdf_at
	jacobian[0] = 0.5f * (sdfAtXplusOne - sdfAtXminusOne);
	jacobian[1] = 0.5f * (sdfAtYplusOne - sdfAtYminusOne);
	jacobian[2] = 0.5f * (sdfAtZplusOne - sdfAtZminusOne);
};

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences(Vector3f& jacobian,
                                            const Vector3f& voxelPosition,
                                            const TVoxel* voxels,
                                            const TIndexData* indexData,
                                            THREADPTR(TCache)& cache) {
	int vmIndex = 0;
#define sdf_at(offset) (_DEBUG_InterpolateTrilinearly(voxels, indexData, voxelPosition + (offset),  cache))

	float sdfAtXplusOne = sdf_at(Vector3f(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3f(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3f(0, 0, 1));
	float sdfAtXminusOne = sdf_at(Vector3f(-1, 0, 0));
	float sdfAtYminusOne = sdf_at(Vector3f(0, -1, 0));
	float sdfAtZminusOne = sdf_at(Vector3f(0, 0, -1));

#undef sdf_at
	jacobian[0] = 0.5f * (sdfAtXplusOne - sdfAtXminusOne);
	jacobian[1] = 0.5f * (sdfAtYplusOne - sdfAtYminusOne);
	jacobian[2] = 0.5f * (sdfAtZplusOne - sdfAtZminusOne);
};


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveGradient(
		THREADPTR(Vector3f)& jacobian,
		const CONSTPTR(Vector3i)& voxelPosition,
		const CONSTPTR(TVoxel)* voxels,
		const CONSTPTR(TIndexData)* indexData,
		THREADPTR(TCache)& cache,
		const CONSTPTR(int)& fieldIndex) {
	int vmIndex = 0;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, indexData, voxelPosition + (offset), vmIndex, cache).sdf_values[fieldIndex]))

	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));
	float sdfAtXminusOne = sdf_at(Vector3i(-1, 0, 0));
	float sdfAtYminusOne = sdf_at(Vector3i(0, -1, 0));
	float sdfAtZminusOne = sdf_at(Vector3i(0, 0, -1));

#undef sdf_at
	jacobian[0] = 0.5f * (sdfAtXplusOne - sdfAtXminusOne);
	jacobian[1] = 0.5f * (sdfAtYplusOne - sdfAtYminusOne);
	jacobian[2] = 0.5f * (sdfAtZplusOne - sdfAtZminusOne);
};

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeLiveGradient_CentralDifferences_IndexedFields(
		THREADPTR(Vector3f)& jacobian,
		const CONSTPTR(Vector3i)& voxelPosition,
		const CONSTPTR(TVoxel)* voxels,
		const CONSTPTR(TIndexData)* indexData,
		THREADPTR(TCache)& cache,
		const CONSTPTR(int)& fieldIndex) {
	int vmIndex = 0;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, indexData, voxelPosition + (offset), vmIndex, cache).sdf_values[fieldIndex]))

	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));
	float sdfAtXminusOne = sdf_at(Vector3i(-1, 0, 0));
	float sdfAtYminusOne = sdf_at(Vector3i(0, -1, 0));
	float sdfAtZminusOne = sdf_at(Vector3i(0, 0, -1));

#undef sdf_at
	jacobian[0] = 0.5f * (sdfAtXplusOne - sdfAtXminusOne);
	jacobian[1] = 0.5f * (sdfAtYplusOne - sdfAtYminusOne);
	jacobian[2] = 0.5f * (sdfAtZplusOne - sdfAtZminusOne);
};


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeLiveGradient_CentralDifferences_IndexedFields_AdvancedGrad(
		THREADPTR(Vector3f)& gradient,
		const CONSTPTR(Vector3i)& voxelPosition,
		const CONSTPTR(TVoxel)* voxels,
		const CONSTPTR(TIndexData)* indexData,
		THREADPTR(TCache)& cache,
		const CONSTPTR(int)& fieldIndex,
		const CONSTPTR(float)& currentSdf) {
	float thresh = 0.25;
	int vmIndex = 0;
	{
		TVoxel xPlusOne = readVoxel(voxels, indexData, voxelPosition + Vector3i(1, 0, 0), vmIndex, cache);
		float sdfAtXplusOne = TVoxel::valueToFloat(xPlusOne.sdf_values[fieldIndex]);
		TVoxel xMinusOne = readVoxel(voxels, indexData, voxelPosition + Vector3i(-1, 0, 0), vmIndex, cache);
		float sdfAtXminusOne = TVoxel::valueToFloat(xMinusOne.sdf_values[fieldIndex]);
		if (xPlusOne.flag_values[fieldIndex] != ITMLib::VOXEL_NONTRUNCATED || std::abs(currentSdf - sdfAtXplusOne) > thresh) {
			if (xMinusOne.flag_values[fieldIndex] != ITMLib::VOXEL_NONTRUNCATED || std::abs(currentSdf - sdfAtXminusOne) > thresh) {
				gradient.x = 0.0f;
			} else {
				gradient.x = currentSdf - sdfAtXminusOne;
			}
		} else if (xMinusOne.flag_values[fieldIndex] != ITMLib::VOXEL_NONTRUNCATED || std::abs(currentSdf - sdfAtXminusOne) > thresh) {
			gradient.x = sdfAtXplusOne - currentSdf;
		} else {
			gradient.x = 0.5f * (sdfAtXplusOne - sdfAtXminusOne);
		}
	}
	{
		TVoxel yPlusOne = readVoxel(voxels, indexData, voxelPosition + Vector3i(0, 1, 0), vmIndex, cache);
		float sdfAtYplusOne = TVoxel::valueToFloat(yPlusOne.sdf_values[fieldIndex]);
		TVoxel yMinusOne = readVoxel(voxels, indexData, voxelPosition + Vector3i(0, -1, 0), vmIndex, cache);
		float sdfAtYminusOne = TVoxel::valueToFloat(yMinusOne.sdf_values[fieldIndex]);
		if (yPlusOne.flag_values[fieldIndex] != ITMLib::VOXEL_NONTRUNCATED || std::abs(currentSdf - sdfAtYplusOne) > thresh) {
			if (yMinusOne.flag_values[fieldIndex] != ITMLib::VOXEL_NONTRUNCATED || std::abs(currentSdf - sdfAtYminusOne) > thresh) {
				gradient.y = 0.0f;
			} else {
				gradient.y = currentSdf - TVoxel::valueToFloat(yMinusOne.sdf_values[fieldIndex]);
			}
		} else if (yMinusOne.flag_values[fieldIndex] != ITMLib::VOXEL_NONTRUNCATED || std::abs(currentSdf - sdfAtYminusOne) > thresh) {
			gradient.y = TVoxel::valueToFloat(yPlusOne.sdf_values[fieldIndex]) - currentSdf;
		} else {
			gradient.y = 0.5f * (sdfAtYplusOne - sdfAtYminusOne);
		}
	}
	{
		TVoxel zPlusOne = readVoxel(voxels, indexData, voxelPosition + Vector3i(0, 0, 1), vmIndex, cache);
		float sdfAtZplusOne = TVoxel::valueToFloat(zPlusOne.sdf_values[fieldIndex]);
		TVoxel zMinusOne = readVoxel(voxels, indexData, voxelPosition + Vector3i(0, 0, -1), vmIndex, cache);
		float sdfAtZminusOne = TVoxel::valueToFloat(zMinusOne.sdf_values[fieldIndex]);
		if (zPlusOne.flag_values[fieldIndex] != ITMLib::VOXEL_NONTRUNCATED || std::abs(currentSdf - sdfAtZplusOne) > thresh) {
			if (zMinusOne.flag_values[fieldIndex] != ITMLib::VOXEL_NONTRUNCATED || std::abs(currentSdf - sdfAtZminusOne) > thresh) {
				gradient.z = 0.0f;
			} else {
				gradient.z = currentSdf - TVoxel::valueToFloat(zMinusOne.sdf_values[fieldIndex]);
			}
		} else if (zMinusOne.flag_values[fieldIndex] != ITMLib::VOXEL_NONTRUNCATED || std::abs(currentSdf - sdfAtZminusOne) > thresh) {
			gradient.z = TVoxel::valueToFloat(zPlusOne.sdf_values[fieldIndex]) - currentSdf;
		} else {
			gradient.z = 0.5f * (sdfAtZplusOne - sdfAtZminusOne);
		}
	}
};
// endregion

// region ================================= SDF HESSIAN ================================================================

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeSdfHessian(THREADPTR(Matrix3f)& hessian,
                              const CONSTPTR(Vector3i &) position,
                              const CONSTPTR(float)& sdfAtPosition,
		//const CONSTPTR(Vector3f&) jacobianAtPosition,
		                      const CONSTPTR(TVoxel)* voxels,
		                      const CONSTPTR(TIndexData)* indexData,
		                      THREADPTR(TCache)& cache) {
	int vmIndex = 0;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, indexData, position + (offset), vmIndex, cache).sdf))
	//for xx, yy, zz
	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));
	float sdfAtXminusOne = sdf_at(Vector3i(-1, 0, 0));
	float sdfAtYminusOne = sdf_at(Vector3i(0, -1, 0));
	float sdfAtZminusOne = sdf_at(Vector3i(0, 0, -1));

	//for xy, xz, yz
	float sdfAtXplusOneYplusOne = sdf_at(Vector3i(1, 1, 0));
	float sdfAtXminusOneYminusOne = sdf_at(Vector3i(-1, -1, 0));
	float sdfAtYplusOneZplusOne = sdf_at(Vector3i(0, 1, 1));
	float sdfAtYminusOneZminusOne = sdf_at(Vector3i(0, -1, -1));
	float sdfAtXplusOneZplusOne = sdf_at(Vector3i(1, 0, 1));
	float sdfAtXminusOneZminusOne = sdf_at(Vector3i(-1, 0, -1));
#undef sdf_at
	float delta_xx = sdfAtXplusOne - 2 * sdfAtPosition + sdfAtXminusOne;
	float delta_yy = sdfAtYplusOne - 2 * sdfAtPosition + sdfAtYminusOne;
	float delta_zz = sdfAtZplusOne - 2 * sdfAtPosition + sdfAtZminusOne;

	float delta_xy = 0.5f * (sdfAtXplusOneYplusOne - sdfAtXplusOne - sdfAtYplusOne
	                         + 2 * sdfAtPosition
	                         - sdfAtXminusOne - sdfAtYminusOne + sdfAtXminusOneYminusOne);

	float delta_yz = 0.5f * (sdfAtYplusOneZplusOne - sdfAtYplusOne - sdfAtZplusOne
	                         + 2 * sdfAtPosition
	                         - sdfAtYminusOne - sdfAtZminusOne + sdfAtYminusOneZminusOne);

	float delta_xz = 0.5f * (sdfAtXplusOneZplusOne - sdfAtXplusOne - sdfAtZplusOne
	                         + 2 * sdfAtPosition
	                         - sdfAtXminusOne - sdfAtZminusOne + sdfAtXminusOneZminusOne);

	float vals[9] = {delta_xx, delta_xy, delta_xz,
	                 delta_xy, delta_yy, delta_yz,
	                 delta_yz, delta_yz, delta_zz};

	hessian.setValues(vals);
};


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeSdfHessian_IndexedFields(THREADPTR(Matrix3f)& hessian,
                                            const CONSTPTR(Vector3i &) position,
                                            const CONSTPTR(float)& sdfAtPosition,
                                            const CONSTPTR(TVoxel)* voxels,
                                            const CONSTPTR(TIndexData)* indexData,
                                            THREADPTR(TCache)& cache,
                                            const CONSTPTR(int)& fieldIndex) {
	int vmIndex = 0;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, indexData, position + (offset), vmIndex, cache).sdf_values[fieldIndex]))
	//for xx, yy, zz
	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));
	float sdfAtXminusOne = sdf_at(Vector3i(-1, 0, 0));
	float sdfAtYminusOne = sdf_at(Vector3i(0, -1, 0));
	float sdfAtZminusOne = sdf_at(Vector3i(0, 0, -1));

	//for xy, xz, yz
	float sdfAtXplusOneYplusOne = sdf_at(Vector3i(1, 1, 0));
	float sdfAtXminusOneYminusOne = sdf_at(Vector3i(-1, -1, 0));
	float sdfAtYplusOneZplusOne = sdf_at(Vector3i(0, 1, 1));
	float sdfAtYminusOneZminusOne = sdf_at(Vector3i(0, -1, -1));
	float sdfAtXplusOneZplusOne = sdf_at(Vector3i(1, 0, 1));
	float sdfAtXminusOneZminusOne = sdf_at(Vector3i(-1, 0, -1));
#undef sdf_at
	float delta_xx = sdfAtXplusOne - 2 * sdfAtPosition + sdfAtXminusOne;
	float delta_yy = sdfAtYplusOne - 2 * sdfAtPosition + sdfAtYminusOne;
	float delta_zz = sdfAtZplusOne - 2 * sdfAtPosition + sdfAtZminusOne;

	float delta_xy = 0.5f * (sdfAtXplusOneYplusOne - sdfAtXplusOne - sdfAtYplusOne
	                         + 2 * sdfAtPosition
	                         - sdfAtXminusOne - sdfAtYminusOne + sdfAtXminusOneYminusOne);

	float delta_yz = 0.5f * (sdfAtYplusOneZplusOne - sdfAtYplusOne - sdfAtZplusOne
	                         + 2 * sdfAtPosition
	                         - sdfAtYminusOne - sdfAtZminusOne + sdfAtYminusOneZminusOne);

	float delta_xz = 0.5f * (sdfAtXplusOneZplusOne - sdfAtXplusOne - sdfAtZplusOne
	                         + 2 * sdfAtPosition
	                         - sdfAtXminusOne - sdfAtZminusOne + sdfAtXminusOneZminusOne);

	float vals[9] = {delta_xx, delta_xy, delta_xz,
	                 delta_xy, delta_yy, delta_yz,
	                 delta_yz, delta_yz, delta_zz};

	hessian.setValues(vals);
};


//endregion

// region ================================ WARP LAPLACIAN (SMOOTHING/TIKHONOV TERM) ====================================
_CPU_AND_GPU_CODE_
inline void ComputeWarpLaplacian(THREADPTR(Vector3f)& laplacian,
                                 const CONSTPTR(Vector3f)& voxelWarp,
                                 const CONSTPTR(Vector3f*) neighborWarps) {//in, x6-9
	//    0        1        2          3         4         5
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)
	laplacian = Vector3f(0.0f);
	//3D discrete Laplacian filter based on https://en.wikipedia.org/wiki/Discrete_Laplace_operator
	for (int iSixConnectedNeighbor = 0; iSixConnectedNeighbor < 6; iSixConnectedNeighbor++) {
		laplacian += neighborWarps[iSixConnectedNeighbor];
	}
	laplacian -= 6 * voxelWarp;
}

_CPU_AND_GPU_CODE_
inline void ComputeWarpLaplacianAndJacobian(THREADPTR(Vector3f)& laplacian,
                                            THREADPTR(Matrix3f)& jacobian,
                                            const CONSTPTR(Vector3f)& voxelWarp,
                                            const CONSTPTR(Vector3f*) neighborWarps) {//in, x6-9
	//    0        1        2          3         4         5
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)
	laplacian = Vector3f(0.0f);
	//3D discrete Laplacian filter based on https://en.wikipedia.org/wiki/Discrete_Laplace_operator
	for (int iSixConnectedNeighbor = 0; iSixConnectedNeighbor < 6; iSixConnectedNeighbor++) {
		laplacian += neighborWarps[iSixConnectedNeighbor];
	}
	laplacian -= 6 * voxelWarp;
	//use first-order differences for jacobian
	jacobian.setColumn(0, 0.5 * (neighborWarps[3] - neighborWarps[0]));//1st derivative in x
	jacobian.setColumn(1, 0.5 * (neighborWarps[4] - neighborWarps[1]));//1st derivative in y
	jacobian.setColumn(2, 0.5 * (neighborWarps[5] - neighborWarps[2]));//1st derivative in z
}

// endregion

// region =================================== WARP JACOBIAN AND HESSIAN (SMOOTHING/KILLING TERM) =======================
//Computes the jacobian and hessian approximation for the warp vectors themselves in a given neighborhood
_CPU_AND_GPU_CODE_
inline void ComputePerVoxelWarpJacobianAndHessian(const CONSTPTR(Vector3f)& voxelFlowWarp,
                                                  const CONSTPTR(Vector3f*) neighborFlowWarps, //in, x9
                                                  THREADPTR(Matrix3f)& jacobian, //out
                                                  THREADPTR(Matrix3f)* hessian //out, x3
) {
	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)

	// |u_x, u_y, u_z|       |m00, m10, m20|
	// |v_x, v_y, v_z|       |m01, m11, m21|
	// |w_x, w_y, w_z|       |m02, m12, m22|
	jacobian.setColumn(0, neighborFlowWarps[3] - voxelFlowWarp);//1st derivative in x
	jacobian.setColumn(1, neighborFlowWarps[4] - voxelFlowWarp);//1st derivative in y
	jacobian.setColumn(2, neighborFlowWarps[5] - voxelFlowWarp);//1st derivative in z

	Matrix3f backwardDifferences;
	// |u_x, u_y, u_z|
	// |v_x, v_y, v_z|
	// |w_x, w_y, w_z|
	backwardDifferences.setColumn(0, voxelFlowWarp - neighborFlowWarps[0]);//1st derivative in x
	backwardDifferences.setColumn(1, voxelFlowWarp - neighborFlowWarps[1]);//1st derivative in y
	backwardDifferences.setColumn(2, voxelFlowWarp - neighborFlowWarps[2]);//1st derivative in z

	//second derivatives in same direction
	// |u_xx, u_yy, u_zz|       |m00, m10, m20|
	// |v_xx, v_yy, v_zz|       |m01, m11, m21|
	// |w_xx, w_yy, w_zz|       |m02, m12, m22|
	Matrix3f dd_XX_YY_ZZ = jacobian - backwardDifferences;

	Matrix3f neighborDifferences;
	neighborDifferences.setColumn(0, neighborFlowWarps[6] - neighborFlowWarps[4]);//(0,1,0)->(1,1,0)
	neighborDifferences.setColumn(1, neighborFlowWarps[7] - neighborFlowWarps[5]);//(0,0,1)->(0,1,1)
	neighborDifferences.setColumn(2, neighborFlowWarps[8] - neighborFlowWarps[3]);//(1,0,0)->(1,0,1)

	//second derivatives in different directions
	// |u_xy, u_yz, u_zx|      |m00, m10, m20|
	// |v_xy, v_yz, v_zx|      |m01, m11, m21|
	// |w_xy, w_yz, w_zx|      |m02, m12, m22|
	Matrix3f dd_XY_YZ_ZX = neighborDifferences - jacobian;

	//NOTE: Hessian matrices are symmetric in this case
	// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
	// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
	// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
	float valsU[9] = {dd_XX_YY_ZZ.m00, dd_XY_YZ_ZX.m00, dd_XY_YZ_ZX.m20,
	                  dd_XY_YZ_ZX.m00, dd_XX_YY_ZZ.m10, dd_XY_YZ_ZX.m10,
	                  dd_XY_YZ_ZX.m20, dd_XY_YZ_ZX.m10, dd_XX_YY_ZZ.m20};
	hessian[0].setValues(valsU);

	// |0, 3, 6|     |m00, m10, m20|      |v_xx, v_xy, v_xz|
	// |1, 4, 7|     |m01, m11, m21|      |v_xy, v_yy, v_yz|
	// |2, 5, 8|     |m02, m12, m22|      |v_xz, v_yz, v_zz|
	float valsV[9] = {dd_XX_YY_ZZ.m01, dd_XY_YZ_ZX.m01, dd_XY_YZ_ZX.m21,
	                  dd_XY_YZ_ZX.m01, dd_XX_YY_ZZ.m11, dd_XY_YZ_ZX.m11,
	                  dd_XY_YZ_ZX.m21, dd_XY_YZ_ZX.m11, dd_XX_YY_ZZ.m21};
	hessian[1].setValues(valsV);

	// |0, 3, 6|     |m00, m10, m20|      |w_xx, w_xy, w_xz|
	// |1, 4, 7|     |m01, m11, m21|      |w_xy, w_yy, w_yz|
	// |2, 5, 8|     |m02, m12, m22|      |w_xz, w_yz, w_zz|
	float valsW[9] = {dd_XX_YY_ZZ.m02, dd_XY_YZ_ZX.m02, dd_XY_YZ_ZX.m22,
	                  dd_XY_YZ_ZX.m02, dd_XX_YY_ZZ.m12, dd_XY_YZ_ZX.m12,
	                  dd_XY_YZ_ZX.m22, dd_XY_YZ_ZX.m12, dd_XX_YY_ZZ.m22};
	hessian[2].setValues(valsW);

};

// endregion
// region =========================================== DATA TERM ========================================================

template<typename TVoxel, typename TIndexData, typename TCache>
inline void
_CPU_AND_GPU_CODE_
computeDataTermUpdateContribution(
		THREADPTR(Vector3f)& localDataEnergyGradient,
		THREADPTR(float)& localDataEnergy,
		THREADPTR(TCache)& liveCache,
		const CONSTPTR(Vector3i)& voxelPosition,
		const CONSTPTR(TVoxel)* liveVoxels,
		const CONSTPTR(TIndexData)* liveIndexData,
		const CONSTPTR(int)& sourceSdfIndex,
		const CONSTPTR(float)& liveSdf,
		const CONSTPTR(float)& canonicalSdf,
		const CONSTPTR(float)& sdfToVoxelScalingFactorSquared) {

	Vector3f liveSdfGradient;
	ComputeLiveGradient_CentralDifferences_IndexedFields_AdvancedGrad(
			liveSdfGradient, voxelPosition, liveVoxels, liveIndexData, liveCache, sourceSdfIndex, liveSdf);

	// Compute data term error / energy
	float sdfDifferenceBetweenLiveAndCanonical = liveSdf - canonicalSdf;
	// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ) - also denoted as - (φ_{proj}(Ψ)−φ_{model}) ∇φ_{proj}(Ψ)
	// φ_n(Ψ) = φ_n(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
	// φ_{global} = φ_{global}(x, y, z)
	localDataEnergyGradient = sdfDifferenceBetweenLiveAndCanonical * liveSdfGradient * sdfToVoxelScalingFactorSquared;
	localDataEnergy =
			0.5f * (sdfDifferenceBetweenLiveAndCanonical * sdfDifferenceBetweenLiveAndCanonical *
			        sdfToVoxelScalingFactorSquared);
}