//  ================================================================
//  Created by Gregory Kramida on 1/5/18.
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
#include "ITMSceneStatisticsCalculator.h"
#include "../ITMMath.h"
#include "../../Objects/Scene/ITMScene.h"
#include "../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../Objects/Scene/ITMSceneTraversal_VoxelBlockHash.h"

using namespace ITMLib;

/**
 * \brief Computes minimum and maximum point in the current scene.
 * \tparam TVoxel
 * \tparam TIndex
 * \param scene
 * \param minVoxelPoint
 * \param maxVoxelPoint
 */
template<typename TVoxel, typename TIndex>
void ITMSceneStatisticsCalculator<TVoxel, TIndex>::ComputeVoxelBounds(const ITMScene<TVoxel, TIndex>* scene,
                                                                      Vector3i& minVoxelPoint,
                                                                      Vector3i& maxVoxelPoint) {

	minVoxelPoint = maxVoxelPoint = Vector3i(0);
	const TVoxel* voxelBlocks = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;

	//TODO: if OpenMP standard is 3.1 or above, use OpenMP parallel for reduction clause with (max:maxVoxelPointX,...) -Greg (GitHub: Algomorph)
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {

		const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		Vector3i currentHashBlockPositionVoxels = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		Vector3i hashBlockLimitPositionVoxels = (currentHashEntry.pos.toInt() + Vector3i(1, 1, 1)) * SDF_BLOCK_SIZE;

		if (minVoxelPoint.x > currentHashBlockPositionVoxels.x) {
			minVoxelPoint.x = currentHashBlockPositionVoxels.x;
		}
		if (maxVoxelPoint.x < hashBlockLimitPositionVoxels.x) {
			maxVoxelPoint.x = hashBlockLimitPositionVoxels.x;
		}
		if (minVoxelPoint.y > currentHashBlockPositionVoxels.y) {
			minVoxelPoint.y = currentHashBlockPositionVoxels.y;
		}
		if (maxVoxelPoint.y < hashBlockLimitPositionVoxels.y) {
			maxVoxelPoint.y = hashBlockLimitPositionVoxels.y;
		}
		if (minVoxelPoint.z > currentHashBlockPositionVoxels.z) {
			minVoxelPoint.z = currentHashBlockPositionVoxels.z;
		}
		if (maxVoxelPoint.z < hashBlockLimitPositionVoxels.z) {
			maxVoxelPoint.z = hashBlockLimitPositionVoxels.z;
		}
	}
}

//============================================== COUNT VOXELS ==========================================================
template<typename TVoxel, typename TIndex>
int ITMSceneStatisticsCalculator<TVoxel, TIndex>::ComputeAllocatedVoxelCount(ITMScene<TVoxel, TIndex>* scene) {
	int count = 0;

	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
#ifdef WITH_OPENMP
#pragma omp parallel for reduction(+:count)
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];
		if (currentHashEntry.ptr < 0) continue;
		count += SDF_BLOCK_SIZE3;
	}
	return count;
}

template<bool hasSemanticInformation, typename TVoxel, typename TIndex>
struct ComputeNonTruncatedVoxelCountFunctor;

template<class TVoxel, typename TIndex>
struct ComputeNonTruncatedVoxelCountFunctor<false, TVoxel, TIndex> {
	static int compute(ITMScene<TVoxel, TIndex>* scene) {
		DIEWITHEXCEPTION("Voxels need to have semantic information to be marked as truncated or non-truncated.");
	}
};
template<class TVoxel, typename TIndex>
struct ComputeNonTruncatedVoxelCountFunctor<true, TVoxel, TIndex> {
	static int compute(ITMScene<TVoxel, TIndex>* scene) {
		ComputeNonTruncatedVoxelCountFunctor instance;
		VoxelTraversal_CPU(scene, instance);
		return instance.count;
	}

	int count = 0;

	void operator()(TVoxel& voxel) {
		count += voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	}
};


template<typename TVoxel, typename TIndex>
int ITMSceneStatisticsCalculator<TVoxel, TIndex>::ComputeNonTruncatedVoxelCount(ITMScene<TVoxel, TIndex>* scene) {
	return ComputeNonTruncatedVoxelCountFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex>::compute(scene);
};
//================================================== COUNT VOXELS WITH SPECIFIC VALUE ==================================

template<class TVoxel, typename TIndex>
struct ComputeVoxelWithValueCountFunctor {
	static int compute(ITMScene<TVoxel, TIndex>* scene, float value) {
		ComputeVoxelWithValueCountFunctor instance;
		instance.value = value;
		VoxelTraversal_CPU(scene, instance);
		return instance.count;
	}

	int count = 0;
	float value = 0.0f;

	void operator()(TVoxel& voxel) {
		count += (voxel.sdf == value);
	}
};


template<typename TVoxel, typename TIndex>
int
ITMSceneStatisticsCalculator<TVoxel, TIndex>::ComputeVoxelWithValueCount(ITMScene<TVoxel, TIndex>* scene, float value) {
	return ComputeVoxelWithValueCountFunctor<TVoxel, TIndex>::compute(scene, value);
};
//================================================== SUM OF TOTAL SDF ==================================================

template<bool hasSemanticInformation, typename TVoxel, typename TIndex>
struct SumSDFFunctor;

template<class TVoxel, typename TIndex>
struct SumSDFFunctor<false, TVoxel, TIndex> {
	static double compute(ITMScene<TVoxel, TIndex>* scene) {
		DIEWITHEXCEPTION_REPORTLOCATION(
				"Voxels need to have semantic information to be marked as truncated or non-truncated.");
	}
};
template<class TVoxel, typename TIndex>
struct SumSDFFunctor<true, TVoxel, TIndex> {
	static double compute(ITMScene<TVoxel, TIndex>* scene, ITMLib::VoxelFlags voxelType) {
		SumSDFFunctor instance;
		instance.voxelType = voxelType;
		VoxelTraversal_CPU(scene, instance);
		return instance.sum;
	}

	double sum;
	ITMLib::VoxelFlags voxelType;

	void operator()(TVoxel& voxel) {
		if (voxelType == (ITMLib::VoxelFlags) voxel.flags) {
			sum += std::abs(static_cast<double>(TVoxel::valueToFloat(voxel.sdf)));
		}
	}
};


template<typename TVoxel, typename TIndex>
double
ITMSceneStatisticsCalculator<TVoxel, TIndex>::ComputeNonTruncatedVoxelAbsSdfSum(ITMScene<TVoxel, TIndex>* scene) {
	return SumSDFFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex>::compute(scene,
	                                                                              VoxelFlags::VOXEL_NONTRUNCATED);

};

template<typename TVoxel, typename TIndex>
double ITMSceneStatisticsCalculator<TVoxel, TIndex>::ComputeTruncatedVoxelAbsSdfSum(ITMScene<TVoxel, TIndex>* scene) {
	return SumSDFFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex>::compute(scene, VoxelFlags::VOXEL_TRUNCATED);
};

//======================================================================================================================

template<typename TVoxel, typename TIndex>
std::vector<int> ITMSceneStatisticsCalculator<TVoxel, TIndex>::GetFilledHashBlockIds(ITMScene<TVoxel, TIndex>* scene) {
	std::vector<int> ids;
	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];
		if (currentHashEntry.ptr < 0) continue;
		ids.push_back(entryId);
	}
	return ids;
}

template<typename TVoxel, typename TIndex>
int ITMSceneStatisticsCalculator<TVoxel, TIndex>::ComputeAllocatedHashBlockCount(ITMScene<TVoxel, TIndex>* scene) {
	int count;
	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];
		if (currentHashEntry.ptr >= 0) count++;
	}
	return count;
}
// region ================================ VOXEL GRADIENTS =============================================================

template<typename TVoxel, typename TIndex>
struct MaxGradientFunctor;

template<typename TIndex>
struct MaxGradientFunctor<ITMVoxel, TIndex> {
	static float find(ITMScene<ITMVoxel, TIndex>* scene, bool secondGradientField, Vector3i& maxPosition) {
		DIEWITHEXCEPTION_REPORTLOCATION("Not implemented for voxels without gradients.");
	}
};

template<typename TIndex>
struct MaxGradientFunctor<ITMVoxelLive, TIndex> {
	static float find(ITMScene<ITMVoxelLive, TIndex>* scene, bool secondGradientField, Vector3i& maxPosition) {
		DIEWITHEXCEPTION_REPORTLOCATION("Not implemented for voxels without gradients.");
	}
};

template<typename TIndex>
struct MaxGradientFunctor<ITMVoxelCanonical, TIndex> {
	static float find(ITMScene<ITMVoxelCanonical, TIndex>* scene, bool secondGradientField, Vector3i& maxPosition) {
		MaxGradientFunctor instance;
		instance.secondGradientField = secondGradientField;
		VoxelPositionTraversal_CPU(scene, instance);
		maxPosition = instance.maxPosition;
		return instance.maxLength;
	}

	float maxLength = 0.0f;
	bool secondGradientField;
	Vector3i maxPosition = Vector3i(0);

	void operator()(ITMVoxelCanonical& voxel, Vector3i position) {
		float gradientLength;
		if (secondGradientField) {
			gradientLength = ORUtils::length(voxel.gradient1);
		}else{
			gradientLength = ORUtils::length(voxel.gradient0);
		}
		if(gradientLength > maxLength){
			maxLength = gradientLength;
			maxPosition = position;
		}
	}
};

template<typename TVoxel, typename TIndex>
float ITMSceneStatisticsCalculator<TVoxel, TIndex>::FindMaxGradient1LengthAndPosition(ITMScene<TVoxel, TIndex>* scene, Vector3i& positionOut) {
	return MaxGradientFunctor<TVoxel, TIndex>::find(scene, true, positionOut);
}

template<typename TVoxel, typename TIndex>
float ITMSceneStatisticsCalculator<TVoxel, TIndex>::FindMaxGradient0LengthAndPosition(ITMScene<TVoxel, TIndex>* scene,
                                                                                      Vector3i& positionOut) {
	return MaxGradientFunctor<TVoxel, TIndex>::find(scene, false, positionOut);
}

template<typename TVoxel, typename TIndex>
void ITMSceneStatisticsCalculator<TVoxel, TIndex>::ComputeVoxelBounds(const ITMScene<TVoxel, TIndex>* scene,
                                                                      Vector6i& bounds) {
	Vector3i minPoint, maxPoint;
	this->ComputeVoxelBounds(scene, minPoint,maxPoint);
	bounds.min_x = minPoint.x;bounds.min_y = minPoint.y;bounds.min_z = minPoint.z;
	bounds.max_x = maxPoint.x;bounds.max_y = maxPoint.y;bounds.max_z = maxPoint.z;
}
// endregion ===========================================================================================================

