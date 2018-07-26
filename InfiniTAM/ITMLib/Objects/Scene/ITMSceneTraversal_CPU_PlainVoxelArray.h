//  ================================================================
//  Created by Gregory Kramida on 5/22/18.
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

#include <cassert>
#include "ITMScene.h"
#include "../../Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"

namespace ITMLib {

//======================================================================================================================
//                              TRAVERSAL METHODS FOR SCENES USING ITMPlainVoxelArray FOR INDEXING
//======================================================================================================================

// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================


inline static void
ComputePositionFromLinearIndex_PlainVoxelArray(int& x, int& y, int& z, const ITMPlainVoxelArray::IndexData* indexData,
                                               int linearIndex) {

	z = linearIndex / (indexData->size.x * indexData->size.y);
	int tmp = linearIndex - z * indexData->size.x * indexData->size.y;
	y = tmp / indexData->size.x;
	x = tmp - y * indexData->size.x;
	x += indexData->offset.x; y += indexData->offset.y; z += indexData->offset.z;
}

inline static Vector3i
ComputePositionVectorFromLinearIndex_PlainVoxelArray(const ITMPlainVoxelArray::IndexData* indexData,
                                                     int linearIndex) {
	int z = linearIndex / (indexData->size.x * indexData->size.y);
	int tmp = linearIndex - z * indexData->size.x * indexData->size.y;
	int y = tmp / indexData->size.x;
	int x = tmp - y * indexData->size.x;
	return {x + indexData->offset.x, y + indexData->offset.y, z + indexData->offset.z};
}



//======================================================================================================================
//                              TRAVERSAL METHODS FOR SCENES USING ITMPlainVoxelArray FOR INDEXING
//======================================================================================================================
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================

template<typename TFunctor, typename TVoxel>
inline
void VoxelTraversal(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, TFunctor& functor) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int voxelCount = scene->index.getVolumeSize().x * scene->index.getVolumeSize().y * scene->index.getVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
		TVoxel& voxel = voxels[linearIndex];
		functor(voxel);
	}
};


template<typename TFunctor, typename TVoxel>
inline
void VoxelPositionTraversal(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, TFunctor& functor) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int voxelCount = scene->index.getVolumeSize().x * scene->index.getVolumeSize().y * scene->index.getVolumeSize().z;
	const ITMPlainVoxelArray::IndexData* indexData = scene->index.getIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
		Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
		TVoxel& voxel = voxels[linearIndex];
		functor(voxel, voxelPosition);
	}
};


template<typename TFunctor, typename TVoxel>
inline void
VoxelTraversalWithinBounds(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, TFunctor& functor, Vector6i bounds) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int vmIndex = 0;
	const ITMPlainVoxelArray::IndexData* indexData = scene->index.getIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int z = bounds.min_z; z < bounds.max_z; z++) {
		for (int y = bounds.min_y; y < bounds.max_y; y++) {
			for (int x = bounds.min_x; x < bounds.max_x; x++) {
				int linearIndex = findVoxel(indexData, Vector3i(x, y, z), vmIndex);
				TVoxel& voxel = voxels[linearIndex];
				functor(voxel);
			}
		}
	}

};


template<typename TFunctor, typename TVoxel>
inline void
VoxelPositionTraversalWithinBounds(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, TFunctor& functor,
                                   Vector6i bounds) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int vmIndex = 0;
	const ITMPlainVoxelArray::IndexData* indexData = scene->index.getIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int z = bounds.min_z; z < bounds.max_z; z++) {
		for (int y = bounds.min_y; y < bounds.max_y; y++) {
			for (int x = bounds.min_x; x < bounds.max_x; x++) {
				Vector3i position(x, y, z);
				int linearIndex = findVoxel(indexData, Vector3i(x, y, z), vmIndex);
				TVoxel& voxel = voxels[linearIndex];
				functor(voxel, position);
			}
		}
	}
};

template<typename TFunctor, typename TVoxel>
inline void
VoxelPositionAndHashEntryTraversalWithinBounds(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, TFunctor& functor,
                                               Vector6i bounds) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int vmIndex = 0;
	const ITMPlainVoxelArray::IndexData* indexData = scene->index.getIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int z = bounds.min_z; z < bounds.max_z; z++) {
		for (int y = bounds.min_y; y < bounds.max_y; y++) {
			for (int x = bounds.min_x; x < bounds.max_x; x++) {
				Vector3i position(x, y, z);
				int linearIndex = findVoxel(indexData, Vector3i(x, y, z), vmIndex);
				TVoxel& voxel = voxels[linearIndex];
				functor(voxel, position);
			}
		}
	}
};

// endregion ===========================================================================================================

// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================

template<typename TStaticFunctor, typename TVoxel>
inline void StaticVoxelTraversal(ITMScene<TVoxel, ITMPlainVoxelArray>* scene) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int voxelCount = scene->index.getVolumeSize().x * scene->index.getVolumeSize().y * scene->index.getVolumeSize().z;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {

		TVoxel& voxel = voxels[linearIndex];
		TStaticFunctor::run(voxel);
	}
};

template<typename TStaticFunctor, typename TVoxel>
inline void StaticVoxelPositionTraversal(ITMScene<TVoxel, ITMPlainVoxelArray>* scene) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int voxelCount = scene->index.getVolumeSize().x * scene->index.getVolumeSize().y * scene->index.getVolumeSize().z;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
		Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(scene, linearIndex);
		TVoxel& voxel = voxels[linearIndex];
		TStaticFunctor::run(voxel, voxelPosition);
	}
};

// endregion
// region ================================ STATIC TWO-SCENE TRAVERSAL ==================================================

template<typename TStaticFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
inline void StaticDualVoxelTraversal(
		ITMScene<TVoxelPrimary, ITMPlainVoxelArray>* primaryScene,
		ITMScene<TVoxelSecondary, ITMPlainVoxelArray>* secondaryScene) {
	assert(primaryScene->index.getVolumeSize() == secondaryScene->index.getVolumeSize());
// *** traversal vars
	TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
	TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
	//asserted to be the same
	int voxelCount = primaryScene->index.getVolumeSize().x * primaryScene->index.getVolumeSize().y *
	                 primaryScene->index.getVolumeSize().z;


#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
		TVoxelPrimary& primaryVoxel = primaryScene[linearIndex];
		TVoxelSecondary& secondaryVoxel = secondaryScene[linearIndex];
		TStaticFunctor::run(primaryVoxel, secondaryVoxel);
	}
};
// endregion
// region ================================ DYNAMIC TWO-SCENE TRAVERSAL =================================================



template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
inline void DualVoxelTraversal(
		ITMScene<TVoxelPrimary, ITMPlainVoxelArray>* primaryScene,
		ITMScene<TVoxelSecondary, ITMPlainVoxelArray>* secondaryScene,
		TFunctor& functor) {

	assert(primaryScene->index.getVolumeSize() == secondaryScene->index.getVolumeSize());
// *** traversal vars
	TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
	TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
	//asserted to be the same
	int voxelCount = primaryScene->index.getVolumeSize().x * primaryScene->index.getVolumeSize().y *
	                 primaryScene->index.getVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
		TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
		TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
		functor(primaryVoxel, secondaryVoxel);
	}
};


template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
inline void DualVoxelPositionTraversal(
		ITMScene<TVoxelPrimary, ITMPlainVoxelArray>* primaryScene,
		ITMScene<TVoxelSecondary, ITMPlainVoxelArray>* secondaryScene,
		TFunctor& functor) {

	assert(primaryScene->index.getVolumeSize() == secondaryScene->index.getVolumeSize());
// *** traversal vars
	TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
	TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
	//asserted to be the same
	int voxelCount = primaryScene->index.getVolumeSize().x * primaryScene->index.getVolumeSize().y *
	                 primaryScene->index.getVolumeSize().z;
	const ITMPlainVoxelArray::IndexData* indexData = primaryScene->index.getIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
		Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
		TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
		TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
		functor(primaryVoxel, secondaryVoxel, voxelPosition);

	}
};


template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
inline void DualVoxelPositionTraversal_SingleThreaded(
		ITMScene<TVoxelPrimary, ITMPlainVoxelArray>* primaryScene,
		ITMScene<TVoxelSecondary, ITMPlainVoxelArray>* secondaryScene,
		TFunctor& functor) {
	assert(primaryScene->index.getVolumeSize() == secondaryScene->index.getVolumeSize());
	// *** traversal vars
	TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
	TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
	//asserted to be the same
	int voxelCount = primaryScene->index.getVolumeSize().x * primaryScene->index.getVolumeSize().y *
	                 primaryScene->index.getVolumeSize().z;
	const ITMPlainVoxelArray::IndexData* indexData = primaryScene->index.getIndexData();
	for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
		Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
		TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
		TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
		functor(primaryVoxel, secondaryVoxel, voxelPosition);
	}
}
// endregion ===========================================================================================================

}//namespace ITMLib