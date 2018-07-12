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
#include "../../Engines/Manipulation/ITMSceneManipulation.h"

namespace ITMLib {

//======================================================================================================================
//                              TRAVERSAL METHODS FOR SCENES USING ITMPlainVoxelArray FOR INDEXING
//======================================================================================================================

// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================

template<typename TVoxel>
inline static int
ComputeLinearIndexFromPosition_PlainVoxelArray(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, Vector3i position) {
	int linearIndex = position.z * (scene->index.getVolumeSize().z * scene->index.getVolumeSize().y)
	                  + position.y * (scene->index.getVolumeSize().x + position.x);
	return linearIndex;
};

template<typename TVoxel>
inline static int
ComputeLinearIndexFromPosition_PlainVoxelArray(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, int x, int y, int z) {
	int linearIndex = z * (scene->index.getVolumeSize().z * scene->index.getVolumeSize().y)
	                  + y * (scene->index.getVolumeSize().x + x);
	return linearIndex;
};

template<typename TVoxel>
inline static void
ComputePositionFromLinearIndex_PlainVoxelArray(int& x, int& y, int& z, ITMScene<TVoxel, ITMPlainVoxelArray>* scene,
                                               int linearIndex) {
	z = linearIndex / (scene->index.getVolumeSize().x * scene->index.getVolumeSize().y);
	int tmp = linearIndex - z * scene->index.getVolumeSize().x * scene->index.getVolumeSize().y;
	y = tmp / scene->index.getVolumeSize().x;
	x = tmp - y * scene->index.getVolumeSize().x;
}

template<typename TVoxel>
inline static Vector3i
ComputePositionVectorFromLinearIndex_PlainVoxelArray(ITMScene<TVoxel, ITMPlainVoxelArray>* scene,
                                                     int linearIndex) {
	int z = linearIndex / (scene->index.getVolumeSize().x * scene->index.getVolumeSize().y);
	int tmp = linearIndex - z * scene->index.getVolumeSize().x * scene->index.getVolumeSize().y;
	int y = tmp / scene->index.getVolumeSize().x;
	int x = tmp - y * scene->index.getVolumeSize().x;
	return {x, y, z};
}



//======================================================================================================================
//                              TRAVERSAL METHODS FOR SCENES USING ITMPlainVoxelArray FOR INDEXING
//======================================================================================================================
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================

template<typename TFunctor, typename TVoxel>
inline
void VoxelTraversal_CPU(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, TFunctor& functor) {
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
void VoxelPositionTraversal_CPU(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, TFunctor& functor) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int voxelCount = scene->index.getVolumeSize().x * scene->index.getVolumeSize().y * scene->index.getVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
		Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(scene, linearIndex);
		TVoxel& voxel = voxels[linearIndex];
		functor(voxel, voxelPosition);
	}
};


template<typename TFunctor, typename TVoxel>
inline void
VoxelTraversalWithinBounds_CPU(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, TFunctor& functor, Vector6i bounds) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int voxelCount = scene->index.getVolumeSize().x * scene->index.getVolumeSize().y * scene->index.getVolumeSize().z;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int z = bounds.min_z; z < bounds.max_z; z++) {
		for (int y = bounds.min_y; y < bounds.max_y; y++) {
			for (int x = bounds.min_x; x < bounds.max_x; x++) {
				int linearIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(scene, x, y, z);
				TVoxel& voxel = voxels[linearIndex];
				functor(voxel);
			}
		}
	}

};


template<typename TFunctor, typename TVoxel>
inline void
VoxelPositionTraversalWithinBounds_CPU(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, TFunctor& functor,
                                       Vector6i bounds) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int voxelCount = scene->index.getVolumeSize().x * scene->index.getVolumeSize().y * scene->index.getVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int z = bounds.min_z; z < bounds.max_z; z++) {
		for (int y = bounds.min_y; y < bounds.max_y; y++) {
			for (int x = bounds.min_x; x < bounds.max_x; x++) {
				Vector3i position(x, y, z);
				int linearIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(scene, x, y, z);
				TVoxel& voxel = voxels[linearIndex];
				functor(voxel, position);
			}
		}
	}
};

template<typename TFunctor, typename TVoxel>
inline void
VoxelPositionAndHashEntryTraversalWithinBounds_CPU(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, TFunctor& functor,
                                       Vector6i bounds) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int voxelCount = scene->index.getVolumeSize().x * scene->index.getVolumeSize().y * scene->index.getVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int z = bounds.min_z; z < bounds.max_z; z++) {
		for (int y = bounds.min_y; y < bounds.max_y; y++) {
			for (int x = bounds.min_x; x < bounds.max_x; x++) {
				Vector3i position(x, y, z);
				int linearIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(scene, x, y, z);
				TVoxel& voxel = voxels[linearIndex];
				functor(voxel, position);
			}
		}
	}
};

// endregion ===========================================================================================================

// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================

template<typename TStaticFunctor, typename TVoxel>
inline void StaticVoxelTraversal_CPU(ITMScene<TVoxel, ITMPlainVoxelArray>* scene) {
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
inline void StaticVoxelPositionTraversal_CPU(ITMScene<TVoxel, ITMPlainVoxelArray>* scene) {
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
// region ================================ DYNAMIC TWO-SCENE TRAVERSAL =================================================
template<typename TStaticFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
inline void StaticDualVoxelTraversal_CPU(
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


template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
inline void DualVoxelTraversal_CPU(
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
inline void DualVoxelPositionTraversal_CPU(
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
		Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(primaryScene, linearIndex);
		TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
		TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
		functor(primaryVoxel, secondaryVoxel, voxelPosition);

	}
};


template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
inline void DualVoxelPositionTraversal_CPU_SingleThreaded(
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

	for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
		Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(primaryScene, linearIndex);
		TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
		TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
		functor(primaryVoxel, secondaryVoxel, voxelPosition);
	}
}
// endregion ===========================================================================================================

}//namespace ITMLib