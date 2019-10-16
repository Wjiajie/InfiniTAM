//  ================================================================
//  Created by Gregory Kramida on 11/5/17.
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

#include "ITMSceneManipulationEngine_CPU.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../Traversal/Shared/ITMSceneTraversal_Shared.h"
#include "../Shared/ITMSceneManipulationEngine_Shared.h"
#include "../Shared/ITMSceneManipulationEngine_Functors.h"


namespace ITMLib {


// region ==================================== Voxel Array Scene Manipulation Engine ===================================

template<typename TVoxel>
void ITMLib::ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::ResetScene(
		ITMLib::ITMVoxelVolume<TVoxel, ITMLib::ITMPlainVoxelArray>* scene) {
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
	int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
	scene->localVBA.lastFreeBlockId = numBlocks - 1;
}

template<typename TVoxel>
bool
ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::SetVoxel(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene,
                                                                     Vector3i at, TVoxel voxel) {
	int vmIndex = 0;
	int arrayIndex = findVoxel(scene->index.getIndexData(), at, vmIndex);
	if (vmIndex) {
		scene->localVBA.GetVoxelBlocks()[arrayIndex] = voxel;
		return true;
	} else {
		return false;
	}

}

template<typename TVoxel>
TVoxel
ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::ReadVoxel(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene,
                                                                      Vector3i at) {
	int vmIndex = 0;
	int arrayIndex = findVoxel(scene->index.getIndexData(), at, vmIndex);
	if (arrayIndex < 0) {
		TVoxel voxel;
		return voxel;
	}
	return scene->localVBA.GetVoxelBlocks()[arrayIndex];
}

template<typename TVoxel>
TVoxel
ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::ReadVoxel(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene,
                                                                      Vector3i at,
                                                                      ITMPlainVoxelArray::IndexCache& cache) {
	int vmIndex = 0;
	int arrayIndex = findVoxel(scene->index.getIndexData(), at, vmIndex);
	if (arrayIndex < 0) {
		TVoxel voxel;
		return voxel;
	}
	return scene->localVBA.GetVoxelBlocks()[arrayIndex];
}

template<typename TVoxel>
bool ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::
IsPointInBounds(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const Vector3i& point) {
	ITMLib::ITMPlainVoxelArray::IndexData* index_bounds = scene->index.getIndexData();
	Vector3i point2 = point - index_bounds->offset;
	return !((point2.x < 0) || (point2.x >= index_bounds->size.x) ||
	         (point2.y < 0) || (point2.y >= index_bounds->size.y) ||
	         (point2.z < 0) || (point2.z >= index_bounds->size.z));
}


template<typename TVoxel>
void
ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::OffsetWarps(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene,
		Vector3f offset) {
	OffsetWarpsFunctor<TVoxel, ITMPlainVoxelArray, TVoxel::hasCumulativeWarp>::OffsetWarps(scene, offset);
}


template<typename TVoxel>
bool ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::CopySceneSlice(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* destination, ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* source,
		Vector6i bounds, const Vector3i& offset) {

	Vector3i min_pt_source = Vector3i(bounds.min_x, bounds.min_y, bounds.min_z);
	Vector3i max_pt_source = Vector3i(bounds.max_x - 1, bounds.max_y - 1, bounds.max_z - 1);

	if (!ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::IsPointInBounds(source, min_pt_source) ||
	    !ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::IsPointInBounds(source, max_pt_source)) {
		DIEWITHEXCEPTION_REPORTLOCATION(
				"Specified source volume is at least partially out of bounds of the source scene.");
	}

	Vector6i bounds_destination;

	bounds_destination.min_x = bounds.min_x + offset.x;
	bounds_destination.max_x = bounds.max_x + offset.x;
	bounds_destination.min_y = bounds.min_y + offset.y;
	bounds_destination.max_y = bounds.max_y + offset.y;
	bounds_destination.min_z = bounds.min_z + offset.z;
	bounds_destination.max_z = bounds.max_z + offset.z;

	Vector3i min_pt_destination = Vector3i(bounds_destination.min_x, bounds_destination.min_y,
	                                       bounds_destination.min_z);
	Vector3i max_pt_destination = Vector3i(bounds_destination.max_x - 1, bounds_destination.max_y - 1,
	                                       bounds_destination.max_z - 1);

	if (!ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::IsPointInBounds(destination, min_pt_destination) ||
	    !ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::IsPointInBounds(destination, max_pt_destination)) {
		DIEWITHEXCEPTION_REPORTLOCATION(
				"Targeted volume is at least partially out of bounds of the destination scene.");
	}
	TVoxel* sourceVoxels = source->localVBA.GetVoxelBlocks();
	TVoxel* destinationVoxels = destination->localVBA.GetVoxelBlocks();
	if (offset == Vector3i(0)) {
		const ITMPlainVoxelArray::IndexData* sourceIndexData = source->index.getIndexData();
		const ITMPlainVoxelArray::IndexData* destinationIndexData = destination->index.getIndexData();
		for (int source_z = bounds.min_z; source_z < bounds.max_z; source_z++) {
			for (int source_y = bounds.min_y; source_y < bounds.max_y; source_y++) {
				for (int source_x = bounds.min_x; source_x < bounds.max_x; source_x++) {
					int vmIndex = 0;
					int linearSourceIndex = findVoxel(sourceIndexData, Vector3i(source_x, source_y, source_z), vmIndex);
					int linearDestinationIndex = findVoxel(destinationIndexData, Vector3i(source_x, source_y, source_z), vmIndex);
					memcpy(&destinationVoxels[linearDestinationIndex], &sourceVoxels[linearSourceIndex], sizeof(TVoxel));
				}
			}
		}
	} else {
		const ITMPlainVoxelArray::IndexData* sourceIndexData = source->index.getIndexData();
		const ITMPlainVoxelArray::IndexData* destinationIndexData = destination->index.getIndexData();
		for (int source_z = bounds.min_z; source_z < bounds.max_z; source_z++) {
			for (int source_y = bounds.min_y; source_y < bounds.max_y; source_y++) {
				for (int source_x = bounds.min_x; source_x < bounds.max_x; source_x++) {
					int vmIndex = 0;
					int destination_z = source_z + offset.z, destination_y = source_y + offset.y, destination_x =
							source_x + offset.x;
					int linearSourceIndex = findVoxel(sourceIndexData, Vector3i(source_x, source_y, source_z), vmIndex);
					int linearDestinationIndex = findVoxel(destinationIndexData,
					                                       Vector3i(destination_x, destination_y, destination_z),
					                                       vmIndex);
					memcpy(&destinationVoxels[linearDestinationIndex], &sourceVoxels[linearSourceIndex],
					       sizeof(TVoxel));
				}
			}
		}
	}
	return true;
}

template<typename TVoxel>
inline static Vector6i GetSceneBounds(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* source) {
	ITMPlainVoxelArray::IndexData* indexData = source->index.getIndexData();
	return {indexData->offset.x, indexData->offset.y, indexData->offset.z,
	        indexData->offset.x + indexData->size.x,
	        indexData->offset.y + indexData->size.y,
	        indexData->offset.z + indexData->size.z};
}


template<typename TVoxel>
bool ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::CopyScene(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* destination,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* source,
		const Vector3i& offset) {
	ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::ResetScene(destination);
	//TODO: this bounds treatment isn't quite correct -- it assumes same bounds for source & dest. Need to fix.
	Vector6i bounds = GetSceneBounds(source);
	if (offset.x > 0) {
		bounds.max_x -= offset.x;
	} else {
		bounds.min_x -= offset.x;
	}
	if (offset.y > 0) {
		bounds.max_y -= offset.y;
	} else {
		bounds.min_y -= offset.y;
	}
	if (offset.z > 0) {
		bounds.max_z -= offset.z;
	} else {
		bounds.min_z -= offset.z;
	}
	return ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::CopySceneSlice(destination, source, bounds,
	                                                                                  offset);
}

//endregion ============================================================================================================


}//namespace ITMLib