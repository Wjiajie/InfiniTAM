//  ================================================================
//  Created by Gregory Kramida on 8/29/19.
//  Copyright (c) 2019 Gregory Kramida
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

namespace ITMLib {
//region ================================= AUXILIARY FUNCTIONS =========================================================

inline static void
ComputePositionFromLinearIndex_PlainVoxelArray(int& x, int& y, int& z, const ITMPlainVoxelArray::IndexData* indexData,
                                               int linearIndex) {

	z = linearIndex / (indexData->size.x * indexData->size.y);
	int tmp = linearIndex - z * indexData->size.x * indexData->size.y;
	y = tmp / indexData->size.x;
	x = tmp - y * indexData->size.x;
	x += indexData->offset.x;
	y += indexData->offset.y;
	z += indexData->offset.z;
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


// region ======================================= AUXILIARY FUNCTIONS ==================================================

inline bool HashBlockDoesNotIntersectBounds(const Vector3i& hashEntryMinPoint, const Vector3i& hashEntryMaxPoint,
                                            const Vector6i& bounds) {
	return hashEntryMaxPoint.x < bounds.min_x ||
	       hashEntryMaxPoint.y < bounds.min_y ||
	       hashEntryMaxPoint.z < bounds.min_z ||
	       hashEntryMinPoint.x >= bounds.max_x ||
	       hashEntryMinPoint.y >= bounds.max_y ||
	       hashEntryMinPoint.z >= bounds.max_z;
}

inline
Vector6i computeLocalBounds(const Vector3i& hashEntryMinPoint, const Vector3i& hashEntryMaxPoint,
                            const Vector6i& bounds) {
	return Vector6i(std::max(0, bounds.min_x - hashEntryMinPoint.x),
	                std::max(0, bounds.min_y - hashEntryMinPoint.y),
	                std::max(0, bounds.min_z - hashEntryMinPoint.z),
	                std::min(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE - (hashEntryMaxPoint.x - bounds.max_x)),
	                std::min(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE - (hashEntryMaxPoint.y - bounds.max_y)),
	                std::min(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE - (hashEntryMaxPoint.z - bounds.max_z)));
}

// endregion ===========================================================================================================


//endregion
} // namespace ITMLib