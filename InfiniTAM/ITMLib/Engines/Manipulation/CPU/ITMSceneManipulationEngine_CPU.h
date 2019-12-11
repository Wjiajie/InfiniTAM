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
#pragma once

#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../ITMLibDefines.h"
#include "../../../Utils/ITMHashBlockProperties.h"
#include "../../../Utils/ITMPrintHelpers.h"
#include "../Interface/ITMSceneManipulationEngine.h"


//TODO: Make CUDA versions -Greg (GitHub: Algomorph)

namespace ITMLib {

// region ==================== SCENE MANIPULATION ENGINE ===============================================================

template<typename TVoxel, typename TIndex>
class ITMSceneManipulationEngine_CPU : public ITMSceneManipulationEngine<TVoxel, TIndex> {
};


template<typename TVoxel>
class ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>
		: public ITMSceneManipulationEngine<TVoxel, ITMVoxelBlockHash> {
public:
	//can be used as a singleton, but doesn't HAVE TO be
	static ITMSceneManipulationEngine_CPU& Inst() {
		static ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash> instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	void ResetScene(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene) override;
	bool SetVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* volume, Vector3i at, TVoxel voxel) override;
	bool SetVoxelNoAllocation(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at, TVoxel voxel);
	TVoxel ReadVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at) override;
	TVoxel ReadVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at,
	                 ITMVoxelBlockHash::IndexCache& cache) override;
	void OffsetWarps(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3f offset) override;
	bool
	CopySceneSlice(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* destination,
	               ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* source,
	               Vector6i bounds, const Vector3i& offset = Vector3i(0)) override;
	bool CopyScene(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* destination,
	               ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* source,
	               const Vector3i& offset = Vector3i(0)) override;
	virtual ~ITMSceneManipulationEngine_CPU() = default;
};

template<typename TVoxel>
class ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray> :
		public ITMSceneManipulationEngine<TVoxel, ITMPlainVoxelArray> {
public:
	// can use this as singleton, but don't have to
	static ITMSceneManipulationEngine_CPU& Inst() {
		static ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray> instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	void ResetScene(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene) override;
	bool SetVoxel(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, Vector3i at, TVoxel voxel) override;
	TVoxel ReadVoxel(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, Vector3i at) override;
	TVoxel ReadVoxel(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, Vector3i at,
	                 ITMPlainVoxelArray::IndexCache& cache) override;
	bool IsPointInBounds(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const Vector3i& at);
	void OffsetWarps(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, Vector3f offset) override;
	bool CopySceneSlice(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* destination,
	                    ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* source,
	                    Vector6i bounds, const Vector3i& offset = Vector3i(0)) override;
	bool CopyScene(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* destination,
	               ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* source,
	               const Vector3i& offset = Vector3i(0)) override;

};

// endregion ================= SCENE MANIPULATION ENGINE ===============================================================

typedef ITMSceneManipulationEngine_CPU<ITMVoxel, ITMPlainVoxelArray> ManipulationEngine_CPU_PVA_Voxel;
typedef ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash> ManipulationEngine_CPU_VBH_Voxel;
typedef ITMSceneManipulationEngine_CPU<ITMWarp, ITMPlainVoxelArray> ManipulationEngine_CPU_PVA_Warp;
typedef ITMSceneManipulationEngine_CPU<ITMWarp, ITMVoxelBlockHash> ManipulationEngine_CPU_VBH_Warp;

// region ======================================== HELPER RANGE COMPUTATION / CHECK ROUTINES ===========================
// =====================================================================================================================
inline
void BoundsFromExtrema(Vector6i& bounds, const Vector3i& extremum1, const Vector3i& extremum2) {
	// ** set min/max **
	if (extremum1.x > extremum2.x) {
		bounds.min_x = extremum2.x;
		bounds.max_x = extremum1.x;
	} else {
		bounds.min_x = extremum1.x;
		bounds.max_x = extremum2.x;
	}
	if (extremum1.y > extremum2.y) {
		bounds.min_y = extremum2.y;
		bounds.max_y = extremum1.y;
	} else {
		bounds.min_y = extremum1.y;
		bounds.max_y = extremum2.y;
	}
	if (extremum1.z > extremum2.z) {
		bounds.min_z = extremum2.z;
		bounds.max_z = extremum1.z;
	} else {
		bounds.min_z = extremum1.z;
		bounds.max_z = extremum2.z;
	}
}

inline
void
ComputeCopyRanges(int& xRangeStart, int& xRangeEnd, int& yRangeStart, int& yRangeEnd, int& zRangeStart, int& zRangeEnd,
                  const Vector3i& hashBlockPositionVoxels, const Vector6i& bounds) {
	zRangeStart = MAX(0, bounds.min_z - hashBlockPositionVoxels.z);
	zRangeEnd = MIN(VOXEL_BLOCK_SIZE, bounds.max_z - hashBlockPositionVoxels.z + 1);
	yRangeStart = MAX(0, bounds.min_y - hashBlockPositionVoxels.y);
	yRangeEnd = MIN(VOXEL_BLOCK_SIZE, bounds.max_y - hashBlockPositionVoxels.y + 1);
	xRangeStart = MAX(0, bounds.min_x - hashBlockPositionVoxels.x);
	xRangeEnd = MIN(VOXEL_BLOCK_SIZE, bounds.max_x - hashBlockPositionVoxels.x + 1);
}

// endregion ===========================================================================================================

}//namespace ITMLib