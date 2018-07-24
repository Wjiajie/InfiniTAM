//  ================================================================
//  Created by Gregory Kramida on 7/12/18.
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

#include "../../../Objects/Scene/ITMTrilinearInterpolation.h"

template<typename TVoxelMulti>
struct IndexedFieldClearFunctor {
	IndexedFieldClearFunctor(int flagFieldIndex) : flagFieldIndex(flagFieldIndex) {}

	void operator()(TVoxelMulti& voxel) {
		voxel.flag_values[flagFieldIndex] = ITMLib::VOXEL_UNKNOWN;
		voxel.sdf_values[flagFieldIndex] = TVoxelMulti::SDF_initialValue();
	}

private:
	const int flagFieldIndex;
};


template <typename TVoxelLive, typename TVoxelCanonical>
struct FusionFunctor{
	FusionFunctor(int maximumWeight, int liveSourceFieldIndex) :
			maximumWeight(maximumWeight),
			liveSourceFieldIndex(liveSourceFieldIndex){}
	void operator()(TVoxelLive& liveVoxel, TVoxelCanonical& canonicalVoxel){
		fuseLiveVoxelIntoCanonical(liveVoxel,liveSourceFieldIndex,maximumWeight,canonicalVoxel);
	}
private:
	const int maximumWeight;
	const int liveSourceFieldIndex;
};


template<typename TVoxelWarpSource, typename TVoxelSdf, typename TIndex, typename TLookupPositionFunctor>
struct TrilinearInterpolationFunctor {
	/**
	 * \brief Initialize to transfer data from source sdf scene to a target sdf scene using the warps in the warp source scene
	 * \details traverses
	 * \param sdfSourceScene
	 * \param warpSourceScene
	 */
	TrilinearInterpolationFunctor(ITMScene<TVoxelSdf, TIndex>* sdfSourceScene,
	                              ITMScene<TVoxelWarpSource, TIndex>* warpSourceScene, int sourceSdfIndex,
	                              int targetSdfIndex) :

			sdfSourceScene(sdfSourceScene),
			sdfSourceVoxels(sdfSourceScene->localVBA.GetVoxelBlocks()),
			sdfSourceIndexData(sdfSourceScene->index.getIndexData()),
			sdfSourceCache(),

			warpSourceScene(warpSourceScene),
			warpSourceVoxels(warpSourceScene->localVBA.GetVoxelBlocks()),
			warpSourceHashEntries(warpSourceScene->index.getIndexData()),
			warpSourceCache(),
			sourceSdfIndex(sourceSdfIndex),
			targetSdfIndex(targetSdfIndex),
			hasFocusCoordinates(ITMLibSettings::Instance().FocusCoordinatesAreSpecified()),
			focusCoordinates(ITMLibSettings::Instance().GetFocusCoordinates())
	{}


	void operator()(TVoxelSdf& destinationVoxel,TVoxelWarpSource& warpSourceVoxel,
	                Vector3i warpAndDestinationVoxelPosition) {
		int vmIndex;

		Vector3f warpedPosition =
				TLookupPositionFunctor::GetWarpedPosition(warpSourceVoxel, warpAndDestinationVoxelPosition);

		bool struckKnown;

		float sdf = _DEBUG_InterpolateMultiSdfTrilinearly_StruckKnown(
				sdfSourceVoxels, sdfSourceIndexData, warpedPosition, sourceSdfIndex, sdfSourceCache, struckKnown,
				hasFocusCoordinates && warpAndDestinationVoxelPosition == focusCoordinates);

		// Update flags
		if (struckKnown) {
			destinationVoxel.sdf_values[targetSdfIndex] = TVoxelSdf::floatToValue(sdf);
			if (1.0f - std::abs(sdf) < 1e-5f) {
				destinationVoxel.flag_values[targetSdfIndex] = ITMLib::VOXEL_TRUNCATED;
			} else {
				destinationVoxel.flag_values[targetSdfIndex] = ITMLib::VOXEL_NONTRUNCATED;
			}
		}
	}

private:

	ITMScene<TVoxelSdf, TIndex>* sdfSourceScene;
	TVoxelSdf* sdfSourceVoxels;
	typename TIndex::IndexData* sdfSourceIndexData;
	typename TIndex::IndexCache sdfSourceCache;

	ITMScene<TVoxelWarpSource, TIndex>* warpSourceScene;
	TVoxelWarpSource* warpSourceVoxels;
	typename TIndex::IndexData* warpSourceHashEntries;
	typename TIndex::IndexCache warpSourceCache;

	//_DEBUG
	bool hasFocusCoordinates;
	Vector3i focusCoordinates;


	const int sourceSdfIndex;
	const int targetSdfIndex;
};

template<typename TVoxel>
struct CopyIndexedSceneFunctor{
	CopyIndexedSceneFunctor(int sourceIndex, int destinationIndex):
	sourceIndex(sourceIndex), destinationIndex(destinationIndex)
	{}
	void operator()(TVoxel& voxel,Vector3i voxelPosition) {
		voxel.sdf_values[destinationIndex] = voxel.sdf_values[sourceIndex];
		voxel.flag_values[destinationIndex] = voxel.flag_values[sourceIndex];
	}
private:
	int sourceIndex;
	int destinationIndex;
};