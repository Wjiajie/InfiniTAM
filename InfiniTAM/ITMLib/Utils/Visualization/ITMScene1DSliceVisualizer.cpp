//  ================================================================
//  Created by Gregory Kramida on 6/5/18.
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

//stdlib
#include <utility>

//local
#include "ITMScene1DSliceVisualizer.h"
#include "ITMScene1DSliceVisualizer.tpp"
#include "../../Objects/Scene/ITMScene.h"
#include "../../Objects/Scene/ITMRepresentationAccess.h"
#include "../ITMPrintHelpers.h"

#include "../../ITMLibDefines.h"


using namespace ITMLib;

// region ==================================== CONSTRUCTORS / DESTRUCTORS ==============================================

ITMScene1DSliceVisualizer::ITMScene1DSliceVisualizer(Vector3i focusCoordinate, Axis axis, unsigned int voxelRange,
                                                     std::string imageOutputDirectory) :
		focusCoordinate(focusCoordinate),
		axis(axis),
		voxelRange(voxelRange),
		rangeStartVoxelIndex(focusCoordinate[axis] - ((voxelRange + 1) / 2)),
		rangeEndVoxelIndex(focusCoordinate[axis] + (voxelRange / 2)),
		imageOutputDirectory(std::move(imageOutputDirectory)) {
}

// endregion
// region ==================================== INSTANTIATIONS ==========================================================

template void
ITMScene1DSliceVisualizer::Plot1DSceneSlice<ITMVoxelCanonical, ITMVoxelIndex>(
		ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* scene, Vector4i color, double width);
template void
ITMScene1DSliceVisualizer::Plot1DSceneSlice<ITMVoxelLive, ITMVoxelIndex>(ITMScene<ITMVoxelLive, ITMVoxelIndex>* scene,
                                                                         Vector4i color, double width);

template void
ITMScene1DSliceVisualizer::Plot1DIndexedSceneSlice<ITMVoxelLive, ITMVoxelIndex>(
		ITMScene<ITMVoxelLive, ITMVoxelIndex>* scene, Vector4i color, double width, int fieldIndex);

template void
ITMScene1DSliceVisualizer::Draw1DWarpUpdateVector<ITMVoxelCanonical, ITMVoxelIndex>(
		ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* scene, Vector4i color);

//======================================================================================================================