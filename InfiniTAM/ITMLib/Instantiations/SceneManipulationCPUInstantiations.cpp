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

#include "../ITMLibDefines.h"

//Note: ".tpp" files have to be included for all explicit instantiations in order to link properly
#include "../Objects/Scene/ITMSceneManipulation.tpp"
#include "../Objects/Scene/ITMScene.h"

using namespace ITMLib;

//scene manipulation functions
template void CopySceneSDFandFlagsWithOffset_CPU<ITMVoxelCanonical,ITMVoxelLive,ITMVoxelIndex>(
		ITMScene<ITMVoxelLive, ITMVoxelIndex>* destination,
		ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* source,
		Vector3i offset);
template bool SetVoxel_CPU<ITMVoxelCanonical,ITMVoxelIndex>(ITMScene<ITMVoxelCanonical,
                                                            ITMVoxelIndex>* scene, Vector3i at, ITMVoxelCanonical voxel);

template class ITMSceneManipulationEngine_CPU<ITMVoxelCanonical,ITMVoxelIndex>;
template class ITMSceneManipulationEngine_CPU<ITMVoxelLive,ITMVoxelIndex>;

template bool SetVoxel_CPU<ITMVoxelLive,ITMVoxelIndex>(
		ITMScene<ITMVoxelLive, ITMVoxelIndex>* scene,Vector3i at, ITMVoxelLive voxel);
template ITMVoxelCanonical ReadVoxel<ITMVoxelCanonical,ITMVoxelIndex>(
		ITMScene<ITMVoxelCanonical, ITMVoxelIndex>& scene, Vector3i at);

template void GetVoxelHashLocals<ITMVoxelLive>(THREADPTR(int)& vmIndex,
                                               THREADPTR(int)& locId,
                                               THREADPTR(int)& xInBlock,
                                               THREADPTR(int)& yInBlock,
                                               THREADPTR(int)& zInBlock,
                                               const CONSTPTR(ITMVoxelLive*) voxels,
                                               const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* hashEntries,
                                               THREADPTR(ITMLib::ITMVoxelBlockHash::IndexCache) & cache,
                                               const CONSTPTR(Vector3i)& at);

template void OffsetWarps<ITMVoxelCanonical,ITMVoxelIndex>(
		ITMScene<ITMVoxelCanonical, ITMVoxelIndex>& destination, Vector3f offset);

template bool CopySceneSlice_CPU<ITMVoxelCanonical,ITMVoxelIndex>(
		ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* destination,
		ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* source,
		Vector3i minPoint, Vector3i maxPoint);