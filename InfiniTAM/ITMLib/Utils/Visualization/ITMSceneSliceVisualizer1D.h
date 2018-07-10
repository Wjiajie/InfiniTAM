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
#pragma once

//VTK
#include <vtkContextView.h>

//local
#include "ITMVisualizationCommon.h"
#include "../ITMMath.h"
#include "../../Objects/Scene/ITMScene.h"
#include "ITMVisualizationWindowManager.h"

template<typename T>
class vtkSmartPointer;
class vtkChartXY;


namespace ITMLib{
class ITMSceneSliceVisualizer1D {
public:
	ITMSceneSliceVisualizer1D(Vector3i focusCoordinate, Axis axis, unsigned int voxelRange);
	~ITMSceneSliceVisualizer1D() = default;


	template<typename TVoxel, typename TIndex>
	void Plot1DSceneSlice(ITMScene <TVoxel, TIndex>* scene, Vector4i color, double width);
	template<typename TVoxel, typename TIndex>
	void Draw1DWarpUpdateVector(ITMScene <TVoxel, TIndex>* scene, Vector4i color);
	template<typename TVoxel, typename TIndex>
	void Plot1DIndexedSceneSlice(ITMScene<TVoxel, TIndex>* scene, Vector4i color, double width, int fieldIndex);
	void SaveScreenshot(std::string path);


private:
	template<typename TVoxel, typename TIndex, typename TGetSDFFunctor>
	void Plot1DSceneSliceHelper(ITMScene <TVoxel, TIndex>* scene, Vector4i color, double width);

	ITMChartWindow* window;

	const Vector3i focusCoordinates;
	const Axis axis;
	const int rangeStartVoxelIndex;
	const int rangeEndVoxelIndex;
	const unsigned int voxelRange;


};


}//namespace ITMLib


