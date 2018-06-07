//  ================================================================
//  Created by Gregory Kramida on 6/6/18.
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

//stdlib
#include <utility>

//VTK
#include <vtkContextView.h>
#include <vtkSmartPointer.h>
#include <vtkChartXY.h>
#include <vtkTable.h>
#include <vtkFloatArray.h>
#include <vtkRenderer.h>
#include <vtkContextScene.h>
#include <vtkPlot.h>
#include <vtkRenderWindow.h>

//local
#include "ITMScene1DSliceVisualizer.h"
#include "../../Objects/Scene/ITMScene.h"
#include "../../Objects/Scene/ITMRepresentationAccess.h"
#include "ITMVTKVisualizer.h"

using namespace ITMLib;



template<typename TVoxel, typename TIndex>
void ITMScene1DSliceVisualizer::Plot1DSceneSlice(ITMScene<TVoxel, TIndex>* scene, Vector4i color) {

	// set up table & columns
	vtkSmartPointer<vtkFloatArray> horizontalAxisPoints = vtkSmartPointer<vtkFloatArray>::New();
	horizontalAxisPoints->SetName((AxisToString(this->axis) + " Axis").c_str());
	vtkSmartPointer<vtkFloatArray> sdfValues = vtkSmartPointer<vtkFloatArray>::New();
	sdfValues->SetName("SDF Value");
	vtkSmartPointer<vtkTable> table = vtkSmartPointer<vtkTable>::New();
	table->AddColumn(horizontalAxisPoints);
	table->AddColumn(sdfValues);
	table->SetNumberOfRows(voxelRange);

	// scene access variables
	typename TIndex::IndexCache cache;
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	typename TIndex::IndexData* indexData = scene->index.GetEntries();
	Vector3i currentVoxelPosition = focusCoordinate;
	currentVoxelPosition[axis] = rangeStartVoxelIndex;

	// fill table
	for(int iValue = 0; iValue < voxelRange; iValue++,currentVoxelPosition[axis]++){
		int vmIndex = 0;
		TVoxel voxel = readVoxel(voxels, indexData, currentVoxelPosition, vmIndex, cache);
		table->SetValue(iValue, 0, iValue);
		table->SetValue(iValue, 1, TVoxel::valueToFloat(voxel.sdf));
	}

	vtkPlot* line = ITMVTKVisualizer::Instance().GetChart()->AddPlot(vtkChart::LINE);
	line->SetInputData(table, 0, 1);
	line->SetColor(color.r, color.g, color.b, color.a);
	line->SetWidth(1.0);

	ITMVTKVisualizer::Instance().GetChart()->Update();
	ITMVTKVisualizer::Instance().Update();

	debug_print("MIAU");
	//chart->Update();
	//chart->Modified();
	//view->Update();
	//view->Render();
	//view->GetRenderWindow()->Render();
}
