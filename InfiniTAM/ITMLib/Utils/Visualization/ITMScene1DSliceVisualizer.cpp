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

//VTK
#include <vtkContextView.h>
#include <vtkSmartPointer.h>
#include <vtkChartXY.h>
#include <vtkTable.h>
#include <vtkFloatArray.h>
#include <vtkRenderer.h>
#include <vtkContextScene.h>
#include <vtkPlot.h>

//local
#include "ITMScene1DSliceVisualizer.h"
#include "../../Objects/Scene/ITMScene.h"
#include "../../Objects/Scene/ITMRepresentationAccess.h"
#include "../ITMPrintHelpers.h"


using namespace ITMLib;


ITMScene1DSliceVisualizer::ITMScene1DSliceVisualizer(Vector3i focusCoordinate, Axis axis, unsigned int voxelRange,
                                                     std::string imageOutputDirectory,
                                                     vtkSmartPointer<vtkContextView> view) :
		focusCoordinate(focusCoordinate),
		axis(axis),
		voxelRange(voxelRange),
		rangeStartVoxelIndex(focusCoordinate[axis] - ((voxelRange+1) / 2)),
		rangeEndVoxelIndex(focusCoordinate[axis] + (voxelRange / 2)),
		imageOutputDirectory(std::move(imageOutputDirectory)),
		view(view), chart(vtkSmartPointer<vtkChartXY>::New()) {

	view->GetScene()->AddItem(chart);

}

