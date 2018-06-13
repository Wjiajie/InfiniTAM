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
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

//local
#include "ITMScene1DSliceVisualizer.h"
#include "ITMScene1DSliceVisualizer.tpp"
#include "../../Objects/Scene/ITMScene.h"
#include "../../Objects/Scene/ITMRepresentationAccess.h"
#include "../ITMPrintHelpers.h"

#include "../../ITMLibDefines.h"
#include "ITMVisualizationWindowManager.h"


using namespace ITMLib;

// region ==================================== CONSTRUCTORS / DESTRUCTORS ==============================================

ITMScene1DSliceVisualizer::ITMScene1DSliceVisualizer(Vector3i focusCoordinate, Axis axis, unsigned int voxelRange) :
		focusCoordinate(focusCoordinate),
		axis(axis),
		voxelRange(voxelRange),
		rangeStartVoxelIndex(focusCoordinate[axis] - ((voxelRange + 1) / 2)),
		rangeEndVoxelIndex(focusCoordinate[axis] + (voxelRange / 2)),
		window(ITMVisualizationWindowManager::Instance().MakeWindow(
				"Scene1DSliceVisualizer_" + AxisToString(axis),
				"Scene 1D Slice Visualizer for "  + AxisToString(axis) + " Axis")){}

//TODO: DRY violation -- same code as EnergyPlotter -- group into single class hierarchy with shared methods
void ITMScene1DSliceVisualizer::SaveScreenshot(std::string path) {
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow = window->GetRenderWindow();
	windowToImageFilter->SetInput(renderWindow);
	windowToImageFilter->SetScale(2);
	windowToImageFilter->SetInputBufferTypeToRGBA();
	windowToImageFilter->ReadFrontBufferOff();
	windowToImageFilter->Update();
	vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
	writer->SetFileName(path.c_str());
	writer->SetInputConnection(windowToImageFilter->GetOutputPort());
	writer->Write();
}

// endregion
// region ==================================== EXPLICIT INSTANTIATIONS =================================================

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