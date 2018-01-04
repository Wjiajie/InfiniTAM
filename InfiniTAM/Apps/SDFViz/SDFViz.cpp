//  ================================================================
//  Created by Gregory Kramida on 1/3/18.
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
#include "SDFViz.h"

//vtk
#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtk-8.1/vtkInteractorStyleTrackballActor.h>
#include <vtk-8.1/vtkInteractorStyleTrackballCamera.h>
#include <vtk-8.1/vtkVertexGlyphFilter.h>
#include <vtk-8.1/vtkPointData.h>
#include <vtk-8.1/vtkNamedColors.h>

//local
#include "../../ITMLib/Utils/ITMLibSceneWarpFileIO.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"

int SDFViz::run() {

	//Generate random test voxel array, geometries, and actors
	const int sceneWidthVoxels = 20;
	const int sceneHeightVoxels = 20;
	const int sceneDepthVoxels = 20;
	const int centerX = sceneWidthVoxels / 2;
	const int centerY = sceneHeightVoxels / 2;
	const int centerZ = sceneDepthVoxels / 2;
	const int voxelCount = sceneWidthVoxels * sceneHeightVoxels * sceneDepthVoxels;
	const double voxelDrawSize = 0.2;

	float testSdfValues[sceneDepthVoxels][sceneHeightVoxels][sceneWidthVoxels];

	vtkSmartPointer<vtkPoints> points =
			vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkUnsignedCharArray> colors =
			vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName ("Colors");

	//0.4, 0.8, 0.3
	unsigned char subtleGreen[3] = {100, 220, 75};
	// Setup colors
	vtkSmartPointer<vtkNamedColors> namedColors =
			vtkSmartPointer<vtkNamedColors>::New();

	for (int z = 0; z < sceneDepthVoxels; z++) {
		for (int y = 0; y < sceneHeightVoxels; y++) {
			for (int x = 0; x < sceneWidthVoxels; x++) {
				float SdfValue = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				testSdfValues[z][x][y] = SdfValue;

				points->InsertNextPoint (voxelDrawSize * (x - centerX), voxelDrawSize * (y - centerY),
				                         voxelDrawSize * (z - centerZ));

				unsigned char sdfColor[3] = {static_cast<unsigned char>(subtleGreen[0]*SdfValue),
				                             static_cast<unsigned char>(subtleGreen[1]*SdfValue),
				                             static_cast<unsigned char>(subtleGreen[2]*SdfValue)};
				colors->InsertNextTypedTuple(sdfColor);
			}
		}
	}
//	points->InsertNextPoint (0.0, 0.0, 0.0);
//	points->InsertNextPoint (1.0, 0.0, 0.0);
//	points->InsertNextPoint (0.0, 1.0, 0.0);
//	colors->InsertNextTypedTuple(namedColors->GetColor3ub("Tomato").GetData());
//	colors->InsertNextTypedTuple(namedColors->GetColor3ub("Mint").GetData());
//	colors->InsertNextTypedTuple(namedColors->GetColor3ub("Peacock").GetData());

	vtkSmartPointer<vtkPolyData> pointsPolydata =
			vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
			vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata =
			vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());

	polydata->GetPointData()->SetScalars(colors);

	// Visualization
	vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkSmartPointer<vtkActor> actor =
			vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(8);
	renderer->AddActor(actor);

	//renderer->GetActiveCamera()->SetPosition(0.0,0.0,-1.0);
	//renderer->ResetCamera();
	//renderer->GetActiveCamera()->Zoom(1.5);

	renderWindow->Render();
	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

SDFViz::SDFViz() {
	ITMLibSettings* settings = new ITMLibSettings();
	MemoryDeviceType memoryType = settings->GetMemoryType();
	canonicalScene = new ITMScene<ITMVoxelCanonical, ITMVoxelIndex>(&settings->sceneParams, settings->swappingMode ==
	                                                                                        ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                                memoryType);
	liveScene = new ITMScene<ITMVoxelLive, ITMVoxelIndex>(&settings->sceneParams, settings->swappingMode ==
	                                                                              ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                      memoryType);
	sceneLogger = new ITMLibSceneWarpFileIO<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(
			"/media/algomorph/Data/4dmseg/Killing/scene", canonicalScene, liveScene);
	initializeRendering();
}


SDFViz::~SDFViz() {
	delete canonicalScene;
	delete liveScene;
	delete sceneLogger;
}

void SDFViz::initializeRendering() {

	renderer = vtkSmartPointer<vtkRenderer>::New();

	renderer->SetBackground(0.0, 0.0, 0.0);
	//renderer->SetBackground(1.0, 1.0, 1.0);

	renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->SetSize(1280, 768);
	renderWindow->AddRenderer(renderer);

	renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	vtkSmartPointer<vtkInteractorStyle> interactorStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactorStyle->SetMouseWheelMotionFactor(0.05);

	renderWindowInteractor->SetInteractorStyle(interactorStyle);
	renderWindowInteractor->SetRenderWindow(renderWindow);

}

