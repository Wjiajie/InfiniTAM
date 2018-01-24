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
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkPointData.h>
#include <vtkSphereSource.h>
#include <vtkBridgeDataSet.h>
#include <vtkGlyph3D.h>
#include <vtkFloatArray.h>
#include <vtkGlyph3DMapper.h>
#include <vtkLookupTable.h>
#include <vtkLegendBoxActor.h>
#include <vtkNamedColors.h>
#include <vtkClipPolyData.h>
#include <vtkCubeSource.h>
#include <vtkBox.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkImageData.h>
#include <vtkSetGet.h>
#include <vtkDataSetMapper.h>
#include <vtkStructuredGrid.h>
#include <vtkUnstructuredGrid.h>
#include <vtkUniformGrid.h>

//local
#include "SDFSceneVizPipe.h"

//ITMLib
#include "../../ITMLib/Utils/ITMSceneLogger.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"
#include "../../ITMLib/Utils/ITMSceneStatisticsCalculator.h"

//** public **

const std::array<double,4>  SDFViz::canonicalNegativeSDFVoxelColor = {0.141, 0.215, 0.396, 1.0};
const std::array<double,4>  SDFViz::canonicalPositiveSDFVoxelColor = {0.717, 0.788, 0.960, 1.0};
const std::array<double,3>  SDFViz::canonicalHashBlockEdgeColor = {0.286, 0.623, 0.854};
const std::array<double,4>  SDFViz::liveNegativeSDFVoxelColor = {0.101, 0.219, 0.125, 0.6};
const std::array<double,4>  SDFViz::livePositiveSDFVoxelColor = {0.717, 0.882, 0.749, 0.6};
const std::array<double,3>  SDFViz::liveHashBlockEdgeColor = {0.537, 0.819, 0.631};
//** private **



SDFViz::SDFViz() :

		canonicalScenePipe(canonicalNegativeSDFVoxelColor,
		                   canonicalPositiveSDFVoxelColor,
		                   canonicalHashBlockEdgeColor),
		liveScenePipe(liveNegativeSDFVoxelColor,
		              livePositiveSDFVoxelColor,
		              liveHashBlockEdgeColor) {
	sceneLogger = new ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(
			"/media/algomorph/Data/Reconstruction/debug_output/test_scene",
			canonicalScenePipe.GetScene(), liveScenePipe.GetScene());
	InitializeRendering();
	DrawLegend();
}

int SDFViz::run() {
	//read scenes from disk
	sceneLogger->LoadScenesCompact();
	sceneLogger->StartLoadingWarpState();
	InitializeWarpBuffers();

	//Individual voxel shape
	vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
	sphere->SetThetaResolution(6);
	sphere->SetPhiResolution(6);
	sphere->SetRadius(canonicalScenePipe.maxVoxelDrawSize / 2);
	sphere->Update();

	//Voxel hash block shape
	vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
	cube->SetBounds(0, SDF_BLOCK_SIZE, 0, SDF_BLOCK_SIZE, 0, SDF_BLOCK_SIZE);

	canonicalScenePipe.PreparePipeline(sphere->GetOutputPort(),cube->GetOutputPort());
	liveScenePipe.PreparePipeline(sphere->GetOutputPort(),cube->GetOutputPort());

	renderer->AddActor(canonicalScenePipe.GetVoxelActor());
	renderer->AddActor(liveScenePipe.GetVoxelActor());
	renderer->AddActor(canonicalScenePipe.GetVoxelActor());
	renderer->AddActor(liveScenePipe.GetVoxelActor());


	renderer->GetActiveCamera()->SetPosition(7.0, 22.0, -72.66);
	renderer->GetActiveCamera()->SetFocalPoint(0.5, -40.5, 230.0);
	renderer->GetActiveCamera()->SetViewUp(0.0, -1.0, 0.0);
//	renderer->ResetCamera();//used when need to choose new better initial camera pose manually

	renderWindow->Render();
	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}


SDFViz::~SDFViz() {
	sceneLogger->StopLoadingWarpState();
	delete sceneLogger;
}


void SDFViz::InitializeRendering() {

	renderer = vtkSmartPointer<vtkRenderer>::New();

	renderer->SetBackground(0.0, 0.0, 0.0);
	//renderer->SetBackground(1.0, 1.0, 1.0);

	renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->SetSize(1280, 768);
	renderWindow->SetWindowName("SDF Viz (pre-alpha)");//TODO insert git hash here --Greg (GitHub:Algomorph)
	renderWindow->AddRenderer(renderer);

	renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	vtkSmartPointer<KeyPressInteractorStyle> interactorStyle = vtkSmartPointer<KeyPressInteractorStyle>::New();
	interactorStyle->parent = this;
	interactorStyle->SetMouseWheelMotionFactor(0.05);

	renderWindowInteractor->SetInteractorStyle(interactorStyle);
	renderWindowInteractor->SetRenderWindow(renderWindow);

}

void SDFViz::UpdateVoxelPositionsFromWarpBuffer() {
	canonicalScenePipe.UpdatePointPositionsFromBuffer(warpBuffer->GetVoidPointer(0));
	renderWindow->Render();
}

bool SDFViz::NextWarps() {
	if (!sceneLogger->BufferNextWarpState(this->warpBuffer->GetVoidPointer(0))) {
		return false;
	}
	UpdateVoxelPositionsFromWarpBuffer();
	return true;
}

bool SDFViz::PreviousWarps() {
	if (!sceneLogger->BufferPreviousWarpState(this->warpBuffer->GetVoidPointer(0))) {
		return false;
	}
	UpdateVoxelPositionsFromWarpBuffer();
	return true;
}

void SDFViz::DrawLegend() {
	vtkSmartPointer<vtkLegendBoxActor> legend = vtkSmartPointer<vtkLegendBoxActor>::New();
	legend->SetNumberOfEntries(4);
	vtkSmartPointer<vtkSphereSource> legendSphere = vtkSmartPointer<vtkSphereSource>::New();
	legendSphere->Update();

	//set up legend entries
	legend->SetEntry(0, legendSphere->GetOutput(), "Positive Canonical", (double*) canonicalPositiveSDFVoxelColor.data());
	legend->SetEntry(1, legendSphere->GetOutput(), "Negative Canonical", (double*) canonicalNegativeSDFVoxelColor.data());
	legend->SetEntry(2, legendSphere->GetOutput(), "Positive Live", (double*) livePositiveSDFVoxelColor.data());
	legend->SetEntry(3, legendSphere->GetOutput(), "Negative Live", (double*) liveNegativeSDFVoxelColor.data());

	legend->GetPositionCoordinate()->SetCoordinateSystemToView();
	legend->GetPositionCoordinate()->SetValue(0.8, -1.0);
	legend->GetPosition2Coordinate()->SetCoordinateSystemToView();
	legend->GetPosition2Coordinate()->SetValue(1.0, -0.7);

	//set up legend background
	vtkSmartPointer<vtkNamedColors> colors =
			vtkSmartPointer<vtkNamedColors>::New();
	legend->UseBackgroundOn();
	double background[4];
	colors->GetColor("black", background);
	legend->SetBackgroundColor(background);

	renderer->AddActor(legend);
}

void SDFViz::ToggleCanonicalHashBlockVisibility() {
	canonicalScenePipe.GetHashBlockActor()->SetVisibility(!canonicalScenePipe.GetHashBlockActor()->GetVisibility());
	std::cout << "meh" << std::endl;
	renderWindow->Render();
}

void SDFViz::ToggleLiveHashBlockVisibility() {
	liveScenePipe.GetHashBlockActor()->SetVisibility(!liveScenePipe.GetHashBlockActor()->GetVisibility());
	std::cout << "blah" << std::endl;
	renderWindow->Render();
}

void SDFViz::ToggleCanonicalVoxelVisibility() {
	canonicalScenePipe.GetVoxelActor()->SetVisibility(!canonicalScenePipe.GetVoxelActor()->GetVisibility());
	renderWindow->Render();
}

void SDFViz::ToggleLiveVoxelVisibility() {
	liveScenePipe.GetVoxelActor()->SetVisibility(!liveScenePipe.GetVoxelActor()->GetVisibility());
	renderWindow->Render();
}

void SDFViz::InitializeWarpBuffers() {
	if (!sceneLogger->GetScenesLoaded()) {
		DIEWITHEXCEPTION("Scenes not yet loaded, cannot initialize WarpBuffers");
	}
	this->warpBuffer = vtkSmartPointer<vtkFloatArray>::New();
	warpBuffer->SetNumberOfComponents(3);
	warpBuffer->SetNumberOfTuples(sceneLogger->GetVoxelCount() * 2);
}

void SDFViz::DecreaseCanonicalVoxelOpacity() {
	canonicalScenePipe.GetVoxelActor()->GetProperty()->SetOpacity(
			std::max(0.0, canonicalScenePipe.GetVoxelActor()->GetProperty()->GetOpacity() - 0.05));
	renderWindow->Render();
}

void SDFViz::IncreaseCanonicalVoxelOpacity() {
	canonicalScenePipe.GetVoxelActor()->GetProperty()->SetOpacity(
			std::max(0.0, canonicalScenePipe.GetVoxelActor()->GetProperty()->GetOpacity() + 0.05));
	renderWindow->Render();
}

vtkStandardNewMacro(KeyPressInteractorStyle);

void KeyPressInteractorStyle::OnKeyPress() {

	// Get the keypress
	vtkRenderWindowInteractor* rwi = this->Interactor;
	std::string key = rwi->GetKeySym();


	if (parent != nullptr) {
		if (key == "c") {
			//record camera position
			double x, y, z;
			double xFocalPoint, yFocalPoint, zFocalPoint;
			double xUpVector, yUpVector, zUpVector;
			parent->renderer->GetActiveCamera()->GetPosition(x, y, z);
			parent->renderer->GetActiveCamera()->GetFocalPoint(xFocalPoint, yFocalPoint, zFocalPoint);
			parent->renderer->GetActiveCamera()->GetViewUp(xUpVector, yUpVector, zUpVector);
			std::cout << "Camera:" << std::endl;
			std::cout << "  Current position: " << x << ", " << y << ", " << z << std::endl;
			std::cout << "  Current focal point: " << xFocalPoint << ", " << yFocalPoint << ", " << zFocalPoint
			          << std::endl;
			std::cout << "  Current up-vector: " << xUpVector << ", " << yUpVector << ", " << zUpVector
			          << std::endl;
			std::cout.flush();
		} else if (key == "v") {
			//toggle voxel blocks visibility
			if (rwi->GetAltKey()) {
				parent->ToggleCanonicalVoxelVisibility();
			} else {
				parent->ToggleLiveVoxelVisibility();
			}
		} else if (key == "h") {
			//toggle hash blocks visibility
			if (rwi->GetAltKey()) {
				parent->ToggleCanonicalHashBlockVisibility();
			} else {
				parent->ToggleLiveHashBlockVisibility();
			}
		} else if (key == "period") {
			std::cout << "Loading next iteration warp & updates." << std::endl;
			if (parent->NextWarps()) {
				std::cout << "Next warps loaded and display updated." << std::endl;
			} else {
				std::cout << "Could not load next iteration warp & updates." << std::endl;
			}

		} else if (key == "comma") {
			std::cout << "Loading previous iteration warp & updates." << std::endl;
			if (parent->PreviousWarps()) {
				std::cout << "Previous warps loaded and display updated." << std::endl;
			} else {
				std::cout << "Could not load previous iteration warp & updates." << std::endl;
			}
		} else if (key == "minus" || key == "KP_Subtract") {
			parent->DecreaseCanonicalVoxelOpacity();
		} else if (key == "equal" || key == "KP_Add") {
			parent->IncreaseCanonicalVoxelOpacity();
		}

	}
	std::cout << "Key symbol: " << key << std::endl;

	// Forward events
	vtkInteractorStyleTrackballCamera::OnKeyPress();

}
