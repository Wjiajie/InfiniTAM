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
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

//local
#include "SDFSceneVizPipe.h"

//ITMLib
#include "../../ITMLib/Utils/ITMSceneLogger.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"
#include "../../ITMLib/Utils/ITMSceneStatisticsCalculator.h"

//** public **

const std::array<double, 4>  SDFViz::canonicalNegativeSDFVoxelColor = {0.141, 0.215, 0.396, 1.0};
const std::array<double, 4>  SDFViz::canonicalPositiveSDFVoxelColor = {0.717, 0.788, 0.960, 1.0};
const std::array<double, 4>  SDFViz::canonicalNegativeInterestSDFVoxelColor = {0.690, 0.878, 0.902, 1.0};
const std::array<double, 4>  SDFViz::canonicalPositiveInterestSDFVoxelColor = {0.000, 1.000, 1.000, 1.0};
const std::array<double, 4>  SDFViz::canonicalHighlightSDFVoxelColor = {1.000, 0.647, 0.000, 1.0};
const std::array<double, 3>  SDFViz::canonicalHashBlockEdgeColor = {0.286, 0.623, 0.854};
const std::array<double, 4>  SDFViz::liveNegativeSDFVoxelColor = {0.101, 0.219, 0.125, 0.6};
const std::array<double, 4>  SDFViz::livePositiveSDFVoxelColor = {0.717, 0.882, 0.749, 0.6};
const std::array<double, 3>  SDFViz::liveHashBlockEdgeColor = {0.537, 0.819, 0.631};
//** private **

SDFViz::SDFViz(std::string pathToScene) :

		canonicalScenePipe(canonicalNegativeSDFVoxelColor, canonicalPositiveSDFVoxelColor,
		                   canonicalNegativeInterestSDFVoxelColor, canonicalPositiveInterestSDFVoxelColor,
		                   canonicalHighlightSDFVoxelColor, canonicalHashBlockEdgeColor,
		                   0),
		liveScenePipe(liveNegativeSDFVoxelColor,
		              livePositiveSDFVoxelColor,
		              liveHashBlockEdgeColor),
		iterationIndicator(vtkSmartPointer<vtkTextActor>::New()) {
	sceneLogger = new ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(pathToScene,
	                                                                                 canonicalScenePipe.GetScene(),
	                                                                                 liveScenePipe.GetScene());
	InitializeRendering();
	DrawLegend();
	DrawIterationCounter();
}

int SDFViz::Run() {
	//read scenes from disk
	sceneLogger->LoadScenesCompact();
	sceneLogger->LoadHighlights();
	sceneLogger->SetUpInterestRegionsForLoading();
	sceneLogger->StartLoadingWarpState(frameIx);
	canonicalScenePipe.SetFrameIndex(frameIx);
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

	// set up viz pipelines
	highlights = sceneLogger->GetHighlights();

	canonicalScenePipe.SetInterestRegionInfo(sceneLogger->GetInterestRegionHashes(), highlights);
	canonicalScenePipe.PreparePipeline(sphere->GetOutputPort(), cube->GetOutputPort());
	canonicalScenePipe.PrepareInterestRegions(sphere->GetOutputPort());
	liveScenePipe.PreparePipeline(sphere->GetOutputPort(), cube->GetOutputPort());

	// add actors
	renderer->AddActor(canonicalScenePipe.GetVoxelActor());
	renderer->AddActor(canonicalScenePipe.GetInterestVoxelActor());
	renderer->AddActor(canonicalScenePipe.GetHashBlockActor());
	renderer->AddActor(liveScenePipe.GetVoxelActor());
	renderer->AddActor(liveScenePipe.GetHashBlockActor());

	// set up initial camera position & orientation
	currentHighlight = highlights.GetFirstLevel1Value();
    RefocusACurrentHighlight();

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

	renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->SetSize(renderWindow->GetScreenSize());

	renderWindow->SetWindowName("SDF Viz (pre-alpha)");//TODO insert git hash here --Greg (GitHub:Algomorph)
	renderWindow->AddRenderer(renderer);

	renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	vtkSmartPointer<KeyPressInteractorStyle> interactorStyle = vtkSmartPointer<KeyPressInteractorStyle>::New();
	interactorStyle->parent = this;
	interactorStyle->SetMouseWheelMotionFactor(0.05);

	renderWindowInteractor->SetInteractorStyle(interactorStyle);
	renderWindowInteractor->SetRenderWindow(renderWindow);

}

bool SDFViz::NextNonInterestWarps() {
	if (!sceneLogger->BufferCurrentWarpState(this->allWarpBuffer->GetVoidPointer(0))) {
		return false;
	}
	canonicalScenePipe.UpdatePointPositionsFromBuffer(allWarpBuffer->GetVoidPointer(0));
	renderWindow->Render();
	return true;
}

bool SDFViz::PreviousNonInterestWarps() {
	if (!sceneLogger->BufferPreviousWarpState(this->allWarpBuffer->GetVoidPointer(0))) {
		return false;
	}
	canonicalScenePipe.UpdatePointPositionsFromBuffer(allWarpBuffer->GetVoidPointer(0));
	renderWindow->Render();
	return true;
}

bool SDFViz::NextInterestWarps() {
	if(sceneLogger->BufferCurrentInterestWarpState(this->interestWarpBuffer->GetVoidPointer(0))){
		UpdateIterationIndicator(sceneLogger->GetIterationCursor());
	};
	canonicalScenePipe.UpdateInterestRegionsFromBuffers(this->interestWarpBuffer->GetVoidPointer(0));
	renderWindow->Render();
}

bool SDFViz::PreviousInterestWarps() {
	if (sceneLogger->BufferPreviousInterestWarpState(this->interestWarpBuffer->GetVoidPointer(0))) {
		UpdateIterationIndicator(sceneLogger->GetIterationCursor() == 0 ? 0 : sceneLogger->GetIterationCursor() - 1);
	}
	canonicalScenePipe.UpdateInterestRegionsFromBuffers(this->interestWarpBuffer->GetVoidPointer(0));
	renderWindow->Render();
}

void SDFViz::DrawLegend() {
	vtkSmartPointer<vtkLegendBoxActor> legend = vtkSmartPointer<vtkLegendBoxActor>::New();
	legend->SetNumberOfEntries(7);
	vtkSmartPointer<vtkSphereSource> legendSphere = vtkSmartPointer<vtkSphereSource>::New();
	legendSphere->Update();

	//set up legend entries
	legend->SetEntry(0, legendSphere->GetOutput(), "Positive Interest",
	                 (double*) canonicalPositiveInterestSDFVoxelColor.data());
	legend->SetEntry(1, legendSphere->GetOutput(), "Negative Interest",
	                 (double*) canonicalNegativeInterestSDFVoxelColor.data());
	legend->SetEntry(2, legendSphere->GetOutput(), "Highlight",
	                 (double*) canonicalHighlightSDFVoxelColor.data());
	legend->SetEntry(3, legendSphere->GetOutput(), "Positive Canonical",
	                 (double*) canonicalPositiveSDFVoxelColor.data());
	legend->SetEntry(4, legendSphere->GetOutput(), "Negative Canonical",
	                 (double*) canonicalNegativeSDFVoxelColor.data());
	legend->SetEntry(5, legendSphere->GetOutput(), "Positive Live", (double*) livePositiveSDFVoxelColor.data());
	legend->SetEntry(6, legendSphere->GetOutput(), "Negative Live", (double*) liveNegativeSDFVoxelColor.data());

	legend->GetPositionCoordinate()->SetCoordinateSystemToView();
	legend->GetPositionCoordinate()->SetValue(0.8, -1.0);
	legend->GetPosition2Coordinate()->SetCoordinateSystemToView();
	legend->GetPosition2Coordinate()->SetValue(1.0, -0.5);

	//set up legend background
	vtkSmartPointer<vtkNamedColors> colors =
			vtkSmartPointer<vtkNamedColors>::New();
	legend->UseBackgroundOn();
	double background[4];
	colors->GetColor("black", background);
	legend->SetBackgroundColor(background);

	renderer->AddActor(legend);
}

void SDFViz::DrawIterationCounter() {

	iterationIndicator->SetInput("Iteration: 0");
	iterationIndicator->GetPositionCoordinate()->SetCoordinateSystemToView();
	iterationIndicator->GetPositionCoordinate()->SetValue(-0.95, 0.9);
	iterationIndicator->GetPosition2Coordinate()->SetCoordinateSystemToView();
	iterationIndicator->GetPosition2Coordinate()->SetValue(-0.9, 1.0);
	iterationIndicator->GetTextProperty()->SetFontSize(24);
	iterationIndicator->GetTextProperty()->SetColor(0.1, 0.8, 0.5);

	renderer->AddActor2D(iterationIndicator);
}

void SDFViz::UpdateIterationIndicator(unsigned int newValue) {
	iterationIndicator->SetInput(("Iteration: " + std::to_string(newValue)).c_str());
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
	if (!sceneLogger->GetInterestRegionsSetUp()) {
		DIEWITHEXCEPTION("Interest regions haven't been set up, cannot initialize WarpBuffers");
	}
	this->allWarpBuffer = vtkSmartPointer<vtkFloatArray>::New();
	allWarpBuffer->SetNumberOfComponents(3);
	allWarpBuffer->SetNumberOfTuples(sceneLogger->GetVoxelCount() * 2);
	this->interestWarpBuffer = vtkSmartPointer<vtkFloatArray>::New();
	interestWarpBuffer->SetNumberOfComponents(3);
	interestWarpBuffer->SetNumberOfTuples(sceneLogger->GetTotalInterestVoxelCount() * 2);

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

void SDFViz::MoveFocusToHighlightAt(int hash, int localId){
	Vector3d position = canonicalScenePipe.GetHighlightPosition(hash, localId);
	std::cout << "Now viewing highlight at hash " << hash << ", voxel "
	          << localId << " in the canonical frame." << std::endl;
	renderer->GetActiveCamera()->SetFocalPoint(position.values);
	renderer->GetActiveCamera()->SetViewUp(0.0, -1.0, 0.0);
	position.x -= 0.9;
	position.y -= 0.5;
	position.z -= 1.0;

	renderer->GetActiveCamera()->SetPosition(position.values);
	renderWindow->Render();
}

void SDFViz::RefocusACurrentHighlight(){
	const ITMHighlightIterationInfo& highlightNew = (*currentHighlight)[0];
	MoveFocusToHighlightAt(highlightNew.hash,highlightNew.localId);
}

void SDFViz::MoveFocusToNextHighlight() {
	const ITMHighlightIterationInfo& highlightOld = (*currentHighlight)[0];
	currentHighlight = highlights.GetLevel1ValueAfter(highlightOld.hash, highlightOld.localId, highlightOld.frame);
	const ITMHighlightIterationInfo& highlightNew = (*currentHighlight)[0];
	MoveFocusToHighlightAt(highlightNew.hash,highlightNew.localId);
}

void SDFViz::MoveFocusToPreviousHighlight() {
	const ITMHighlightIterationInfo& highlightOld = (*currentHighlight)[0];
	currentHighlight = highlights.GetLevel1ValueBefore(highlightOld.hash, highlightOld.localId, highlightOld.frame);
	const ITMHighlightIterationInfo& highlightNew = (*currentHighlight)[0];
	MoveFocusToHighlightAt(highlightNew.hash,highlightNew.localId);
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
			if (parent->NextNonInterestWarps()) {
				std::cout << "Next warps loaded and display updated." << std::endl;
			} else {
				std::cout << "Could not load next iteration warp & updates." << std::endl;
			}

		} else if (key == "comma") {
			std::cout << "Loading previous iteration warp & updates." << std::endl;
			if (parent->PreviousNonInterestWarps()) {
				std::cout << "Previous warps loaded and display updated." << std::endl;
			} else {
				std::cout << "Could not load previous iteration warp & updates." << std::endl;
			}
		} else if (key == "minus" || key == "KP_Subtract") {
			parent->DecreaseCanonicalVoxelOpacity();
		} else if (key == "equal" || key == "KP_Add") {
			parent->IncreaseCanonicalVoxelOpacity();
		} else if (key == "bracketright") {
			parent->NextInterestWarps();
			std::cout << "Loading next interest voxel warps." << std::endl;
		} else if (key == "bracketleft") {
			parent->PreviousInterestWarps();
			std::cout << "Loading previous interest voxel warps." << std::endl;
		} else if (key == "Prior") {
			parent->MoveFocusToPreviousHighlight();
		} else if (key == "Next") {
			parent->MoveFocusToNextHighlight();
		} else if (key == "Escape") {
			rwi->TerminateApp();
			std::cout << "Exiting application..." << std::endl;
		} else if (key == "Home"){
			parent->RefocusACurrentHighlight();
		}


	}
	std::cout << "Key symbol: " << key << std::endl;

	// Forward events
	vtkInteractorStyleTrackballCamera::OnKeyPress();

}
