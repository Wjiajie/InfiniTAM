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
#include <vtk-8.1/vtkVertexGlyphFilter.h>

//local
#include "SDFSceneVizPipe.h"

//ITMLib
#include "../../ITMLib/Utils/ITMSceneLogger.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"
#include "../../ITMLib/Utils/ITMSceneStatisticsCalculator.h"
#include "SDFVizGlobalDefines.h"

//** public **
const std::array<double, 4>  SDFViz::canonicalNegativeSDFVoxelColor = {0.141, 0.215, 0.396, 1.0};
const std::array<double, 4>  SDFViz::canonicalPositiveSDFVoxelColor = {0.717, 0.788, 0.960, 1.0};
const std::array<double, 4>  SDFViz::canonicalNegativeInterestSDFVoxelColor = {0.690, 0.878, 0.902, 1.0};
const std::array<double, 4>  SDFViz::canonicalPositiveInterestSDFVoxelColor = {0.000, 1.000, 1.000, 1.0};
const std::array<double, 4>  SDFViz::canonicalHighlightSDFVoxelColor = {1.000, 0.647, 0.000, 1.0};
const std::array<double, 3>  SDFViz::canonicalHashBlockEdgeColor = {0.286, 0.623, 0.854};
const std::array<double, 4>  SDFViz::liveNegativeSDFVoxelColor = {0.101, 0.219, 0.125, 1.0};
const std::array<double, 4>  SDFViz::livePositiveSDFVoxelColor = {0.717, 0.882, 0.749, 1.0};
const std::array<double, 3>  SDFViz::liveHashBlockEdgeColor = {0.537, 0.819, 0.631};
//** private **

const std::array<std::array<double, 4>, 4> SDFViz::backgroundColors = {{{0.96, 0.96, 0.98, 1.00},  // beige
		                                                        {0.09, 0.07, 0.05, 1.00},  // black raspberry
		                                                        {0.59, 0.44, 0.09, 1.00},  // bristle brown
		                                                        {0.57, 0.64, 0.69, 1.0}}}; // cadet grey

SDFViz::SDFViz(std::string pathToScene, bool showNonInterestCanonicalVoxels, bool showLiveVoxels,
               bool hideInterestCanonicalRegions, bool useInitialCoords,
               Vector3i initialCoords) :
		canonicalScenePipe(canonicalNegativeSDFVoxelColor, canonicalPositiveSDFVoxelColor,
		                   canonicalNegativeInterestSDFVoxelColor, canonicalPositiveInterestSDFVoxelColor,
		                   canonicalHighlightSDFVoxelColor, canonicalHashBlockEdgeColor,
		                   0),
		liveScenePipe(liveNegativeSDFVoxelColor,
		              livePositiveSDFVoxelColor,
		              liveHashBlockEdgeColor),
		highlightVisualizer(),
		iterationIndicator(vtkSmartPointer<vtkTextActor>::New()),
		iterationIndex(0),
		legend(vtkSmartPointer<vtkLegendBoxActor>::New()){
	sceneLogger = new ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(pathToScene,
	                                                                                 canonicalScenePipe.GetScene(),
	                                                                                 liveScenePipe.GetScene());
	InitializeRendering();
	DrawLegend();
	DrawIterationCounter();
	//read scenes from disk
	sceneLogger->LoadScenesCompact();
	hasHighlightInfo = sceneLogger->LoadHighlights(false, "continuous");
	if (hasHighlightInfo) {
		sceneLogger->SetUpInterestRegionsForLoading();
	}
	hasWarpIterationInfo = sceneLogger->StartLoadingWarpState(frameIndex);
	if (hasWarpIterationInfo) {
		canonicalScenePipe.SetFrameIndex(frameIndex);
		InitializeWarpBuffers();
	}


	//Individual voxel shape
	vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
	sphere->SetThetaResolution(6);
	sphere->SetPhiResolution(6);
	sphere->SetRadius(0.5);//size of half a voxel
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
	sdfRenderer->AddActor(canonicalScenePipe.GetVoxelActor());
	sdfRenderer->AddActor(canonicalScenePipe.GetInterestVoxelActor());
	sdfRenderer->AddActor(canonicalScenePipe.GetHashBlockActor());
	sdfRenderer->AddActor(liveScenePipe.GetVoxelActor());
	sdfRenderer->AddActor(liveScenePipe.GetHashBlockActor());
	topRenderer->AddActor(highlightVisualizer.GetHighlightActor());
	// to prevent VTK from doing excessive near-cliping with multi-layered renderers
	DrawDummyMarkers();

	// set up visibility
	canonicalScenePipe.GetVoxelActor()->SetVisibility(showNonInterestCanonicalVoxels);
	liveScenePipe.GetVoxelActor()->SetVisibility(showLiveVoxels);
	canonicalScenePipe.GetInterestVoxelActor()->SetVisibility(!hideInterestCanonicalRegions);

	// set up initial camera position & orientation

	if (useInitialCoords) {
		MoveFocusToVoxelAt(initialCoords.toDouble());
	} else {
		if (hasHighlightInfo) {
			currentHighlight = highlights.GetFirstArray();
			RefocusAtCurrentHighlight();
			RefocusAtCurrentHighlight();//double call as work-around for bug
		} else {
			sdfRenderer->ResetCamera();
		}
	}

}

int SDFViz::Run() {
	renderWindow->Render();
	renderWindowInteractor->Start();
	return EXIT_SUCCESS;
}


SDFViz::~SDFViz() {
	sceneLogger->StopLoadingWarpState();
	delete sceneLogger;
}


void SDFViz::InitializeRendering() {

	sdfRenderer = vtkSmartPointer<vtkRenderer>::New();
	topRenderer = vtkSmartPointer<vtkRenderer>::New();

	double backgroundColor[4];
	memcpy(backgroundColor,backgroundColors[currentBackgrounColorIx].data(), 4*sizeof(double));
	sdfRenderer->SetBackground(backgroundColor);

	renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->SetSize(renderWindow->GetScreenSize());


	renderWindow->SetWindowName("SDF Viz (pre-alpha)");//TODO insert git hash here --Greg (GitHub:Algomorph)
	renderWindow->SetNumberOfLayers(2);
	renderWindow->AddRenderer(sdfRenderer);
	sdfRenderer->SetLayer(0);
	sdfRenderer->InteractiveOn();
	renderWindow->AddRenderer(topRenderer);
	topRenderer->SetLayer(1);
	topRenderer->InteractiveOn();
	topRenderer->SetActiveCamera(sdfRenderer->GetActiveCamera());


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
	if (sceneLogger->BufferCurrentInterestWarpState(this->interestWarpBuffer->GetVoidPointer(0))) {
		UpdateIteration(sceneLogger->GetIterationCursor());
	};
	canonicalScenePipe.UpdateInterestRegionsFromBuffers(this->interestWarpBuffer->GetVoidPointer(0));

	//update highlight viz
	const ITMHighlightIterationInfo info = (*currentHighlight)[iterationIndex];
	Vector3d cameraRight = SDFViz::ComputeCameraRightVector(sdfRenderer->GetActiveCamera());
	Vector3d position = canonicalScenePipe.GetHighlightPosition(info.hash, info.localId);
	std::vector<Vector3d> neighborPositions = canonicalScenePipe.GetHighlightNeighborPositions(info.hash, info.localId);
	highlightVisualizer.SetData(position, info, neighborPositions, cameraRight);

	renderWindow->Render();
}

bool SDFViz::PreviousInterestWarps() {
	if (sceneLogger->BufferPreviousInterestWarpState(this->interestWarpBuffer->GetVoidPointer(0))) {
		UpdateIteration(sceneLogger->GetIterationCursor() == 0 ? 0 : sceneLogger->GetIterationCursor() - 1);
	}
	canonicalScenePipe.UpdateInterestRegionsFromBuffers(this->interestWarpBuffer->GetVoidPointer(0));

	const ITMHighlightIterationInfo info = (*currentHighlight)[iterationIndex];
	Vector3d cameraRight = SDFViz::ComputeCameraRightVector(sdfRenderer->GetActiveCamera());
	Vector3d position = canonicalScenePipe.GetHighlightPosition(info.hash, info.localId);
	std::vector<Vector3d> neighborPositions = canonicalScenePipe.GetHighlightNeighborPositions(info.hash, info.localId);
	highlightVisualizer.SetData(position, info, neighborPositions, cameraRight);

	renderWindow->Render();
}

void SDFViz::DrawLegend() {
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

	legend->UseBackgroundOn();

	//set up legend background
	double backgroundColor[4];
	memcpy(backgroundColor,backgroundColors[1].data(), 4*sizeof(double));//use only black for legend background color
	legend->SetBackgroundColor(backgroundColor);

	sdfRenderer->AddActor(legend);
}

void SDFViz::DrawIterationCounter() {

	iterationIndicator->SetInput("Iteration: 0");
	iterationIndicator->GetPositionCoordinate()->SetCoordinateSystemToView();
	iterationIndicator->GetPositionCoordinate()->SetValue(-0.95, 0.9);
	iterationIndicator->GetPosition2Coordinate()->SetCoordinateSystemToView();
	iterationIndicator->GetPosition2Coordinate()->SetValue(-0.9, 1.0);
	iterationIndicator->GetTextProperty()->SetFontSize(24);
	iterationIndicator->GetTextProperty()->SetColor(0.1, 0.8, 0.5);

	sdfRenderer->AddActor2D(iterationIndicator);
}

void SDFViz::UpdateIteration(unsigned int newValue) {
	iterationIndex = newValue;
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
	canonicalScenePipe.GetInterestVoxelActor()->SetVisibility(!canonicalScenePipe.GetVoxelActor()->GetVisibility());
	canonicalScenePipe.GetVoxelActor()->SetVisibility(!canonicalScenePipe.GetVoxelActor()->GetVisibility());
	renderWindow->Render();
}

void SDFViz::ToggleLiveVoxelVisibility() {
	liveScenePipe.GetVoxelActor()->SetVisibility(!liveScenePipe.GetVoxelActor()->GetVisibility());
	renderWindow->Render();
}

void SDFViz::ToggleInterestVoxelVisibility() {
	canonicalScenePipe.GetInterestVoxelActor()->SetVisibility(
			!canonicalScenePipe.GetInterestVoxelActor()->GetVisibility());
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


void SDFViz::DecreaseLiveVoxelOpacity() {
	liveScenePipe.GetVoxelActor()->GetProperty()->SetOpacity(
			std::max(0.0, liveScenePipe.GetVoxelActor()->GetProperty()->GetOpacity() - 0.05));
	renderWindow->Render();
}

void SDFViz::IncreaseLiveVoxelOpacity() {
	liveScenePipe.GetVoxelActor()->GetProperty()->SetOpacity(
			std::max(0.0, liveScenePipe.GetVoxelActor()->GetProperty()->GetOpacity() + 0.05));
	renderWindow->Render();
}

void SDFViz::MoveFocusToHighlightAt(int hash, int localId) {
	Vector3d position = canonicalScenePipe.GetHighlightPosition(hash, localId);
	std::vector<Vector3d> neighborPositions = canonicalScenePipe.GetHighlightNeighborPositions(hash, localId);
	const ITMHighlightIterationInfo info = (*highlights.GetArrayAt(hash, localId, frameIndex))[iterationIndex];

	std::cout << "Now viewing highlight at hash " << hash << ", voxel "
	          << localId << " in the canonical frame." << std::endl;
	//_DEBUG
//	std::cout << "Neighbor positions: "<<std::endl;
//	for(auto pos : neighborPositions){
//		std::cout << "   " << pos << std::endl;
//	}

	//TODO: bug -- theoretically, needs to be done after camera repositioning, but that doesn't work for some obscure reason -Greg
	Vector3d cameraRight = SDFViz::ComputeCameraRightVector(sdfRenderer->GetActiveCamera());
	highlightVisualizer.SetData(position, info, neighborPositions, cameraRight);
	MoveFocusToVoxelAt(position);
	renderWindow->Render();
}

void SDFViz::MoveFocusToVoxelAt(Vector3d absoluteCoordinates) {
	sdfRenderer->GetActiveCamera()->SetFocalPoint(absoluteCoordinates.values);
	sdfRenderer->GetActiveCamera()->SetViewUp(0.0, 1.0, 0.0);
	absoluteCoordinates.x += 0.9;
	absoluteCoordinates.y += 0.5;
	absoluteCoordinates.z += 1.0;

	sdfRenderer->GetActiveCamera()->SetPosition(absoluteCoordinates.values);
}

void SDFViz::RefocusAtCurrentHighlight() {
	const ITMHighlightIterationInfo& highlightNew = (*currentHighlight)[iterationIndex];
	MoveFocusToHighlightAt(highlightNew.hash, highlightNew.localId);
}

void SDFViz::MoveFocusToNextHighlight() {
	const ITMHighlightIterationInfo& highlightOld = (*currentHighlight)[iterationIndex];
	currentHighlight = highlights.GetArrayAfter(highlightOld.hash, highlightOld.localId, highlightOld.frame);
	const ITMHighlightIterationInfo& highlightNew = (*currentHighlight)[iterationIndex];
	MoveFocusToHighlightAt(highlightNew.hash, highlightNew.localId);
}

void SDFViz::MoveFocusToPreviousHighlight() {
	const ITMHighlightIterationInfo& highlightOld = (*currentHighlight)[iterationIndex];
	currentHighlight = highlights.GetArrayBefore(highlightOld.hash, highlightOld.localId, highlightOld.frame);
	const ITMHighlightIterationInfo& highlightNew = (*currentHighlight)[iterationIndex];
	MoveFocusToHighlightAt(highlightNew.hash, highlightNew.localId);
}

void SDFViz::DrawDummyMarkers() {
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(-500.0, -500.0, -500.0);
	points->InsertNextPoint(500.0, 500.0, 500.0);
	vtkSmartPointer<vtkPolyData> dummyPolyData = vtkSmartPointer<vtkPolyData>::New();
	dummyPolyData->SetPoints(points);
	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
			vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(dummyPolyData);
	vertexFilter->Update();
	vtkSmartPointer<vtkPolyData> polydata =
			vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());
	// Visualization
	vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);
	vtkSmartPointer<vtkActor> actor =
			vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(1);
	actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
	topRenderer->AddActor(actor);
}

Vector3d SDFViz::ComputeCameraRightVector(vtkCamera* camera) {
	Vector3d viewPlaneNormal;
	Vector3d viewUp;
	Vector3d viewRight;
	camera->GetViewPlaneNormal(viewPlaneNormal.values);
	camera->GetViewUp(viewUp.values);
	vtkMath::Cross(viewUp.values, viewPlaneNormal.values, viewRight.values);
	return viewRight;
}

void SDFViz::NextBackgroundColor() {
	currentBackgrounColorIx = (currentBackgrounColorIx + 1) % static_cast<int>(backgroundColors.size());
	double backgroundColor[4];
	memcpy(backgroundColor,backgroundColors[currentBackgrounColorIx].data(), 4*sizeof(double));
	sdfRenderer->SetBackground(backgroundColor);
	renderWindow->Render();
}

void SDFViz::PreviousBackgroundColor() {
	int reversedIndex = static_cast<int>(backgroundColors.size()) - 1 - currentBackgrounColorIx ;
	reversedIndex = (reversedIndex + 1) % static_cast<int>(backgroundColors.size());
	currentBackgrounColorIx = static_cast<int>(backgroundColors.size()) - 1 - reversedIndex;
	double backgroundColor[4];
	memcpy(backgroundColor,backgroundColors[currentBackgrounColorIx].data(), 4*sizeof(double));
	sdfRenderer->SetBackground(backgroundColor);
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
			double dNear, dFar;
			parent->sdfRenderer->GetActiveCamera()->GetPosition(x, y, z);
			parent->sdfRenderer->GetActiveCamera()->GetFocalPoint(xFocalPoint, yFocalPoint, zFocalPoint);
			parent->sdfRenderer->GetActiveCamera()->GetViewUp(xUpVector, yUpVector, zUpVector);
			parent->sdfRenderer->GetActiveCamera()->GetClippingRange(dNear, dFar);
			std::cout << "Camera:" << std::endl;
			std::cout << "  Current position: " << x << ", " << y << ", " << z << std::endl;
			std::cout << "  Current focal point: " << xFocalPoint << ", " << yFocalPoint << ", " << zFocalPoint
			          << std::endl;
			std::cout << "  Current up-vector: " << xUpVector << ", " << yUpVector << ", " << zUpVector
			          << std::endl;
			std::cout << "  Current clipping range: " << dNear << ", " << dFar << std::endl;
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
		} else if (key == "i") {
			//toggle interest region visibility
			parent->ToggleInterestVoxelVisibility();
		} else if (key == "period") {
			if (parent->hasWarpIterationInfo) {
				std::cout << "Loading next iteration warp & updates." << std::endl;
				if (parent->NextNonInterestWarps()) {
					std::cout << "Next warps loaded and display updated." << std::endl;
				} else {
					std::cout << "Could not load next iteration warp & updates." << std::endl;
				}
			}
		} else if (key == "comma") {
			if (parent->hasWarpIterationInfo) {
				std::cout << "Loading previous iteration warp & updates." << std::endl;
				if (parent->PreviousNonInterestWarps()) {
					std::cout << "Previous warps loaded and display updated." << std::endl;
				} else {
					std::cout << "Could not load previous iteration warp & updates." << std::endl;
				}
			}
		} else if (key == "minus" || key == "KP_Subtract") {
			if (rwi->GetAltKey()) {
				parent->DecreaseCanonicalVoxelOpacity();
			} else {
				parent->DecreaseLiveVoxelOpacity();
			}
		} else if (key == "equal" || key == "KP_Add") {
			if (rwi->GetAltKey()) {
				parent->IncreaseCanonicalVoxelOpacity();
			} else {
				parent->IncreaseLiveVoxelOpacity();
			}
		} else if (key == "bracketright") {
			if (parent->hasHighlightInfo) {
				parent->NextInterestWarps();
				std::cout << "Loading next interest voxel warps." << std::endl;
			}
		} else if (key == "bracketleft") {
			if (parent->hasHighlightInfo) {
				parent->PreviousInterestWarps();
				std::cout << "Loading previous interest voxel warps." << std::endl;
			}
		} else if (key == "Prior") {
			if (parent->hasHighlightInfo) {
				parent->MoveFocusToPreviousHighlight();
			} else {
				parent->sdfRenderer->GetActiveCamera()->SetViewUp(0.0, 1.0, 0.0);
				parent->renderWindow->Render();
			}
		} else if (key == "Next") {
			if (parent->hasHighlightInfo) {
				parent->MoveFocusToNextHighlight();
			} else {
				parent->sdfRenderer->GetActiveCamera()->SetViewUp(0.0, -1.0, 0.0);
				parent->renderWindow->Render();
			}
		} else if (key == "Escape") {
			rwi->TerminateApp();
			std::cout << "Exiting application..." << std::endl;
		} else if (key == "Home") {
			if (parent->hasHighlightInfo) {
				parent->RefocusAtCurrentHighlight();
			} else {
				parent->sdfRenderer->ResetCamera();
				parent->renderWindow->Render();
			}
		} else if (key == "b"){
			if(rwi->GetAltKey()){
				parent->PreviousBackgroundColor();
			}else{
				parent->NextBackgroundColor();
			}
		} else if (key == "t"){
			if(rwi->GetAltKey()){
				parent->canonicalScenePipe.ToggleScaleMode();
				parent->renderWindow->Render();
			}else{
				parent->liveScenePipe.ToggleScaleMode();
				parent->renderWindow->Render();
			}
		}
	}
	std::cout << "Key symbol: " << key << std::endl;
	std::cout << "Key code: " << static_cast<int>(rwi->GetKeyCode()) << std::endl;

	// Forward events
	vtkInteractorStyleTrackballCamera::OnKeyPress();

}
