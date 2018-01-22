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
#include "SDFSceneVizPipe.tpp"

//ITMLib
#include "../../ITMLib/Utils/ITMSceneLogger.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"
#include "../../ITMLib/Utils/ITMSceneStatisticsCalculator.h"

//** public **

const double SDFViz::canonicalNegativeSDFVoxelColor[4] = {0.141, 0.215, 0.396, 1.0};
const double SDFViz::canonicalPositiveSDFVoxelColor[4] = {0.717, 0.788, 0.960, 1.0};
const double SDFViz::canonicalHashBlockEdgeColor[3] = {0.286, 0.623, 0.854};
const double SDFViz::liveNegativeSDFVoxelColor[4] = {0.101, 0.219, 0.125, 0.6};
const double SDFViz::livePositiveSDFVoxelColor[4] = {0.717, 0.882, 0.749, 0.6};
const double SDFViz::liveHashBlockEdgeColor[3] = {0.537, 0.819, 0.631};
//** private **



SDFViz::SDFViz() :
		minAllowedPoint(-100, -150, 0),
		maxAllowedPoint(200, 50, 300),
		canonicalScenePipe((double*) canonicalNegativeSDFVoxelColor, (double*) canonicalPositiveSDFVoxelColor,
		                   (double*) canonicalHashBlockEdgeColor),
		liveScenePipe((double*) liveNegativeSDFVoxelColor, (double*) livePositiveSDFVoxelColor,
		              (double*) liveHashBlockEdgeColor) {
	sceneLogger = new ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(
			"/media/algomorph/Data/Reconstruction/debug_output/scene",
			canonicalScenePipe.GetScene(), liveScenePipe.GetScene());
	InitializeRendering();
	DrawLegend();
}

int SDFViz::run() {
	//read scenes from disk
	sceneLogger->LoadScenes();
	sceneLogger->StartLoadingWarpState();
	InitializeWarpBuffers();

	ITMSceneStatisticsCalculator<ITMVoxelCanonical, ITMVoxelIndex> statCalculator;

	statCalculator.ComputeVoxelBounds(canonicalScenePipe.GetScene(), minPoint, maxPoint);
	std::cout << "Voxel ranges ( min x,y,z; max x,y,z): " << minPoint << "; " << maxPoint << std::endl;

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



	//set up clipping for data
#define USE_CLIPPING
#ifdef USE_CLIPPING
	vtkSmartPointer<vtkBox> implicitClippingBox = vtkSmartPointer<vtkBox>::New();

	implicitClippingBox->SetBounds(minAllowedPoint.x * maxVoxelDrawSize, maxAllowedPoint.x * maxVoxelDrawSize,
	                               minAllowedPoint.y * maxVoxelDrawSize, maxAllowedPoint.y * maxVoxelDrawSize,
	                               minAllowedPoint.z * maxVoxelDrawSize, maxAllowedPoint.z * maxVoxelDrawSize);

	vtkSmartPointer<vtkExtractPolyDataGeometry> canonicalExtractor = vtkSmartPointer<vtkExtractPolyDataGeometry>::New();
	canonicalExtractor->SetImplicitFunction(implicitClippingBox);
	canonicalExtractor->SetInputData(canonicalVoxelPolydata);
	canonicalExtractor->ExtractInsideOn();
	canonicalExtractor->Update();

	vtkSmartPointer<vtkExtractPolyDataGeometry> liveExtractor = vtkSmartPointer<vtkExtractPolyDataGeometry>::New();
	liveExtractor->SetImplicitFunction(implicitClippingBox);
	liveExtractor->SetInputData(liveVoxelPolydata);
	canonicalExtractor->ExtractInsideOn();
	canonicalExtractor->Update();
#endif


#ifdef USE_CPU_GLYPH //_DEBUG
	// set up glyphs
	vtkSmartPointer<vtkGlyph3D> canonicalGlyph = vtkSmartPointer<vtkGlyph3D>::New();
	vtkSmartPointer<vtkGlyph3D> liveGlyph = vtkSmartPointer<vtkGlyph3D>::New();
	SetUpGlyph(sphere->getOutputPort(), canonicalVoxelPolydata, canonicalGlyph);
	SetUpGlyph(sphere->getOutputPort(), liveVoxelPolydata, liveGlyph);

	vtkSmartPointer<vtkPolyDataMapper> canonicalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkPolyDataMapper> liveMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	SetUpSceneVoxelMapper(canonicalMapper, canonicalColorLookupTable, canonicalGlyph);
	SetUpSceneVoxelMapper(liveMapper, liveColorLookupTable, liveGlyph);
#else
#ifdef USE_CLIPPING
	// set up scene voxel mappers
	vtkSmartPointer<vtkGlyph3DMapper> canonicalMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	vtkSmartPointer<vtkGlyph3DMapper> liveMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	SetUpSceneVoxelMapper(sphere->GetOutputPort(), canonicalMapper, canonicalColorLookupTable, canonicalExtractor);
	SetUpSceneVoxelMapper(sphere->GetOutputPort(), liveMapper, liveColorLookupTable, liveExtractor);
#else
	// set up mappers
	vtkSmartPointer<vtkGlyph3DMapper> canonicalMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	vtkSmartPointer<vtkGlyph3DMapper> liveMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	SetUpSceneVoxelMapper(sphere->GetOutputPort(), canonicalMapper, canonicalColorLookupTable, canonicalVoxelPolydata);
	SetUpSceneVoxelMapper(sphere->GetOutputPort(), liveMapper, liveColorLookupTable, liveVoxelPolydata);
#endif
#endif // ndef USE_CPU_GLYPH


	renderer->AddActor(canonicalVoxelActor);
	renderer->AddActor(liveVoxelActor);
	renderer->AddActor(canonicalHashBlockActor);
	renderer->AddActor(liveHashBlockActor);


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
	vtkPoints* voxels = canonicalVoxelPolydata->GetPoints();
	auto* initialPointRawData = reinterpret_cast<float*>(canonicalInitialPoints->GetVoidPointer(0));
	auto* pointRawData = reinterpret_cast<float*>(voxels->GetVoidPointer(0));
	auto* warpRawData = reinterpret_cast<float*>(warpBuffer->GetVoidPointer(0));

	const auto pointCount = static_cast<const int>(voxels->GetNumberOfPoints());
	for (int iVoxel = 0; iVoxel < pointCount; iVoxel++) {
		//use 1st 3-float field out of 2 for the warp buffer entry
		pointRawData[iVoxel * 3 + 0] = initialPointRawData[iVoxel * 3 + 0] + warpRawData[iVoxel * 6 + 0];
		pointRawData[iVoxel * 3 + 1] = initialPointRawData[iVoxel * 3 + 1] + warpRawData[iVoxel * 6 + 1];
		pointRawData[iVoxel * 3 + 2] = initialPointRawData[iVoxel * 3 + 2] + warpRawData[iVoxel * 6 + 2];
	}
	canonicalVoxelPolydata->Modified();
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
	legend->SetEntry(0, legendSphere->GetOutput(), "Positive Canonical", (double*) canonicalPositiveSDFVoxelColor);
	legend->SetEntry(1, legendSphere->GetOutput(), "Negative Canonical", (double*) canonicalNegativeSDFVoxelColor);
	legend->SetEntry(2, legendSphere->GetOutput(), "Positive Live", (double*) livePositiveSDFVoxelColor);
	legend->SetEntry(3, legendSphere->GetOutput(), "Negative Live", (double*) liveNegativeSDFVoxelColor);

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
	canonicalHashBlockActor->SetVisibility(!canonicalHashBlockActor->GetVisibility());
	renderWindow->Render();
}

void SDFViz::ToggleLiveHashBlockVisibility() {
	liveHashBlockActor->SetVisibility(!liveHashBlockActor->GetVisibility());
	renderWindow->Render();
}

void SDFViz::ToggleCanonicalVoxelVisibility() {
	canonicalVoxelActor->SetVisibility(!canonicalVoxelActor->GetVisibility());
	renderWindow->Render();
}

void SDFViz::ToggleLiveVoxelVisibility() {
	liveVoxelActor->SetVisibility(!liveVoxelActor->GetVisibility());
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
	canonicalVoxelActor->GetProperty()->SetOpacity(
			std::max(0.0, canonicalVoxelActor->GetProperty()->GetOpacity() - 0.05));
	renderWindow->Render();
}

void SDFViz::IncreaseCanonicalVoxelOpacity() {
	canonicalVoxelActor->GetProperty()->SetOpacity(
			std::min(1.0, canonicalVoxelActor->GetProperty()->GetOpacity() + 0.05));
	renderWindow->Render();
}

void SDFViz::SetUpSceneHashBlockMapper(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkGlyph3DMapper>& mapper,
                                       vtkSmartPointer<vtkPolyData>& pointsPolydata) {
	mapper->SetInputData(pointsPolydata);
	mapper->SetSourceConnection(sourceOutput);
	mapper->ScalarVisibilityOff();
	mapper->ScalingOff();
	mapper->SetScaleFactor(1.0);
}

void SDFViz::SetUpSDFColorLookupTable(vtkSmartPointer<vtkLookupTable>& table, const double* rgbaFirstColor,
                                      const double* rgbaSecondColor) {
	table->SetTableRange(0.0, 1.0);
	table->SetNumberOfTableValues(2);
	table->SetNumberOfColors(2);
	table->SetTableValue(0, rgbaFirstColor);
	table->SetTableValue(1, rgbaSecondColor);
	table->Build();
}

void SDFViz::SetUpGlyph(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkPolyData>& polydata,
                        vtkSmartPointer<vtkGlyph3D>& glyph) {
	glyph->SetInputData(polydata);
	glyph->SetSourceConnection(sourceOutput);
	glyph->ScalingOn();
	glyph->ClampingOff();
	glyph->SetScaleModeToScaleByScalar();
	glyph->SetScaleFactor(1.0);
	glyph->SetColorModeToColorByScalar();
}

//CPU glyph version
void SDFViz::SetUpSceneVoxelMapper(vtkSmartPointer<vtkPolyDataMapper>& mapper, vtkSmartPointer<vtkLookupTable>& table,
                                   vtkSmartPointer<vtkGlyph3D>& glyph) {
	mapper->SetInputConnection(glyph->GetOutputPort());
	mapper->ScalarVisibilityOn();
	mapper->SetColorModeToMapScalars();
	mapper->SetLookupTable(table);
	mapper->ColorByArrayComponent("data", 1);
}

//GPU glyph version with filtering
void SDFViz::SetUpSceneVoxelMapper(vtkAlgorithmOutput* sourceOutput,
                                   vtkSmartPointer<vtkGlyph3DMapper>& mapper,
                                   vtkSmartPointer<vtkLookupTable>& table,
                                   vtkSmartPointer<vtkExtractPolyDataGeometry> extractor) {
	mapper->SetInputConnection(extractor->GetOutputPort());
	mapper->SetSourceConnection(sourceOutput);
	mapper->SetLookupTable(table);
	mapper->ScalingOn();
	mapper->ScalarVisibilityOn();
	mapper->SetScalarModeToUsePointData();
	mapper->SetColorModeToMapScalars();
	mapper->SetScaleModeToScaleByMagnitude();
	mapper->SetScaleArray(scalePointAttributeName);
	mapper->Update();
}

//GPU glyph version w/o filtering
void SDFViz::SetUpSceneVoxelMapper(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkGlyph3DMapper>& mapper,
                                   vtkSmartPointer<vtkLookupTable>& table,
                                   vtkSmartPointer<vtkPolyData>& pointsPolydata) {
	mapper->SetInputData(pointsPolydata);
	mapper->SetSourceConnection(sourceOutput);
	mapper->SetLookupTable(table);
	mapper->ScalingOn();
	mapper->ScalarVisibilityOn();
	mapper->SetScalarModeToUsePointData();
	mapper->SetColorModeToMapScalars();
	mapper->SetScaleModeToScaleByMagnitude();
	mapper->SetScaleArray(scalePointAttributeName);
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
