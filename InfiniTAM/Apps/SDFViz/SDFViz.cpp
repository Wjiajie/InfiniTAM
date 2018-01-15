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
#include <vtk-8.1/vtkDataSetMapper.h>
#include <vtk-8.1/vtkStructuredGrid.h>
#include <vtk-8.1/vtkUnstructuredGrid.h>
#include <vtk-8.1/vtkUniformGrid.h>

//local
#include "../../ITMLib/Utils/ITMSceneLogger.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"
#include "../../ITMLib/Utils/ITMSceneStatisticsCalculator.h"

//** public **
const double SDFViz::maxVoxelDrawSize = 1.0;
const double SDFViz::canonicalNegativeSDFColor[4] = {0.141, 0.215, 0.396, 1.0};
const double SDFViz::canonicalPositiveSDFColor[4] = {0.717, 0.788, 0.960, 1.0};
const double SDFViz::liveNegativeSDFColor[4] = {0.101, 0.219, 0.125, 0.6};
const double SDFViz::livePositiveSDFColor[4] = {0.717, 0.882, 0.749, 0.6};

//** private **
const char* SDFViz::colorPointAttributeName = "color";
const char* SDFViz::scalePointAttributeName = "scale";


SDFViz::SDFViz() :
		minAllowedPoint(-100, -150, 0),
		maxAllowedPoint(200, 50, 300),
		canonicalVoxelPolydata(vtkSmartPointer<vtkPolyData>::New()),
		liveVoxelPolydata(vtkSmartPointer<vtkPolyData>::New()),
		canonicalHashBlockGrid(vtkSmartPointer<vtkPolyData>::New()),
		liveHashBlockGrid(vtkSmartPointer<vtkPolyData>::New()) {
	auto* settings = new ITMLibSettings();
	MemoryDeviceType memoryType = settings->GetMemoryType();
	canonicalScene = new ITMScene<ITMVoxelCanonical, ITMVoxelIndex>(
			&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED, memoryType);
	liveScene = new ITMScene<ITMVoxelLive, ITMVoxelIndex>(
			&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED, memoryType);
	sceneLogger = new ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(
			"/media/algomorph/Data/4dmseg/Killing/scene", canonicalScene, liveScene);
	InitializeRendering();
	DrawLegend();
}

int SDFViz::run() {
	//read scenes from disk
	sceneLogger->LoadScenes();
	sceneLogger->StartLoadingWarpState();
	InitializeWarpBuffers();

	ITMSceneStatisticsCalculator<ITMVoxelCanonical, ITMVoxelIndex> statCalculator;

	statCalculator.ComputeVoxelBounds(canonicalScene, minPoint, maxPoint);
	std::cout << "Voxel ranges ( min x,y,z; max x,y,z): " << minPoint << "; " << maxPoint << std::endl;

	PrepareSceneForRendering(canonicalScene, canonicalVoxelPolydata, canonicalHashBlockGrid);
	PrepareSceneForRendering(liveScene, liveVoxelPolydata, liveHashBlockGrid);

	//Individual voxel shape
	vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
	sphere->SetThetaResolution(6);
	sphere->SetPhiResolution(6);
	sphere->SetRadius(maxVoxelDrawSize / 2);
	sphere->Update();

	//Voxel hash block shape
	vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
	cube->SetBounds(0, SDF_BLOCK_SIZE, 0, SDF_BLOCK_SIZE, 0, SDF_BLOCK_SIZE);

	//mappers for voxel blocks
	auto SetUpSceneHashBlockMapper = [&cube](vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                                         vtkSmartPointer<vtkPolyData>& pointsPolydata) {
		mapper->SetInputData(pointsPolydata);
		mapper->SetSourceConnection(cube->GetOutputPort());
		mapper->ScalarVisibilityOff();
		mapper->ScalingOff();
		mapper->SetScaleFactor(1.0);
	};
	vtkSmartPointer<vtkGlyph3DMapper> canonicalHashBlockMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	vtkSmartPointer<vtkGlyph3DMapper> liveHashBlockMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	SetUpSceneHashBlockMapper(canonicalHashBlockMapper, canonicalHashBlockGrid);
	SetUpSceneHashBlockMapper(liveHashBlockMapper, liveHashBlockGrid);


	// Create the color maps
	auto SetUpSDFColorLookupTable = [](vtkSmartPointer<vtkLookupTable>& table,
	                                   const double rgbaFirstColor[4], const double rgbaSecondColor[4]) {
		table->SetTableRange(0.0, 1.0);
		table->SetNumberOfTableValues(2);
		table->SetNumberOfColors(2);
		table->SetTableValue(0, rgbaFirstColor);
		table->SetTableValue(1, rgbaSecondColor);
		table->Build();
	};

	vtkSmartPointer<vtkLookupTable> canonicalColorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	vtkSmartPointer<vtkLookupTable> liveColorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	SetUpSDFColorLookupTable(canonicalColorLookupTable, canonicalNegativeSDFColor, canonicalPositiveSDFColor);
	SetUpSDFColorLookupTable(liveColorLookupTable, liveNegativeSDFColor, livePositiveSDFColor);



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
	auto SetUpGlyph = [&sphere](vtkSmartPointer<vtkPolyData>& polydata, vtkSmartPointer<vtkGlyph3D>& glyph) {
		glyph->SetInputData(polydata);
		glyph->SetSourceConnection(sphere->GetOutputPort());
		glyph->ScalingOn();
		glyph->ClampingOff();
		glyph->SetScaleModeToScaleByScalar();
		glyph->SetScaleFactor(1.0);
		glyph->SetColorModeToColorByScalar();
	};

	vtkSmartPointer<vtkGlyph3D> canonicalGlyph = vtkSmartPointer<vtkGlyph3D>::New();
	vtkSmartPointer<vtkGlyph3D> liveGlyph = vtkSmartPointer<vtkGlyph3D>::New();
	SetUpGlyph(canonicalVoxelPolydata, canonicalGlyph);
	SetUpGlyph(liveVoxelPolydata, liveGlyph);

	// set up mappers
	auto SetUpSceneVoxelMapper = [](vtkSmartPointer<vtkPolyDataMapper>& mapper,
							   vtkSmartPointer<vtkLookupTable>& table,
							   vtkSmartPointer<vtkGlyph3D>& glyph) {
		mapper->SetInputConnection(glyph->GetOutputPort());
		mapper->ScalarVisibilityOn();
		mapper->SetColorModeToMapScalars();
		mapper->SetLookupTable(table);
		mapper->ColorByArrayComponent("data", 1);
	};
	vtkSmartPointer<vtkPolyDataMapper> canonicalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkPolyDataMapper> liveMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	SetUpSceneVoxelMapper(canonicalMapper, canonicalColorLookupTable, canonicalGlyph);
	SetUpSceneVoxelMapper(liveMapper, liveColorLookupTable, liveGlyph);
#else
#ifdef USE_CLIPPING
	// set up scene voxel mappers
	auto SetUpSceneVoxelMapper = [&sphere](vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                                       vtkSmartPointer<vtkLookupTable>& table,
	                                       vtkSmartPointer<vtkExtractPolyDataGeometry> extractor) {
		mapper->SetInputConnection(extractor->GetOutputPort());
		mapper->SetSourceConnection(sphere->GetOutputPort());
		mapper->SetLookupTable(table);
		mapper->ScalingOn();
		mapper->ScalarVisibilityOn();
		mapper->SetScalarModeToUsePointData();
		mapper->SetColorModeToMapScalars();
		mapper->SetScaleModeToScaleByMagnitude();
		mapper->SetScaleArray(scalePointAttributeName);
		mapper->Update();
	};
	canonicalMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	liveMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	SetUpSceneVoxelMapper(canonicalMapper, canonicalColorLookupTable, canonicalExtractor);
	SetUpSceneVoxelMapper(liveMapper, liveColorLookupTable, liveExtractor);

#else
	// set up mappers
	auto SetUpSceneVoxelMapper = [&sphere](vtkSmartPointer<vtkGlyph3DMapper>& mapper,
									  vtkSmartPointer<vtkLookupTable>& table,
									  vtkSmartPointer<vtkPolyData>& pointsPolydata) {
		mapper->SetInputData(pointsPolydata);
		mapper->SetSourceConnection(sphere->GetOutputPort());
		mapper->SetLookupTable(table);
		mapper->ScalingOn();
		mapper->ScalarVisibilityOn();
		mapper->SetScalarModeToUsePointData();
		mapper->SetColorModeToMapScalars();
		mapper->SetScaleModeToScaleByMagnitude();
		mapper->SetScaleArray(scalePointAttributeName);
	};
	canonicalMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	liveMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	SetUpSceneVoxelMapper(canonicalMapper, canonicalColorLookupTable, canonicalVoxelPolydata);
	SetUpSceneVoxelMapper(liveMapper, liveColorLookupTable, liveVoxelPolydata);
#endif
#endif // ndef USE_CPU_GLYPH
	canonicalVoxelActor = vtkSmartPointer<vtkActor>::New();//TODO: move to constructor
	canonicalVoxelActor->SetMapper(canonicalMapper);
	liveVoxelActor = vtkSmartPointer<vtkActor>::New();//TODO: move to constructor
	liveVoxelActor->SetMapper(liveMapper);

	canonicalHashBlockActor = vtkSmartPointer<vtkActor>::New();//TODO: move to constructor
	canonicalHashBlockActor->SetMapper(canonicalHashBlockMapper);
	canonicalHashBlockActor->GetProperty()->SetRepresentationToWireframe();
	canonicalHashBlockActor->GetProperty()->SetColor(0.286, 0.623, 0.854);
	canonicalHashBlockActor->VisibilityOff();

	liveHashBlockActor = vtkSmartPointer<vtkActor>::New();//TODO: move to constructor
	liveHashBlockActor->SetMapper(liveHashBlockMapper);
	liveHashBlockActor->GetProperty()->SetRepresentationToWireframe();
	liveHashBlockActor->GetProperty()->SetColor(0.537, 0.819, 0.631);
	liveHashBlockActor->GetProperty()->SetEdgeColor(0.537, 0.819, 0.631);
	liveHashBlockActor->VisibilityOff();


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
	delete canonicalScene;
	delete liveScene;
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


template<typename TVoxel>
void
SDFViz::PrepareSceneForRendering(ITMScene<TVoxel, ITMVoxelIndex>* scene, vtkSmartPointer<vtkPolyData>& polydata,
                                 vtkSmartPointer<vtkPolyData>& hashBlockGrid) {
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> hashBlockPoints = vtkSmartPointer<vtkPoints>::New();

#ifdef USE_CPU_GLYPH //_DEBUG
	//holds point data attribute
	vtkSmartPointer<vtkFloatArray> pointAttributeData = vtkSmartPointer<vtkFloatArray>::New();
	pointAttributeData->SetNumberOfComponents(2);
	pointAttributeData->SetName("data");
#else
	//holds color for each voxel
	vtkSmartPointer<vtkFloatArray> colorAttribute = vtkSmartPointer<vtkFloatArray>::New();
	colorAttribute->SetName(colorPointAttributeName);

	//holds scale of each voxel
	vtkSmartPointer<vtkFloatArray> scaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	scaleAttribute->SetName(scalePointAttributeName);
#endif

	TVoxel* voxelBlocks = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;

	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];

		//skip unfilled hash
		if (currentHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		Vector3i currentBlockPositionVoxels = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		//_DEBUG
		//const double halfBlock = SDF_BLOCK_SIZE * maxVoxelDrawSize / 2;
		const double halfBlock = 0.0;

		//draw hash block
		hashBlockPoints->InsertNextPoint(currentBlockPositionVoxels.x + halfBlock,
		                                 currentBlockPositionVoxels.y + halfBlock,
		                                 currentBlockPositionVoxels.z + halfBlock);

		TVoxel* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {

					Vector3i originalPositionVoxels = currentBlockPositionVoxels + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxel& voxel = localVoxelBlock[locId];
					//Vector3f projectedPositionVoxels = originalPositionVoxels.toFloat() + voxel.warp_t;
					float voxelScale = 1.0f - std::abs(voxel.sdf);
					float voxelColor = (voxel.sdf + 1.0f) * 0.5f;

					points->InsertNextPoint(maxVoxelDrawSize * originalPositionVoxels.x,
					                        maxVoxelDrawSize * originalPositionVoxels.y,
					                        maxVoxelDrawSize * originalPositionVoxels.z);
#ifdef USE_CPU_GLYPH //_DEBUG
					float nextDataValue[2] = {voxelScale, voxelColor};
					pointAttributeData->InsertNextTypedTuple(nextDataValue);
#endif
					scaleAttribute->InsertNextValue(voxelScale);
					colorAttribute->InsertNextValue(voxelColor);
				}
			}
		}
	}

	std::cout << "Scene voxel count: " << points->GetNumberOfPoints() << std::endl;
	std::cout << "Allocated hash block count: " << hashBlockPoints->GetNumberOfPoints() << std::endl;

	//Points pipeline
	polydata->SetPoints(points);
	//TODO: pointAttributeData is candidate for removal (by GitHub:Algomorph)
#ifdef USE_CPU_GLYPH //_DEBUG
	polydata->GetPointData()->AddArray(pointAttributeData);
	polydata->GetPointData()->SetActiveScalars("data");
#else
	polydata->GetPointData()->AddArray(colorAttribute);
	polydata->GetPointData()->AddArray(scaleAttribute);
	polydata->GetPointData()->SetActiveScalars(colorPointAttributeName);
#endif
	hashBlockGrid->SetPoints(hashBlockPoints);
}


void SDFViz::TestPointShift() {
	vtkPoints* points = canonicalVoxelPolydata->GetPoints();
	double ave[3];
	int pointCount = 0;
	for (int iPoint = 0; iPoint < points->GetNumberOfPoints(); iPoint++) {
		double point[3];
		points->GetPoint(iPoint, point);
		ave[0] += point[0];
		ave[1] += point[1];
		ave[2] += point[2];
		point[0] += 10.0;
		point[1] += 10.0;
		point[2] += 10.0;
		points->SetPoint(iPoint, point);
		pointCount++;
	}
	ave[0] /= pointCount;
	ave[1] /= pointCount;
	ave[2] /= pointCount;
	std::cout << "Average point: " << ave[0] << ", " << ave[1] << ", " << ave[2] << std::endl;

	canonicalVoxelPolydata->Modified();
	renderWindow->Render();
}


bool SDFViz::NextWarps() {
	if(!sceneLogger->BufferNextWarpState(this->warpBuffer->GetVoidPointer(0))){
		return false;
	}

	vtkPoints* voxels = canonicalVoxelPolydata->GetPoints();
	auto* pointRawData = reinterpret_cast<float*>(voxels->GetVoidPointer(0));
	auto* warpRawData = reinterpret_cast<float*>(this->warpBuffer->GetVoidPointer(0));
	const auto pointCount = static_cast<const int>(voxels->GetNumberOfPoints());
	for(int iVoxel = 0; iVoxel < pointCount; iVoxel++){
		//use 1st 3-float field out of 2 for the warp buffer entry
		pointRawData[iVoxel*3 + 0] += warpRawData[iVoxel*6 + 0];
		pointRawData[iVoxel*3 + 1] += warpRawData[iVoxel*6 + 1];
		pointRawData[iVoxel*3 + 2] += warpRawData[iVoxel*6 + 2];
	}
	canonicalVoxelPolydata->Modified();
	renderWindow->Render();
	return true;
}

bool SDFViz::PreviousWarps() {
	//TODO
	return false;
}

void SDFViz::DrawLegend() {
	vtkSmartPointer<vtkLegendBoxActor> legend = vtkSmartPointer<vtkLegendBoxActor>::New();
	legend->SetNumberOfEntries(4);
	vtkSmartPointer<vtkSphereSource> legendSphere = vtkSmartPointer<vtkSphereSource>::New();
	legendSphere->Update();

	//set up legend entries
	legend->SetEntry(0, legendSphere->GetOutput(), "Positive Canonical", (double*) canonicalPositiveSDFColor);
	legend->SetEntry(1, legendSphere->GetOutput(), "Negative Canonical", (double*) canonicalNegativeSDFColor);
	legend->SetEntry(2, legendSphere->GetOutput(), "Positive Live", (double*) livePositiveSDFColor);
	legend->SetEntry(3, legendSphere->GetOutput(), "Negative Live", (double*) liveNegativeSDFColor);

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
	if(!sceneLogger->GetScenesLoaded()){
		DIEWITHEXCEPTION("Scenes not yet loaded, cannot initialize WarpBuffers");
	}
	this->warpBuffer = vtkSmartPointer<vtkFloatArray>::New();
	warpBuffer->SetNumberOfComponents(3);
	warpBuffer->SetNumberOfTuples(sceneLogger->GetVoxelCount()*2);
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
		}
		if (key == "v"){
			//toggle voxel blocks visibility
			if(rwi->GetAltKey()){
				parent->ToggleLiveVoxelVisibility();
			}else{
				parent->ToggleCanonicalVoxelVisibility();
			}
		}
		if (key == "h"){
			//toggle hash blocks visibility
			if(rwi->GetAltKey()){
				parent->ToggleLiveHashBlockVisibility();
			}else{
				parent->ToggleCanonicalHashBlockVisibility();
			}
		}

		if (key == "t") {
			parent->TestPointShift();
			std::cout << "Point shift test conducted." << std::endl;
		}
		if (key == "Right") {
			std::cout << "Loading next iteration warp & updates." << std::endl;
			if (parent->NextWarps()) {
				std::cout << "Next warps loaded and display updated." << std::endl;
			} else {
				std::cout << "Could not load next iteration warp & updates." << std::endl;
			}

		}
		if (key == "Left") {
			std::cout << "Loading previous iteration warp & updates." << std::endl;
			if (parent->PreviousWarps()) {
				std::cout << "Previous warps loaded and display updated." << std::endl;
			} else {
				std::cout << "Could not load previous iteration warp & updates." << std::endl;
			}
		}
	}
	std::cout << "Key symbol: " << key << std::endl;

	// Forward events
	vtkInteractorStyleTrackballCamera::OnKeyPress();

}
