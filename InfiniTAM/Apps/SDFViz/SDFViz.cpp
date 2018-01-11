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
#include <vtk-8.1/vtkLegendBoxActor.h>
#include <vtk-8.1/vtkNamedColors.h>

//local
#include "../../ITMLib/Utils/ITMSceneWarpFileIO.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"
#include "../../ITMLib/Utils/ITMSceneStatisticsCalculator.h"

const double SDFViz::maxVoxelDrawSize = 1.0;
const double SDFViz::canonicalNegativeSDFColor[4] = {0.141, 0.215, 0.396, 1.0};
const double SDFViz::canonicalPositiveSDFColor[4] = {0.717, 0.788, 0.960, 1.0};
const double SDFViz::liveNegativeSDFColor[4] = {0.101, 0.219, 0.125, 0.6};
const double SDFViz::livePositiveSDFColor[4] = {0.717, 0.882, 0.749, 0.6};

int SDFViz::run() {
	//read scenes from disk
	sceneLogger->LoadScenes();

	ITMSceneStatisticsCalculator<ITMVoxelCanonical, ITMVoxelIndex> statCalculator;
	Vector3i minPoint, maxPoint;
	statCalculator.ComputeSceneVoxelBounds(canonicalScene, minPoint, maxPoint);
	std::cout << "Voxel ranges ( min x,y,z; max x,y,z): " << minPoint << "; " << maxPoint << std::endl;

	GenerateInitialScenePoints(canonicalScene, canonicalPoints, canonicalPolydata);
	GenerateInitialScenePoints(liveScene, livePoints, livePolydata);

	//Individual voxel shape
	vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
	sphere->SetThetaResolution(6);
	sphere->SetPhiResolution(6);
	sphere->SetRadius(maxVoxelDrawSize / 2);
	sphere->Update();

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

#ifdef USE_CPU_GLYPH
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
	SetUpGlyph(canonicalPolydata, canonicalGlyph);
	SetUpGlyph(livePolydata, liveGlyph);



	// set up mappers
	auto SetUpSceneMapper = [](vtkSmartPointer<vtkPolyDataMapper>& mapper,
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
	SetUpSceneMapper(canonicalMapper, canonicalColorLookupTable, canonicalGlyph);
	SetUpSceneMapper(liveMapper, liveColorLookupTable, liveGlyph);
#else

#endif

	vtkSmartPointer<vtkActor> canonicalActor = vtkSmartPointer<vtkActor>::New();
	canonicalActor->SetMapper(canonicalMapper);
	vtkSmartPointer<vtkActor> liveActor = vtkSmartPointer<vtkActor>::New();
	liveActor->SetMapper(liveMapper);

	renderer->AddActor(canonicalActor);
	renderer->AddActor(liveActor);
	renderer->GetActiveCamera()->SetPosition(7.0, 22.0, -72.66);
	renderer->GetActiveCamera()->SetFocalPoint(0.5, -40.5, 230.0);
	renderer->GetActiveCamera()->SetViewUp(0.0, -1.0, 0.0);
	//renderer->ResetCamera();//used when need to choose new better initial camera pose manually

	renderWindow->Render();
	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

SDFViz::SDFViz() :
		minAllowedPoint(-100, -150, 0),
		maxAllowedPoint(200, 50, 300),
		canonicalPoints(vtkSmartPointer<vtkPoints>::New()),
		livePoints(vtkSmartPointer<vtkPoints>::New()),
		canonicalPolydata(vtkSmartPointer<vtkPolyData>::New()),
		livePolydata(vtkSmartPointer<vtkPolyData>::New()) {
	auto* settings = new ITMLibSettings();
	MemoryDeviceType memoryType = settings->GetMemoryType();
	canonicalScene = new ITMScene<ITMVoxelCanonical, ITMVoxelIndex>(
			&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED, memoryType);
	liveScene = new ITMScene<ITMVoxelLive, ITMVoxelIndex>(
			&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED, memoryType);
	sceneLogger = new ITMSceneWarpFileIO<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(
			"/media/algomorph/Data/4dmseg/Killing/scene", canonicalScene, liveScene);
	InitializeRendering();
	DrawLegend();
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
	renderWindow->AddRenderer(renderer);

	renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	vtkSmartPointer<KeyPressInteractorStyle> interactorStyle = vtkSmartPointer<KeyPressInteractorStyle>::New();
	interactorStyle->parent = this;
	interactorStyle->SetMouseWheelMotionFactor(0.05);

	renderWindowInteractor->SetInteractorStyle(interactorStyle);
	renderWindowInteractor->SetRenderWindow(renderWindow);

}

bool SDFViz::HashBlockIsAtLeastPartiallyWithinBounds(Vector3i hashBlockPositionVoxels) {
	Vector3i cornerOfBlockCoordinates(hashBlockPositionVoxels.x + SDF_BLOCK_SIZE - 1,
	                                  hashBlockPositionVoxels.y + SDF_BLOCK_SIZE - 1,
	                                  hashBlockPositionVoxels.z + SDF_BLOCK_SIZE - 1);
	return !(hashBlockPositionVoxels.x > maxAllowedPoint.x ||
	         hashBlockPositionVoxels.y > maxAllowedPoint.y ||
	         hashBlockPositionVoxels.z > maxAllowedPoint.z ||
	         cornerOfBlockCoordinates.x < minAllowedPoint.x ||
	         cornerOfBlockCoordinates.y < minAllowedPoint.y ||
	         cornerOfBlockCoordinates.z < minAllowedPoint.z);
}


template<typename TVoxel>
void SDFViz::GenerateInitialScenePoints(ITMScene<TVoxel, ITMVoxelIndex>* scene, vtkSmartPointer<vtkPoints>& points,
                                        vtkSmartPointer<vtkPolyData>& polydata) {
#ifdef USE_CPU_GLYPH
	//holds point data attribute
	vtkSmartPointer<vtkFloatArray> pointAttributeData = vtkSmartPointer<vtkFloatArray>::New();
	pointAttributeData->SetNumberOfComponents(2);
	pointAttributeData->SetName("data");
#else
	//holds color for each voxel
	vtkSmartPointer<vtkFloatArray> colorAttribute = vtkSmartPointer<vtkFloatArray>::New();
	colorAttribute->SetName("color");

	//holds scale of each voxel
	vtkSmartPointer<vtkFloatArray> scaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	scaleAttribute->SetName("scale");
#endif
	int pointCount = 0;//stat logging
	TVoxel* voxelBlocks = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;

	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];

		//skip unfilled hash
		if (currentHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		Vector3i currentBlockPositionVoxels = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		if (!HashBlockIsAtLeastPartiallyWithinBounds(currentBlockPositionVoxels)) {
			continue;
		}

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
#ifdef USE_CPU_GLYPH
					float nextDataValue[2] = {voxelScale, voxelColor};
					pointAttributeData->InsertNextTypedTuple(nextDataValue);
#endif
					scaleAttribute->InsertNextValue(voxelScale);
					colorAttribute->InsertNextValue(voxelColor);
					pointCount++;
				}
			}
		}
	}

	std::cout << "Scene points for visualization: " << pointCount << std::endl;

	//Points pipeline
	polydata->SetPoints(points);
	//TODO: pointAttributeData is candidate for removal (by GitHub:Algomorph)
#ifdef USE_CPU_GLYPH
	polydata->GetPointData()->AddArray(pointAttributeData);
	polydata->GetPointData()->SetActiveScalars("data");
#else
	polydata->GetPointData()->AddArray(colorAttribute);
	polydata->GetPointData()->AddArray(scaleAttribute);
	polydata->GetPointData()->SetActiveScalars("color");
#endif
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
	legend->GetPositionCoordinate()->SetValue(.5, -1.0);//x = .5
	legend->GetPosition2Coordinate()->SetCoordinateSystemToView();
	legend->GetPosition2Coordinate()->SetValue(1.0, -0.5); //y = -.5

	//set up legend background
	vtkSmartPointer<vtkNamedColors> colors =
			vtkSmartPointer<vtkNamedColors>::New();
	legend->UseBackgroundOn();
	double background[4];
	colors->GetColor("warm_grey", background);
	legend->SetBackgroundColor(background);

	renderer->AddActor(legend);
}


vtkStandardNewMacro(KeyPressInteractorStyle);

void KeyPressInteractorStyle::OnKeyPress() {

	// Get the keypress
	vtkRenderWindowInteractor* rwi = this->Interactor;
	std::string key = rwi->GetKeySym();

	if (key == "c" && parent != nullptr) {
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
		std::cout << "  Current up-vector: " << xUpVector << ", " << yUpVector << ", " << zUpVector << std::endl;
		std::cout.flush();
	}

	// Forward events
	vtkInteractorStyleTrackballCamera::OnKeyPress();

}
