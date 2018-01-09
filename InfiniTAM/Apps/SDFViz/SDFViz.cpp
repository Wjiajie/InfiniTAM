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
#include <vtkNamedColors.h>
#include <vtkSphereSource.h>
#include <vtkBridgeDataSet.h>
#include <vtkGlyph3D.h>
#include <vtkFloatArray.h>
#include <vtkGlyph3DMapper.h>
#include <vtkLookupTable.h>

//local
#include "../../ITMLib/Utils/ITMSceneWarpFileIO.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"
#include "../../ITMLib/Utils/ITMSceneStatisticsCalculator.h"

int SDFViz::run() {


	//read scenes from disk
	sceneLogger->LoadScenes();

	ITMVoxelCanonical* voxelBlocks = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	int noTotalEntries = canonicalScene->index.noTotalEntries;
	ITMVoxelIndex::IndexCache canonicalCache;

	//holds raw point data
	vtkSmartPointer<vtkPoints> points =
			vtkSmartPointer<vtkPoints>::New();

//	holds point color attribute
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("color");
	//holds point scale attribute
	vtkSmartPointer<vtkFloatArray> colorsFloat = vtkSmartPointer<vtkFloatArray>::New();
	colorsFloat->SetName("color");

	// Create the color map
	vtkSmartPointer<vtkLookupTable> colorLookupTable =
			vtkSmartPointer<vtkLookupTable>::New();
	colorLookupTable->SetTableRange(0.0, 1.0);
	//colorLookupTable->Set
	colorLookupTable->Build();

	//holds point scale attribute
	vtkSmartPointer<vtkFloatArray> scales = vtkSmartPointer<vtkFloatArray>::New();
	scales->SetName("scale");

	ITMSceneStatisticsCalculator<ITMVoxelCanonical, ITMVoxelIndex> statCalculator;
	Vector3i minPoint, maxPoint;
	statCalculator.ComputeSceneVoxelBounds(canonicalScene, minPoint, maxPoint);
	std::cout << "Voxel ranges ( min x,y,z; max x,y,z): " << minPoint << "; " << maxPoint << std::endl;

	//(temporary) restrict scene bounds to these coordinates:
	Vector3i minAllowedPoint(-100, -150, 0);
	Vector3i maxAllowedPoint(200, 50, 300);

	double maxVoxelDrawSize = 1.0;

	// Set up colorsFloat
	vtkSmartPointer<vtkNamedColors> namedColors =
			vtkSmartPointer<vtkNamedColors>::New();

	unsigned char subtleGreen[3] = {100, 220, 75};
	unsigned char subtleRed[3] = {220, 100, 75};
	int pointCount = 0;


	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];

		//skip unfilled hash
		if (currentHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		Vector3i currentBlockPositionVoxels = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		Vector3i cornerOfBlockCoordinates(currentBlockPositionVoxels.x + SDF_BLOCK_SIZE - 1,
		                                  currentBlockPositionVoxels.y + SDF_BLOCK_SIZE - 1,
		                                  currentBlockPositionVoxels.z + SDF_BLOCK_SIZE - 1);
		if (currentBlockPositionVoxels.x > maxAllowedPoint.x ||
		    currentBlockPositionVoxels.y > maxAllowedPoint.y ||
		    currentBlockPositionVoxels.z > maxAllowedPoint.z ||
			cornerOfBlockCoordinates.x < minAllowedPoint.x ||
			cornerOfBlockCoordinates.y < minAllowedPoint.y ||
			cornerOfBlockCoordinates.z < minAllowedPoint.z){
			continue;//out of bounds
		}

		ITMVoxelCanonical* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPositionVoxels = currentBlockPositionVoxels + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					ITMVoxelCanonical& voxel = localVoxelBlock[locId];
					Vector3f projectedPositionVoxels = originalPositionVoxels.toFloat() + voxel.warp_t;
					bool isNegative = voxel.sdf < 0.0f;
					float voxelScale = 1.0f - std::abs(voxel.sdf);
					float voxelColor = (voxel.sdf + 1.0f) * 0.5f;
					//double voxelScale = 0.2 + (voxel.sdf + 1.0) * 0.4;
					points->InsertNextPoint(maxVoxelDrawSize * originalPositionVoxels.x,
					                        maxVoxelDrawSize * originalPositionVoxels.y,
					                        maxVoxelDrawSize * originalPositionVoxels.z);
					scales->InsertNextValue(voxelScale);
					colorsFloat->InsertNextValue(voxelColor);

//					if (isNegative) {
//						unsigned char sdfColor[4] = {static_cast<unsigned char>(subtleRed[0] * voxelScale),
//						                             static_cast<unsigned char>(subtleRed[1] * voxelScale),
//						                             static_cast<unsigned char>(subtleRed[2] * voxelScale),
//						                             static_cast<unsigned char>(255.0 * voxelScale)};
//						colorsFloat->InsertNextTypedTuple(sdfColor);
//					} else {
//						unsigned char sdfColor[4] = {static_cast<unsigned char>(subtleGreen[0] * voxelScale),
//						                             static_cast<unsigned char>(subtleGreen[1] * voxelScale),
//						                             static_cast<unsigned char>(subtleGreen[2] * voxelScale),
//						                             static_cast<unsigned char>(255.0 * voxelScale)};
//						colorsFloat->InsertNextTypedTuple(sdfColor);
//					};
					colors->InsertNextTypedTuple(subtleGreen);

					pointCount++;
				}


			}
		}
	}

	std::cout << "Total points in scene: " << pointCount <<std::endl;

	//Points pipeline
	vtkSmartPointer<vtkPolyData> pointsPolydada = vtkSmartPointer<vtkPolyData>::New();
	pointsPolydada->SetPoints(points);

	pointsPolydada->GetPointData()->AddArray(colorsFloat);
	pointsPolydada->GetPointData()->AddArray(scales);
	pointsPolydada->GetPointData()->SetActiveScalars("scale");



	//Individual voxel shape
	vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
	sphere->SetThetaResolution(6);
	sphere->SetPhiResolution(6);
	sphere->SetRadius(maxVoxelDrawSize/2);
	sphere->Update();


// Visualization
	vtkSmartPointer<vtkGlyph3DMapper> mapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	mapper->SetSourceConnection(sphere->GetOutputPort());
	mapper->SetInputData(pointsPolydada);

	mapper->ScalingOn();

	mapper->SetScaleModeToScaleByMagnitude();
	mapper->SetColorModeToMapScalars();
	mapper->SetScalarModeToUsePointData();

	mapper->SetScaleFactor(1.0);
	mapper->SetScaleArray("scale");
	mapper->SelectColorArray("color");

	//mapper->SetColorModeToDirectScalars();



	//mapper->SetInputConnection(filter->GetOutputPort());


	vtkSmartPointer<vtkActor> actor =vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	//actor->GetProperty()->SetPointSize(8);
	renderer->AddActor(actor);

//	renderer->GetActiveCamera()->SetPosition(-85, 200, -460.);
//	renderer->GetActiveCamera()->SetFocalPoint(85, -40, 460);
//	renderer->GetActiveCamera()->SetViewUp(0.0, -1.0, 0.0);
    renderer->ResetCamera();
//renderer->GetActiveCamera()->Zoom(1.5);

	renderWindow->Render();

	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

SDFViz::SDFViz() {
	ITMLibSettings* settings = new ITMLibSettings();
	MemoryDeviceType memoryType = settings->GetMemoryType();
	canonicalScene = new ITMScene<ITMVoxelCanonical, ITMVoxelIndex>(
			&settings->sceneParams, settings->swappingMode ==ITMLibSettings::SWAPPINGMODE_ENABLED,memoryType);
	liveScene = new ITMScene<ITMVoxelLive, ITMVoxelIndex>(
			&settings->sceneParams, settings->swappingMode ==ITMLibSettings::SWAPPINGMODE_ENABLED,memoryType);
	sceneLogger = new ITMSceneWarpFileIO<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(
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
	vtkSmartPointer<KeyPressInteractorStyle> interactorStyle = vtkSmartPointer<KeyPressInteractorStyle>::New();
	interactorStyle->parent = this;
	interactorStyle->SetMouseWheelMotionFactor(0.05);

	renderWindowInteractor->SetInteractorStyle(interactorStyle);
	renderWindowInteractor->SetRenderWindow(renderWindow);

}

vtkStandardNewMacro(KeyPressInteractorStyle);

void KeyPressInteractorStyle::OnKeyPress() {

	{
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
}
