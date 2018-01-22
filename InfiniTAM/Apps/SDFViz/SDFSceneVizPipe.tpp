//  ================================================================
//  Created by Gregory Kramida on 1/22/18.
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
//VTK
#include <vtkActor.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>

//Local
#include "SDFSceneVizPipe.h"

//ITMLib
#include "../../ITMLib/Utils/ITMLibSettings.h"

template<typename TVoxel, typename TIndex>
const double SDFSceneVizPipe<TVoxel, TIndex>::maxVoxelDrawSize = 1.0;
template<typename TVoxel, typename TIndex>
const char* SDFSceneVizPipe<TVoxel, TIndex>::colorPointAttributeName = "color";
template<typename TVoxel, typename TIndex>
const char* SDFSceneVizPipe<TVoxel, TIndex>::scalePointAttributeName = "scale";

template<typename TVoxel, typename TIndex>
SDFSceneVizPipe<TVoxel, TIndex>::SDFSceneVizPipe(double* negativeSDFVoxelColor, double* positiveSDFVoxelColor, double* hashBlockEdgeColor)
		:
		initialPoints(vtkSmartPointer<vtkPoints>::New()),
		voxelPolydata(vtkSmartPointer<vtkPolyData>::New()),
		voxelActor(vtkSmartPointer<vtkActor>::New()),
		hashBlockGrid(vtkSmartPointer<vtkPolyData>::New()),
		hashBlockActor(vtkSmartPointer<vtkActor>::New()){
	auto* settings = new ITMLibSettings();
	scene = new ITMScene<TVoxel, TIndex>(
			&settings->sceneParams, settings->swappingMode ==
					ITMLibSettings::SWAPPINGMODE_ENABLED, settings->GetMemoryType());
	delete settings;


}
template<typename TVoxel, typename TIndex>
SDFSceneVizPipe<TVoxel, TIndex>::~SDFSceneVizPipe() {
	delete scene;
}

template<typename TVoxel, typename TIndex>
ITMScene<TVoxel, TIndex>* SDFSceneVizPipe<TVoxel, TIndex>::GetScene() {
	return scene;
}

template<typename TVoxel, typename TIndex>
void SDFSceneVizPipe<TVoxel, TIndex>::PrepareSceneForRendering() {
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
	voxelPolydata->SetPoints(points);
	//TODO: pointAttributeData is candidate for removal (by GitHub:Algomorph)
#ifdef USE_CPU_GLYPH //_DEBUG
	voxelPolydata->GetPointData()->AddArray(pointAttributeData);
	voxelPolydata->GetPointData()->SetActiveScalars("data");
#else
	voxelPolydata->GetPointData()->AddArray(colorAttribute);
	voxelPolydata->GetPointData()->AddArray(scaleAttribute);
	voxelPolydata->GetPointData()->SetActiveScalars(colorPointAttributeName);
#endif
	hashBlockGrid->SetPoints(hashBlockPoints);
}

template<typename TVoxel, typename TIndex>
void SDFSceneVizPipe<TVoxel, TIndex>::PreparePipeline(vtkAlgorithmOutput* voxelSourceGeometry, vtkAlgorithmOutput* hashBlockSourceGeometry) {
	PrepareSceneForRendering();
	initialPoints->DeepCopy(voxelPolydata->GetPoints());

	// Set up hash block visualization
	vtkSmartPointer<vtkGlyph3DMapper> canonicalHashBlockMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	SetUpSceneHashBlockMapper(cube->GetOutputPort(), canonicalHashBlockMapper, canonicalHashBlockGrid);

	// Create the color maps
	vtkSmartPointer<vtkLookupTable> canonicalColorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	SetUpSDFColorLookupTable(canonicalColorLookupTable, canonicalNegativeSDFVoxelColor, canonicalPositiveSDFVoxelColor);


	voxelActor->SetMapper(voxelMapper);
	hashBlockActor->SetMapper(hashBlockMapper);
	hashBlockActor->GetProperty()->SetRepresentationToWireframe();
	hashBlockActor->GetProperty()->SetColor(hashBlockEdgeColor);
	hashBlockActor->VisibilityOff();
}
