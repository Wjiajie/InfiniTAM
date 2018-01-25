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
#include <vtk-8.1/vtkGlyph3DMapper.h>
#include <vtk-8.1/vtkBox.h>
#include <vtkProperty.h>
#include <vtkLookupTable.h>
#include <vtkPolyDataMapper.h>

//Local
#include "SDFSceneVizPipe.h"

//ITMLib
#include "../../ITMLib/Utils/ITMLibSettings.h"
#include "../../ITMLib/Utils/ITMSceneStatisticsCalculator.h"

template<typename TVoxel, typename TIndex>
const double SDFSceneVizPipe<TVoxel, TIndex>::maxVoxelDrawSize = 1.0;
template<typename TVoxel, typename TIndex>
const char* SDFSceneVizPipe<TVoxel, TIndex>::colorPointAttributeName = "color";
template<typename TVoxel, typename TIndex>
const char* SDFSceneVizPipe<TVoxel, TIndex>::scalePointAttributeName = "scale";

template<typename TVoxel, typename TIndex>
SDFSceneVizPipe<TVoxel, TIndex>::SDFSceneVizPipe(std::array<double, 4> negativeSDFVoxelColor,
                                                 std::array<double, 4> positiveSDFVoxelColor,
                                                 std::array<double, 3> hashBlockEdgeColor)
		:
		voxelPolydata(vtkSmartPointer<vtkPolyData>::New()),
		voxelColorLookupTable(vtkSmartPointer<vtkLookupTable>::New()),
		voxelMapper(vtkSmartPointer<vtkGlyph3DMapper>::New()),
		voxelActor(vtkSmartPointer<vtkActor>::New()),

		hashBlockGrid(vtkSmartPointer<vtkPolyData>::New()),
		hashBlockActor(vtkSmartPointer<vtkActor>::New()),
		hashBlockMapper(vtkSmartPointer<vtkGlyph3DMapper>::New()),

		negativeVoxelColor(negativeSDFVoxelColor),
		positiveVoxelColor(positiveSDFVoxelColor),
		hashBlockEdgeColor(hashBlockEdgeColor),

		minAllowedPoint(-100, -150, 0),
		maxAllowedPoint(200, 50, 300) {
	auto* settings = new ITMLibSettings();
	scene = new ITMScene<TVoxel, TIndex>(
			&settings->sceneParams, settings->swappingMode ==
			                        ITMLibSettings::SWAPPINGMODE_ENABLED, settings->GetMemoryType());
	// Create the color maps
	SetUpSDFColorLookupTable(voxelColorLookupTable,negativeSDFVoxelColor.data(),positiveSDFVoxelColor.data());
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
void SDFSceneVizPipe<TVoxel, TIndex>::PreparePointsForRendering() {
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
	PreparePointsForRendering();


	ITMSceneStatisticsCalculator<TVoxel, TIndex> statCalculator;

	statCalculator.ComputeVoxelBounds(scene, minPoint, maxPoint);
	std::cout << "Voxel ranges ( min x,y,z; max x,y,z): " << minPoint << "; " << maxPoint << std::endl;

	// Set up hash block visualization
	SetUpSceneHashBlockMapper(hashBlockSourceGeometry, hashBlockMapper, hashBlockGrid);

	//set up clipping for data
#define USE_CLIPPING
#ifdef USE_CLIPPING
	vtkSmartPointer<vtkBox> implicitClippingBox = vtkSmartPointer<vtkBox>::New();

	implicitClippingBox->SetBounds(minAllowedPoint.x * maxVoxelDrawSize, maxAllowedPoint.x * maxVoxelDrawSize,
	                               minAllowedPoint.y * maxVoxelDrawSize, maxAllowedPoint.y * maxVoxelDrawSize,
	                               minAllowedPoint.z * maxVoxelDrawSize, maxAllowedPoint.z * maxVoxelDrawSize);

	vtkSmartPointer<vtkExtractPolyDataGeometry> voxelExtractor = vtkSmartPointer<vtkExtractPolyDataGeometry>::New();
	voxelExtractor->SetImplicitFunction(implicitClippingBox);
	voxelExtractor->SetInputData(voxelPolydata);
	voxelExtractor->ExtractInsideOn();
	voxelExtractor->Update();
#endif

#ifdef USE_CPU_GLYPH //_DEBUG
	// set up glyphs
	vtkSmartPointer<vtkGlyph3D> canonicalGlyph = vtkSmartPointer<vtkGlyph3D>::New();
	SetUpGlyph(sphere->getOutputPort(), canonicalVoxelPolydata, canonicalGlyph);
	SetUpSceneVoxelMapper(canonicalMapper, canonicalColorLookupTable, canonicalGlyph);
#else
#ifdef USE_CLIPPING
	// set up scene voxel mappers
	SetUpSceneVoxelMapper(voxelSourceGeometry, voxelMapper,voxelColorLookupTable, voxelExtractor);
#else
	// set up mappers
	vtkSmartPointer<vtkGlyph3DMapper> canonicalMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	vtkSmartPointer<vtkGlyph3DMapper> liveMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	SetUpSceneVoxelMapper(sphere->GetOutputPort(), canonicalMapper, canonicalColorLookupTable, canonicalVoxelPolydata);
#endif
#endif // ndef USE_CPU_GLYPH

	voxelActor->SetMapper(voxelMapper);

	hashBlockActor->SetMapper(hashBlockMapper);
	hashBlockActor->GetProperty()->SetRepresentationToWireframe();
	hashBlockActor->GetProperty()->SetColor(hashBlockEdgeColor.data());
	hashBlockActor->VisibilityOff();
}

template<typename TVoxel, typename TIndex>
void SDFSceneVizPipe<TVoxel, TIndex>::SetUpSceneHashBlockMapper(vtkAlgorithmOutput* sourceOutput,
                                                                vtkSmartPointer<vtkGlyph3DMapper>& mapper,
                                                                vtkSmartPointer<vtkPolyData>& pointsPolydata) {
	mapper->SetInputData(pointsPolydata);
	mapper->SetSourceConnection(sourceOutput);
	mapper->ScalarVisibilityOff();
	mapper->ScalingOff();
	mapper->SetScaleFactor(1.0);
}

template<typename TVoxel, typename TIndex>
void SDFSceneVizPipe<TVoxel, TIndex>::SetUpSDFColorLookupTable(vtkSmartPointer<vtkLookupTable>& table,
                                                               const double* rgbaFirstColor,
                                                               const double* rgbaSecondColor) {
	table->SetTableRange(0.0, 1.0);
	table->SetNumberOfTableValues(2);
	table->SetNumberOfColors(2);
	table->SetTableValue(0, rgbaFirstColor);
	table->SetTableValue(1, rgbaSecondColor);
	table->Build();
}

template<typename TVoxel, typename TIndex>
void
SDFSceneVizPipe<TVoxel, TIndex>::SetUpGlyph(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkPolyData>& polydata,
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
template<typename TVoxel, typename TIndex>
void SDFSceneVizPipe<TVoxel, TIndex>::SetUpSceneVoxelMapper(vtkSmartPointer<vtkPolyDataMapper>& mapper,
                                                            vtkSmartPointer<vtkLookupTable>& table,
                                                            vtkSmartPointer<vtkGlyph3D>& glyph) {
	mapper->SetInputConnection(glyph->GetOutputPort());
	mapper->ScalarVisibilityOn();
	mapper->SetColorModeToMapScalars();
	mapper->SetLookupTable(table);
	mapper->ColorByArrayComponent("data", 1);
}

//GPU glyph version with filtering
template<typename TVoxel, typename TIndex>
void SDFSceneVizPipe<TVoxel, TIndex>::SetUpSceneVoxelMapper(vtkAlgorithmOutput* sourceOutput,
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
template<typename TVoxel, typename TIndex>
void SDFSceneVizPipe<TVoxel, TIndex>::SetUpSceneVoxelMapper(
		vtkAlgorithmOutput* sourceOutput,
		vtkSmartPointer<vtkGlyph3DMapper>& mapper,
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

template<typename TVoxel, typename TIndex>
vtkSmartPointer<vtkActor>& SDFSceneVizPipe<TVoxel, TIndex>::GetVoxelActor() {
	return voxelActor;
}

template<typename TVoxel, typename TIndex>
vtkSmartPointer<vtkActor>& SDFSceneVizPipe<TVoxel, TIndex>::GetHashBlockActor() {
	return hashBlockActor;
}


