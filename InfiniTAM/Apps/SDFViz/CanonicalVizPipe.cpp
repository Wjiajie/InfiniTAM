//  ================================================================
//  Created by Gregory Kramida on 1/24/18.
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
#include <vtk-8.1/vtkFloatArray.h>
#include "vtkPolyData.h"
#include "vtkPointData.h"
#include "CanonicalVizPipe.h"

CanonicalVizPipe::CanonicalVizPipe(const std::array<double, 4>& negativeNonInterestVoxelColor,
                                   const std::array<double, 4>& positiveNonInterestVoxelColor,
                                   const std::array<double, 4>& negativeInterestVoxelColor,
                                   const std::array<double, 4>& positiveInterestVoxelColor,
                                   const std::array<double, 3>& hashBlockEdgeColor) :
		SDFSceneVizPipe<ITMVoxelCanonical, ITMVoxelIndex>(
				negativeNonInterestVoxelColor, positiveNonInterestVoxelColor, hashBlockEdgeColor),
		initialPoints(vtkSmartPointer<vtkPoints>::New()),
		negativeInterestVoxelColor(negativeInterestVoxelColor),
		positiveInterestVoxelColor(positiveInterestVoxelColor)
		{}

void CanonicalVizPipe::PreparePointsForRendering() {

	if (!interestRegionHashesAreSet) {
		DIEWITHEXCEPTION("Interest regions need to be set first");
	}

	vtkSmartPointer<vtkPoints> nonInterestVoxelPoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> interestVoxelPoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> hashBlockPoints = vtkSmartPointer<vtkPoints>::New();
#ifdef USE_CPU_GLYPH //_DEBUG
	//holds point data attribute
	vtkSmartPointer<vtkFloatArray> pointAttributeData = vtkSmartPointer<vtkFloatArray>::New();
	pointAttributeData->SetNumberOfComponents(2);
	pointAttributeData->SetName("data");
#else
	//holds color for each voxel
	vtkSmartPointer<vtkFloatArray> nonInterestColorAttribute = vtkSmartPointer<vtkFloatArray>::New();
	nonInterestColorAttribute->SetName(colorPointAttributeName);
	//holds scale of each voxel
	vtkSmartPointer<vtkFloatArray> nonInterestScaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	nonInterestScaleAttribute->SetName(scalePointAttributeName);
	//holds color for each voxel
	vtkSmartPointer<vtkFloatArray> interestColorAttribute = vtkSmartPointer<vtkFloatArray>::New();
	nonInterestColorAttribute->SetName(colorPointAttributeName);
	//holds scale of each voxel
	vtkSmartPointer<vtkFloatArray> interestScaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	nonInterestScaleAttribute->SetName(scalePointAttributeName);
#endif

	ITMVoxelCanonical* voxelBlocks = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;

	for (int entryId = 0; entryId < noTotalEntries; entryId++) {

		const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];

		//skip unfilled hash
		if (currentHashEntry.ptr < 0) continue;

		bool inInterestRegion = interestRegionHashes.find(entryId) != interestRegionHashes.end();

		//position of the current entry in 3D space
		Vector3i currentBlockPositionVoxels = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		//_DEBUG
		//const double halfBlock = SDF_BLOCK_SIZE * maxVoxelDrawSize / 2;
		const double halfBlock = 0.0;

		//draw hash block
		hashBlockPoints->InsertNextPoint(currentBlockPositionVoxels.x + halfBlock,
		                                 currentBlockPositionVoxels.y + halfBlock,
		                                 currentBlockPositionVoxels.z + halfBlock);

		ITMVoxelCanonical* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {

					Vector3i originalPositionVoxels = currentBlockPositionVoxels + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					ITMVoxelCanonical& voxel = localVoxelBlock[locId];
					//Vector3f projectedPositionVoxels = originalPositionVoxels.toFloat() + voxel.warp_t;
					float voxelScale = 1.0f - std::abs(voxel.sdf);
					float voxelColor = (voxel.sdf + 1.0f) * 0.5f;

					if (inInterestRegion) {
						interestVoxelPoints->InsertNextPoint(maxVoxelDrawSize * originalPositionVoxels.x,
						                                     maxVoxelDrawSize * originalPositionVoxels.y,
						                                     maxVoxelDrawSize * originalPositionVoxels.z);
						interestScaleAttribute->InsertNextValue(voxelScale);
						interestScaleAttribute->InsertNextValue(voxelColor);
					}else{
						nonInterestVoxelPoints->InsertNextPoint(maxVoxelDrawSize * originalPositionVoxels.x,
						                        maxVoxelDrawSize * originalPositionVoxels.y,
						                        maxVoxelDrawSize * originalPositionVoxels.z);
#ifdef USE_CPU_GLYPH //_DEBUG
						float nextDataValue[2] = {voxelScale, voxelColor};
						pointAttributeData->InsertNextTypedTuple(nextDataValue);
#endif
						nonInterestScaleAttribute->InsertNextValue(voxelScale);
						nonInterestColorAttribute->InsertNextValue(voxelColor);
					}
				}
			}
		}
	}

	std::cout << "Scene voxel count: " << nonInterestVoxelPoints->GetNumberOfPoints() << std::endl;
	std::cout << "Allocated hash block count: " << hashBlockPoints->GetNumberOfPoints() << std::endl;

	//Points pipeline
	voxelPolydata->SetPoints(nonInterestVoxelPoints);
	interestVoxelPolydata->SetPoints(interestVoxelPoints);
	//TODO: pointAttributeData is candidate for removal (by GitHub:Algomorph)
#ifdef USE_CPU_GLYPH //_DEBUG
	voxelPolydata->GetPointData()->AddArray(pointAttributeData);
	voxelPolydata->GetPointData()->SetActiveScalars("data");
#else
	voxelPolydata->GetPointData()->AddArray(nonInterestColorAttribute);
	voxelPolydata->GetPointData()->AddArray(nonInterestScaleAttribute);
	voxelPolydata->GetPointData()->SetActiveScalars(colorPointAttributeName);
	interestVoxelPolydata->GetPointData()->AddArray(interestColorAttribute);
	interestVoxelPolydata->GetPointData()->AddArray(interestScaleAttribute);
	interestVoxelPolydata->GetPointData()->SetActiveScalars(colorPointAttributeName);
#endif
	hashBlockGrid->SetPoints(hashBlockPoints);
	initialPoints->DeepCopy(voxelPolydata->GetPoints());
}

void CanonicalVizPipe::UpdatePointPositionsFromBuffer(void* buffer) {
	vtkPoints* voxels = voxelPolydata->GetPoints();
	auto* initialPointRawData = reinterpret_cast<float*>(initialPoints->GetVoidPointer(0));
	auto* pointRawData = reinterpret_cast<float*>(voxels->GetVoidPointer(0));
	auto* warpRawData = reinterpret_cast<float*>(buffer);

	const auto pointCount = static_cast<const int>(voxels->GetNumberOfPoints());
	for (int iVoxel = 0; iVoxel < pointCount; iVoxel++) {
		//use 1st 3-float field out of 2 for the warp buffer entry
		pointRawData[iVoxel * 3 + 0] = initialPointRawData[iVoxel * 3 + 0] + warpRawData[iVoxel * 6 + 0];
		pointRawData[iVoxel * 3 + 1] = initialPointRawData[iVoxel * 3 + 1] + warpRawData[iVoxel * 6 + 1];
		pointRawData[iVoxel * 3 + 2] = initialPointRawData[iVoxel * 3 + 2] + warpRawData[iVoxel * 6 + 2];
	}
	voxelPolydata->Modified();
}

void CanonicalVizPipe::SetInterestRegionHashes(std::set<int> interestRegions) {
	this->interestRegionHashes = interestRegions;
	interestRegionHashesAreSet = true;
}
