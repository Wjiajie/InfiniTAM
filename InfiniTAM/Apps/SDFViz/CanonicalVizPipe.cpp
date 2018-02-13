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


//stdlib
#include <utility>

#include <vtkFloatArray.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkActor.h>
#include <vtkGlyph3DMapper.h>
#include <vtkLookupTable.h>

#include "CanonicalVizPipe.h"

//DEBUG
template<typename T>
std::ostream& operator<<(std::ostream& os, std::vector<T> vec) {
	os << "{ ";
	std::copy(vec.begin(), vec.end(), std::ostream_iterator<T>(os, " "));
	os << "}";
	return os;
}

CanonicalVizPipe::CanonicalVizPipe(const std::array<double, 4>& negativeNonInterestVoxelColor,
                                   const std::array<double, 4>& positiveNonInterestVoxelColor,
                                   const std::array<double, 4>& negativeInterestVoxelColor,
                                   const std::array<double, 4>& positiveInterestVoxelColor,
                                   const std::array<double, 4>& highlightVoxelColor,
                                   const std::array<double, 3>& hashBlockEdgeColor) :
		SDFSceneVizPipe<ITMVoxelCanonical, ITMVoxelIndex>(
				negativeNonInterestVoxelColor, positiveNonInterestVoxelColor, hashBlockEdgeColor),
		initialNonInterestPoints(vtkSmartPointer<vtkPoints>::New()),
		initialInterestPoints(vtkSmartPointer<vtkPoints>::New()),
		negativeInterestVoxelColor(negativeInterestVoxelColor),
		positiveInterestVoxelColor(positiveInterestVoxelColor),
		highlightVoxelColor(highlightVoxelColor),
		interestVoxelPolydata(vtkSmartPointer<vtkPolyData>::New()),
		interestVoxelColorLookupTable(vtkSmartPointer<vtkLookupTable>::New()),
		interestVoxelMapper(vtkSmartPointer<vtkGlyph3DMapper>::New()),
		interestVoxelActor(vtkSmartPointer<vtkActor>::New()) {
	SetUpHighlightSDFColorLookupTable(interestVoxelColorLookupTable, negativeInterestVoxelColor.data(),
	                                  positiveInterestVoxelColor.data(), highlightVoxelColor.data());
}

void CanonicalVizPipe::PreparePointsForRendering() {

	if (!interestRegionHashesAreSet) {
		DIEWITHEXCEPTION("Interest regions need to be set first");
	}

	vtkSmartPointer<vtkPoints> nonInterestVoxelPoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> interestVoxelPoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> hashBlockPoints = vtkSmartPointer<vtkPoints>::New();

	//holds color for each voxel
	vtkSmartPointer<vtkFloatArray> nonInterestColorAttribute = vtkSmartPointer<vtkFloatArray>::New();
	nonInterestColorAttribute->SetName(colorPointAttributeName);
	//holds scale of each voxel
	vtkSmartPointer<vtkFloatArray> nonInterestScaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	nonInterestScaleAttribute->SetName(scalePointAttributeName);
	//holds color for each voxel
	vtkSmartPointer<vtkIntArray> interestColorAttribute = vtkSmartPointer<vtkIntArray>::New();
	interestColorAttribute->SetName(colorPointAttributeName);
	//holds scale of each voxel
	vtkSmartPointer<vtkFloatArray> interestScaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	interestScaleAttribute->SetName(scalePointAttributeName);

	ITMVoxelCanonical* voxelBlocks = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	int hashBlockCount = scene->index.noTotalEntries;

	totalVoxelCount = 0;
	interestRegionRanges.clear();
	bool inInterestRegion = false;
	std::tuple<int, int> currentInterestRegionRange;
	for (int hash = 0; hash < hashBlockCount; hash++) {
		const ITMHashEntry& currentHashEntry = canonicalHashTable[hash];
		//skip unfilled hash
		if (currentHashEntry.ptr < 0) continue;
		bool inInterestRegionNewVal = interestRegionHashSet.find(hash) != interestRegionHashSet.end();
		if (!inInterestRegion && inInterestRegionNewVal) {
			//start range
			std::get<0>(currentInterestRegionRange) = totalVoxelCount;
		}
		if (inInterestRegion && !inInterestRegionNewVal) {
			//end range
			std::get<1>(currentInterestRegionRange) = totalVoxelCount;
			interestRegionRanges.push_back(currentInterestRegionRange);
		}
		inInterestRegion = inInterestRegionNewVal;
		//position of the current entry in 3D space
		Vector3i currentBlockPositionVoxels = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		//_DEBUG
		//const double halfBlock = SDF_BLOCK_SIZE * maxVoxelDrawSize / 2;
		const double halfBlock = 0.0;

		//draw hash block
		hashBlockPoints->InsertNextPoint(currentBlockPositionVoxels.x + halfBlock,
		                                 currentBlockPositionVoxels.y + halfBlock,
		                                 currentBlockPositionVoxels.z + halfBlock);
		totalVoxelCount += SDF_BLOCK_SIZE3;
		if (inInterestRegion) continue;

		ITMVoxelCanonical* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPositionVoxels = currentBlockPositionVoxels + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					ITMVoxelCanonical& voxel = localVoxelBlock[locId];
					float voxelScale = COMPUTE_VOXEL_SCALE(voxel);
					float voxelColor = (voxel.sdf + 1.0f) * 0.5f;
					nonInterestVoxelPoints->InsertNextPoint(maxVoxelDrawSize * originalPositionVoxels.x,
					                                        maxVoxelDrawSize * originalPositionVoxels.y,
					                                        maxVoxelDrawSize * originalPositionVoxels.z);
					nonInterestScaleAttribute->InsertNextValue(voxelScale);
					nonInterestColorAttribute->InsertNextValue(voxelColor);

				}
			}
		}
	}
	for (int hash : interestRegionHashes) {
		const ITMHashEntry& currentHashEntry = canonicalHashTable[hash];
		//skip unfilled hash
		if (currentHashEntry.ptr < 0) continue;
		ITMVoxelCanonical* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		Vector3i currentBlockPositionVoxels = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPositionVoxels = currentBlockPositionVoxels + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

					ITMVoxelCanonical& voxel = localVoxelBlock[locId];
					float voxelScale = COMPUTE_VOXEL_SCALE(voxel);
					//[0.0,1.0) == negative
					//[1.0-2.0) == positive
					//3.0 == highlight
					int voxelColor = highlights.Contains(hash, locId) ? 2 : voxel.sdf < 0.0f ? 0 : 1;
					interestVoxelPoints->InsertNextPoint(maxVoxelDrawSize * originalPositionVoxels.x,
					                                     maxVoxelDrawSize * originalPositionVoxels.y,
					                                     maxVoxelDrawSize * originalPositionVoxels.z);
					interestScaleAttribute->InsertNextValue(voxelScale);
					interestColorAttribute->InsertNextValue(voxelColor);
				}
			}
		}
	}

	//Points pipeline
	voxelPolydata->SetPoints(nonInterestVoxelPoints);
	interestVoxelPolydata->SetPoints(interestVoxelPoints);

	voxelPolydata->GetPointData()->AddArray(nonInterestScaleAttribute);
	voxelPolydata->GetPointData()->AddArray(nonInterestColorAttribute);
	voxelPolydata->GetPointData()->SetActiveScalars(colorPointAttributeName);

	interestVoxelPolydata->GetPointData()->AddArray(interestScaleAttribute);
	interestVoxelPolydata->GetPointData()->AddArray(interestColorAttribute);
	interestVoxelPolydata->GetPointData()->SetActiveScalars(colorPointAttributeName);

	hashBlockGrid->SetPoints(hashBlockPoints);
	initialNonInterestPoints->DeepCopy(voxelPolydata->GetPoints());
	initialInterestPoints->DeepCopy(interestVoxelPolydata->GetPoints());
	preparePipelineWasCalled = true;
}

void CanonicalVizPipe::UpdatePointPositionsFromBuffer(void* buffer) {
	vtkPoints* voxels = voxelPolydata->GetPoints();
	auto* initialPointRawData = reinterpret_cast<float*>(initialNonInterestPoints->GetVoidPointer(0));
	auto* pointRawData = reinterpret_cast<float*>(voxels->GetVoidPointer(0));
	auto* warpRawData = reinterpret_cast<float*>(buffer);

	std::tuple<int, int> nextInterestRegionRange;
	bool hasMoreInterestRegions = false;
	int interestRegionIx = 0;
	if (!interestRegionRanges.empty()) {
		nextInterestRegionRange = interestRegionRanges[interestRegionIx];
		hasMoreInterestRegions = true;
	}
	//TODO: parallelize (the way interest regions are handled right now is a problem) -Greg (GitHub: Algomorph)
	for (int iVoxel = 0; iVoxel < totalVoxelCount; iVoxel++) {
		//skip over the intererest region voxels
		if (hasMoreInterestRegions && iVoxel > std::get<0>(nextInterestRegionRange)) {
			if (iVoxel < std::get<1>(nextInterestRegionRange)) {
				continue;
			} else {
				interestRegionIx++;
				if (interestRegionIx == interestRegionRanges.size()) {
					hasMoreInterestRegions = false;
				} else {
					nextInterestRegionRange = interestRegionRanges[interestRegionIx];
				}
			}
		}
		//use 1st 3-float field out of 2 for the warp buffer entry
		pointRawData[iVoxel * 3 + 0] = initialPointRawData[iVoxel * 3 + 0] + warpRawData[iVoxel * 6 + 0];
		pointRawData[iVoxel * 3 + 1] = initialPointRawData[iVoxel * 3 + 1] + warpRawData[iVoxel * 6 + 1];
		pointRawData[iVoxel * 3 + 2] = initialPointRawData[iVoxel * 3 + 2] + warpRawData[iVoxel * 6 + 2];
	}
	voxelPolydata->Modified();
}

void CanonicalVizPipe::SetInterestRegionInfo(std::vector<int> interestRegionHashes, ITM3DNestedMap<ITMHighlightIterationInfo> highlights) {
	this->highlights = highlights;
	this->interestRegionHashes = std::move(interestRegionHashes);
	for (int hash : this->interestRegionHashes) {
		interestRegionHashSet.insert(hash);
	}
	interestRegionHashesAreSet = true;
}

void CanonicalVizPipe::PrepareInterestRegions(vtkAlgorithmOutput* voxelSourceGeometry) {
	if (!preparePipelineWasCalled) {
		DIEWITHEXCEPTION("PreparePipeline needs to be called first.");
	}

	// set up voxel mapper
	SetUpSceneVoxelMapper(voxelSourceGeometry, interestVoxelMapper, interestVoxelColorLookupTable,
	                      interestVoxelPolydata);
	interestVoxelMapper->SetScalarRange(0.0, 3.0);
	interestVoxelMapper->InterpolateScalarsBeforeMappingOff();

	// set up voxel actor
	interestVoxelActor->SetMapper(interestVoxelMapper);
}

vtkSmartPointer<vtkActor>& CanonicalVizPipe::GetInterestVoxelActor() {
	return interestVoxelActor;
}

// assumes (1) buffers are ordered by interest region central hash (2) there is no overlap between interest regions
void CanonicalVizPipe::UpdateInterestRegionsFromBuffers(void* buffer) {
	vtkPoints* voxels = interestVoxelPolydata->GetPoints();

	auto* initialPointRawData = reinterpret_cast<float*>(initialInterestPoints->GetVoidPointer(0));
	auto* pointRawData = reinterpret_cast<float*>(voxels->GetVoidPointer(0));
	auto* warpRawData = reinterpret_cast<float*>(buffer);

	//TODO: parallelize with OpenMP  -Greg (GitHub: Algomorph)
	for (int iVoxel = 0; iVoxel < voxels->GetNumberOfPoints(); iVoxel++) {
		//use 1st 3-float field out of 2 for the warp buffer entry
		pointRawData[iVoxel * 3 + 0] = initialPointRawData[iVoxel * 3 + 0] + warpRawData[iVoxel * 6 + 0];
		pointRawData[iVoxel * 3 + 1] = initialPointRawData[iVoxel * 3 + 1] + warpRawData[iVoxel * 6 + 1];
		pointRawData[iVoxel * 3 + 2] = initialPointRawData[iVoxel * 3 + 2] + warpRawData[iVoxel * 6 + 2];
	}
	interestVoxelPolydata->Modified();
}

void CanonicalVizPipe::SetUpHighlightSDFColorLookupTable(vtkSmartPointer<vtkLookupTable>& table,
                                                         const double* rgbaFirstColor,
                                                         const double* rgbaSecondColor,
                                                         const double* rgbaHighlightColor) {
	table->SetTableRange(0.0, 3.0);
	table->SetNumberOfTableValues(3);
	table->SetNumberOfColors(3);
	table->SetTableValue(0, rgbaFirstColor);
	table->SetTableValue(1, rgbaSecondColor);
	table->SetTableValue(2, rgbaHighlightColor);
	table->SetNanColor(0.4,0.7,0.1,1.0);
}