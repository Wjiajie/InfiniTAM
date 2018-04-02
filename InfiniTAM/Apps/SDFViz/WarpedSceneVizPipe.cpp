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

//VTK
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkFloatArray.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkActor.h>
#include <vtkGlyph3DMapper.h>
#include <vtkLookupTable.h>
#include <vtkSphereSource.h>
#include <vtk-8.1/vtkCubeSource.h>

//ITMLib
#include "../../../InfiniTAM/ITMLib/Objects/Scene/ITMRepresentationAccess.h"
#include "../../../InfiniTAM/ITMLib/Utils/ITMPrintHelpers.h"

//local
#include "WarpedSceneVizPipe.h"
#include "SDFVizGlobalDefines.h"
#include "VizPipeShared.h"

//region ============================= CONSTANTS =======================================================================

const std::array<double, 3> WarpedSceneVizPipe::sliceExtremaMarkerColor = {0.882, 0.239, 0.341};

//endregion
//region ======================== CONSTRUCTORS & DESTRUCTORS ===========================================================

WarpedSceneVizPipe::WarpedSceneVizPipe(const std::array<double, 4>& positiveTruncatedNonInterestVoxelColor,
                                   const std::array<double, 4>& positiveNonTruncatedNonInterestVoxelColor,
                                   const std::array<double, 4>& negativeNonTruncatedNonInterestVoxelColor,
                                   const std::array<double, 4>& negativeTruncatedNonInterestVoxelColor,
                                   const std::array<double, 4>& unknownNonInterestVoxelColor,
                                   const std::array<double, 4>& positiveInterestVoxelColor,
                                   const std::array<double, 4>& negativeInterestVoxelColor,
                                   const std::array<double, 4>& highlightVoxelColor,
                                   const std::array<double, 3>& hashBlockEdgeColor, int frameIx) :
		SDFSceneVizPipe<ITMVoxelCanonical, ITMVoxelIndex>(positiveTruncatedNonInterestVoxelColor,
		                                                  positiveNonTruncatedNonInterestVoxelColor,
		                                                  negativeNonTruncatedNonInterestVoxelColor,
		                                                  negativeTruncatedNonInterestVoxelColor,
		                                                  unknownNonInterestVoxelColor,
		                                                  highlightVoxelColor,
		                                                  hashBlockEdgeColor),
		frameIx(frameIx),
		initialNonInterestPoints(vtkSmartPointer<vtkPoints>::New()),
		initialInterestPoints(vtkSmartPointer<vtkPoints>::New()),
		negativeInterestVoxelColor(negativeInterestVoxelColor),
		positiveInterestVoxelColor(positiveInterestVoxelColor),
		highlightVoxelColor(highlightVoxelColor),
		interestVoxelPolydata(vtkSmartPointer<vtkPolyData>::New()),
		interestVoxelColorLookupTable(vtkSmartPointer<vtkLookupTable>::New()),
		interestVoxelMapper(vtkSmartPointer<vtkGlyph3DMapper>::New()),
		interestVoxelActor(vtkSmartPointer<vtkActor>::New()),
		warplessVoxelPolydata(vtkSmartPointer<vtkPolyData>::New()),
		warplessVoxelMapper(vtkSmartPointer<vtkGlyph3DMapper>::New()),
		warplessVoxelActor(vtkSmartPointer<vtkActor>::New()),
		selectedVoxelActor(vtkSmartPointer<vtkActor>::New()),
		selectedSliceExtrema({vtkSmartPointer<vtkActor>::New(), vtkSmartPointer<vtkActor>::New()}),
		selectedSlicePreview(vtkSmartPointer<vtkActor>::New()) {
	//TODO: set up separate colors for turncated/nontruncated & interest unknown -Greg (GitHub: Algomorph)
	SetUpSDFColorLookupTable(interestVoxelColorLookupTable, highlightVoxelColor.data(),
	                         positiveInterestVoxelColor.data(),
	                         positiveInterestVoxelColor.data(), negativeInterestVoxelColor.data(),
	                         negativeInterestVoxelColor.data(), unknownNonInterestVoxelColor.data());
}
//endregion
//region ========================================== ACTOR GETTERS ======================================================

vtkSmartPointer<vtkActor>& WarpedSceneVizPipe::GetVoxelActor() {
	if (this->warpEnabled) {
		return SDFSceneVizPipe::GetVoxelActor();
	} else {
		return this->warplessVoxelActor;
	}
}

vtkSmartPointer<vtkActor>& WarpedSceneVizPipe::GetInterestVoxelActor() {
	return interestVoxelActor;
}

vtkSmartPointer<vtkActor>& WarpedSceneVizPipe::GetWarplessVoxelActor() {
	return warplessVoxelActor;
}

vtkSmartPointer<vtkActor>& WarpedSceneVizPipe::GetSelectionVoxelActor() {
	return this->selectedVoxelActor;
}

vtkSmartPointer<vtkActor>& WarpedSceneVizPipe::GetSliceSelectionActor(int index) {
	if (index < 0 && index > 1)
		DIEWITHEXCEPTION("Index needs to be 0 or 1."
				                 __FILE__
				                 ":" + std::to_string(__LINE__));
	return this->selectedSliceExtrema[index];
}

vtkSmartPointer<vtkActor>& WarpedSceneVizPipe::GetSlicePreviewActor() {
	return this->selectedSlicePreview;
}
//endregion
//region =========================================== OTHER GETTERS =====================================================

Vector3d WarpedSceneVizPipe::GetHighlightPosition(int hash, int locId) const{
	Vector3d pos;
	this->interestVoxelPolydata->GetPoints()->GetPoint(
			(*this->highlightIndexes.GetValueAt(hash, locId, frameIx)), pos.values);
	return pos;
}

std::vector<Vector3d> WarpedSceneVizPipe::GetHighlightNeighborPositions(int hash, int locId) const{
	std::vector<Vector3d> positions;
	vtkPoints* points = this->interestVoxelPolydata->GetPoints();
	const std::vector<int> indexes = *this->highlightNeighborIndexes.GetArrayAt(hash, locId, frameIx);
	for (int iNeighbor = 0; iNeighbor < indexes.size(); iNeighbor++) {
		Vector3d pos;
		points->GetPoint(indexes[iNeighbor], pos.values);
		positions.push_back(pos);
	}
	return positions;
}

bool WarpedSceneVizPipe::GetWarpEnabled() const {
	return this->warpEnabled;
}

bool WarpedSceneVizPipe::GetSliceCoordinatesAreSet() const {
	return haveSliceCoordinates;
}

void WarpedSceneVizPipe::GetSliceCoordinates(Vector3i& coord0, Vector3i coord1) const {
	coord0 = Vector3i(this->selectedSliceExtremaCoordinates[0]);
	coord1 = Vector3i(this->selectedSliceExtremaCoordinates[1]);
}

//endregion
//region =========================================== PRINTERS ==========================================================

void WarpedSceneVizPipe::PrintHighlightIndexes() {
	std::cout << this->highlightIndexes << std::endl;
}

/**
 * \brief Prints information about the voxel with given point id in voxelPolydata. Currently, doesn't work for interest voxels.
 * \param pointId id of the point in question.
 */
void
WarpedSceneVizPipe::PrintVoxelInfromation(vtkIdType pointId, const ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* scene) {

	auto scaleArray = dynamic_cast<vtkFloatArray*>(voxelPolydata->GetPointData()->GetArray(
			scaleMode == VOXEL_SCALE_HIDE_UNKNOWNS ? scaleUnknownsHiddenAttributeName
			                                       : scaleUnknownsVisibleAttributeName));
	auto colorIndexArray = dynamic_cast<vtkIntArray*>(voxelPolydata->GetPointData()->GetArray(colorAttributeName));

	//retrieve current coordiante
	auto currentPoints = voxelPolydata->GetPoints();
	double currentPoint[3];
	currentPoints->GetPoint(pointId, currentPoint);
	double current_x = currentPoint[0];
	double current_y = -currentPoint[1];
	double current_z = -currentPoint[2];

	//retrieve original coordinate before warp
	Vector3i initialCoords;
	double vizInitialCoords[3];
	RetrieveInitialCoordinates(pointId, initialCoords.values, vizInitialCoords);

	std::cout << yellow << "Selected voxel at " << current_x << ", " << current_y << ", " << current_z
	          << ". Voxel information:" << reset << std::endl;
	float selectedVoxelScale = scaleArray->GetValue(pointId);
	auto selectedVoxelColorIndex = static_cast<VoxelColorIndex>(colorIndexArray->GetValue(pointId));

	const ITMVoxelCanonical* voxelBlocks = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	bool foundPoint;
	ITMVoxelCanonical voxel = readVoxel(voxelBlocks, canonicalHashTable, initialCoords, foundPoint);
	if (!foundPoint) {
		std::cerr << "   Could not find the selected voxel in scene data! " __FILE__ ":" + std::to_string(__LINE__);
		return;
	}
	float sdfValue = ITMVoxelCanonical::valueToFloat(voxel.sdf);
	auto category = static_cast<VoxelFlags>(voxel.flags);
	std::cout << "   Initial voxel position (before warp): " << initialCoords << "." << std::endl;
	std::cout << "   Voxel display scale: " << selectedVoxelScale << ", representing the SDF value of " << sdfValue
	          << "." << std::endl;
	std::cout << "   Voxel color index: " << VoxelColorIndexAsCString(selectedVoxelColorIndex)
	          << ", representing the voxel category " << VoxelFlagsAsCString(category) << "." << std::endl;

}
//endregion
//region =========================================== POINTS, HIGHLIGHTS, & PIPELINE SETUP ==============================

void WarpedSceneVizPipe::PreparePointsForRendering(const ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* scene) {

	if (!interestRegionHashesAreSet) {
		DIEWITHEXCEPTION("Interest regions need to be set first");
	}

	voxelPolydata->GetPointData()->Reset();


	vtkSmartPointer<vtkPoints> nonInterestVoxelPoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> interestVoxelPoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> hashBlockPoints = vtkSmartPointer<vtkPoints>::New();

	//holds color for each voxel
	vtkSmartPointer<vtkIntArray> nonInterestColorAttribute = vtkSmartPointer<vtkIntArray>::New();
	nonInterestColorAttribute->SetName(colorAttributeName);
	//holds scale of each voxel
	vtkSmartPointer<vtkFloatArray> nonInterestScaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	nonInterestScaleAttribute->SetName(scaleUnknownsHiddenAttributeName);

	vtkSmartPointer<vtkFloatArray> nonInterestAlternativeScaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	nonInterestAlternativeScaleAttribute->SetName(scaleUnknownsVisibleAttributeName);


	//holds color for each voxel
	vtkSmartPointer<vtkIntArray> interestColorAttribute = vtkSmartPointer<vtkIntArray>::New();
	interestColorAttribute->SetName(colorAttributeName);
	//holds scale of each voxel
	vtkSmartPointer<vtkFloatArray> interestScaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	interestScaleAttribute->SetName(scaleUnknownsHiddenAttributeName);
	// alternative voxel scaling stragegy, where -1.0 value voxels are preserved
	vtkSmartPointer<vtkFloatArray> interestAlternativeScaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	interestAlternativeScaleAttribute->SetName(scaleUnknownsVisibleAttributeName);

	const ITMVoxelCanonical* voxelBlocks = scene->localVBA.GetVoxelBlocks();
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
		hashBlockPoints->InsertNextPoint((currentBlockPositionVoxels.x + halfBlock),
		                                 -(currentBlockPositionVoxels.y + halfBlock),
		                                 -(currentBlockPositionVoxels.z + halfBlock));
		totalVoxelCount += SDF_BLOCK_SIZE3;
		if (inInterestRegion) continue;

		const ITMVoxelCanonical* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					ComputeVoxelAttributes(currentBlockPositionVoxels, x, y, z, localVoxelBlock, nonInterestVoxelPoints,
					                       nonInterestScaleAttribute,
					                       nonInterestAlternativeScaleAttribute,
					                       nonInterestColorAttribute);
				}
			}
		}
	}
	int currentInterestPointIndex = 0;
	for (int hash : interestRegionHashes) {
		const ITMHashEntry& currentHashEntry = canonicalHashTable[hash];
		//skip unfilled hash
		if (currentHashEntry.ptr < 0) continue;
		const ITMVoxelCanonical* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		Vector3i currentBlockPositionVoxels = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPositionVoxels = currentBlockPositionVoxels + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

					const ITMVoxelCanonical& voxel = localVoxelBlock[locId];
					float sdf = ITMVoxelCanonical::valueToFloat(voxel.sdf);
					float voxelScale = COMPUTE_VOXEL_SCALE_HIDE_UNKNOWNS(sdf, voxel.flags);
					float alternativeVoxelScale = COMPUTE_VOXEL_SCALE(sdf);
					//[0.0,1.0) == negative
					//[1.0-2.0) == positive
					//3.0 == highlight
					int voxelColor;
					if (highlights.Contains(hash, locId, frameIx)) {
						voxelColor = 2;
						const std::vector<ITMHighlightIterationInfo> info = *(highlights.GetArrayAt(hash, locId,
						                                                                            frameIx));
						highlightIndexes.InsertOrdered(hash, locId, frameIx, currentInterestPointIndex);
					} else if (highlightByNeighbor.Contains(hash, locId, frameIx)) {
						const std::tuple<int, int> highlightCoords = *highlightByNeighbor.GetValueAt(hash, locId,
						                                                                             frameIx);
						highlightNeighborIndexes.InsertOrdered(std::get<0>(highlightCoords),
						                                       std::get<1>(highlightCoords), frameIx,
						                                       currentInterestPointIndex);
					} else {
						voxelColor = voxel.sdf < 0.0f ? 0 : 1;
					}

					interestVoxelPoints->InsertNextPoint(originalPositionVoxels.x,
					                                     -originalPositionVoxels.y,
					                                     -originalPositionVoxels.z);

					interestScaleAttribute->InsertNextValue(voxelScale);
					interestAlternativeScaleAttribute->InsertNextValue(alternativeVoxelScale);
					interestColorAttribute->InsertNextValue(voxelColor);
					currentInterestPointIndex++;
				}
			}
		}
	}

	//Points pipeline X2
	voxelPolydata->SetPoints(nonInterestVoxelPoints);
	voxelPolydata->GetPointData()->AddArray(nonInterestScaleAttribute);
	voxelPolydata->GetPointData()->AddArray(nonInterestAlternativeScaleAttribute);
	voxelPolydata->GetPointData()->AddArray(nonInterestColorAttribute);
	voxelPolydata->GetPointData()->SetActiveScalars(colorAttributeName);

	interestVoxelPolydata->SetPoints(interestVoxelPoints);
	interestVoxelPolydata->GetPointData()->AddArray(interestScaleAttribute);
	interestVoxelPolydata->GetPointData()->AddArray(interestAlternativeScaleAttribute);
	interestVoxelPolydata->GetPointData()->AddArray(interestColorAttribute);
	interestVoxelPolydata->GetPointData()->SetActiveScalars(colorAttributeName);

	// pash block setup
	hashBlockGrid->SetPoints(hashBlockPoints);

	// warp prep
	initialNonInterestPoints->DeepCopy(voxelPolydata->GetPoints());
	initialInterestPoints->DeepCopy(interestVoxelPolydata->GetPoints());

	warplessVoxelPolydata->SetPoints(initialNonInterestPoints);
	warplessVoxelPolydata->GetPointData()->AddArray(nonInterestScaleAttribute);
	warplessVoxelPolydata->GetPointData()->AddArray(nonInterestAlternativeScaleAttribute);
	warplessVoxelPolydata->GetPointData()->AddArray(nonInterestColorAttribute);
	warplessVoxelPolydata->GetPointData()->SetActiveScalars(colorAttributeName);

	preparePipelineWasCalled = true;
}

void WarpedSceneVizPipe::PreparePipeline(vtkAlgorithmOutput* voxelSourceGeometry,
                                         vtkAlgorithmOutput* hashBlockSourceGeometry,
                                         const ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* scene) {
	SDFSceneVizPipe::PreparePipeline(voxelSourceGeometry, hashBlockSourceGeometry, scene);

	//*** set up different selection markers ***

	auto selectionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	selectionMapper->SetInputConnection(voxelSourceGeometry);
	// selection voxel and slice extrema markers
	selectedVoxelActor->SetMapper(selectionMapper);
	selectedVoxelActor->VisibilityOff();
	selectedVoxelActor->SetScale(1.0);
	selectedVoxelActor->GetProperty()->SetColor(highlightVoxelColor[0], highlightVoxelColor[1], highlightVoxelColor[2]);
	selectedVoxelActor->GetProperty()->SetOpacity(0.84);


	for (int iExteremum = 0; iExteremum < 2; iExteremum++) {
		this->selectedSliceExtrema[iExteremum]->SetMapper(selectionMapper);
		this->selectedSliceExtrema[iExteremum]->SetScale(.23);
		this->selectedSliceExtrema[iExteremum]->GetProperty()->SetColor(sliceExtremaMarkerColor[0],
		                                                                sliceExtremaMarkerColor[1],
		                                                                sliceExtremaMarkerColor[2]);
		this->selectedSliceExtrema[iExteremum]->GetProperty()->SetOpacity(0.84);
		this->selectedSliceExtrema[iExteremum]->VisibilityOff();
	}

	vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
	auto slicePreviewMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	slicePreviewMapper->SetInputConnection(cubeSource->GetOutputPort());
	selectedSlicePreview->SetMapper(slicePreviewMapper);
	selectedSlicePreview->VisibilityOff();
	selectedSlicePreview->SetScale(1.0, 1.0, 1.0);
	selectedSlicePreview->GetProperty()->SetColor(sliceExtremaMarkerColor[0],
	                                              sliceExtremaMarkerColor[1],
	                                              sliceExtremaMarkerColor[2]);
	selectedSlicePreview->GetProperty()->SetOpacity(0.35);
	//***
}

void WarpedSceneVizPipe::SetInterestRegionInfo(std::vector<int> interestRegionHashes,
                                             ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> highlights) {
	this->highlights = highlights;
	this->interestRegionHashes = std::move(interestRegionHashes);
	interestRegionHashSet.clear();
	for (int hash : this->interestRegionHashes) {
		interestRegionHashSet.insert(hash);
	}
	auto highlightArrays = highlights.GetArrays();
	for (auto highlightArray : highlightArrays) {
		ITMHighlightIterationInfo& info = highlightArray[0];
		for (auto neighbor : info.neighbors) {
			this->highlightByNeighbor.InsertOrdered(neighbor.hash, neighbor.localId, frameIx,
			                                        std::make_tuple(info.hash, info.localId));
		}
	}
	interestRegionHashesAreSet = true;
}

void WarpedSceneVizPipe::PrepareInterestRegions(vtkAlgorithmOutput* voxelSourceGeometry) {
	if (!preparePipelineWasCalled) {
		DIEWITHEXCEPTION("PreparePipeline needs to be called first. ["
				                 __FILE__
				                 ":" + std::to_string(__LINE__) + "]");
	}
	SetUpSceneVoxelMapper(voxelSourceGeometry, interestVoxelMapper, interestVoxelColorLookupTable,
	                      interestVoxelPolydata);
	interestVoxelActor->SetMapper(interestVoxelMapper);
}

void WarpedSceneVizPipe::PrepareWarplessVoxels(vtkAlgorithmOutput* voxelSourceGeometry) {
	if (!preparePipelineWasCalled) {
		DIEWITHEXCEPTION("PreparePipeline needs to be called first. ["
				                 __FILE__
				                 ":" + std::to_string(__LINE__) + "]");
	}
	SetUpSceneVoxelMapper(voxelSourceGeometry, warplessVoxelMapper, voxelColorLookupTable, warplessVoxelPolydata);
	warplessVoxelActor->SetMapper(warplessVoxelMapper);
}

void WarpedSceneVizPipe::SetFrameIndex(int frameIx) {
	this->frameIx = frameIx;
}
//endregion
//region ========================================== WARP UPDATES =======================================================

void WarpedSceneVizPipe::UpdatePointPositionsFromBuffer(void* buffer) {
	vtkSmartPointer<vtkPoints> voxels = this->voxelPolydata->GetPoints();
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
		pointRawData[iVoxel * 3 + 1] = initialPointRawData[iVoxel * 3 + 1] - warpRawData[iVoxel * 6 + 1];
		pointRawData[iVoxel * 3 + 2] = initialPointRawData[iVoxel * 3 + 2] - warpRawData[iVoxel * 6 + 2];
	}
	voxelPolydata->Modified();
}
// assumes (1) buffers are ordered by interest region central hash (2) there is no overlap between interest regions
void WarpedSceneVizPipe::UpdateInterestRegionsFromBuffers(void* buffer) {
	vtkSmartPointer<vtkPoints> voxels = interestVoxelPolydata->GetPoints();
	auto* initialPointRawData = reinterpret_cast<float*>(initialInterestPoints->GetVoidPointer(0));
	auto* pointRawData = reinterpret_cast<float*>(voxels->GetVoidPointer(0));
	auto* warpRawData = reinterpret_cast<float*>(buffer);

	//TODO: parallelize with OpenMP  -Greg (GitHub: Algomorph)
	for (int iVoxel = 0; iVoxel < voxels->GetNumberOfPoints(); iVoxel++) {
		//use 1st 3-float field out of 2 for the warp buffer entry
		pointRawData[iVoxel * 3 + 0] = initialPointRawData[iVoxel * 3 + 0] + warpRawData[iVoxel * 6 + 0];
		pointRawData[iVoxel * 3 + 1] = initialPointRawData[iVoxel * 3 + 1] - warpRawData[iVoxel * 6 + 1];
		pointRawData[iVoxel * 3 + 2] = initialPointRawData[iVoxel * 3 + 2] - warpRawData[iVoxel * 6 + 2];
	}
	interestVoxelPolydata->Modified();
}
//endregion
//region =========================================== INTERACTIONS / MODES ===============================================

void WarpedSceneVizPipe::ToggleScaleMode() {
	SDFSceneVizPipe::ToggleScaleMode();
	if (scaleMode == VoxelScaleMode::VOXEL_SCALE_HIDE_UNKNOWNS) {
		interestVoxelMapper->SetScaleArray(scaleUnknownsHiddenAttributeName);
		warplessVoxelMapper->SetScaleArray(scaleUnknownsHiddenAttributeName);
	} else {
		interestVoxelMapper->SetScaleArray(scaleUnknownsVisibleAttributeName);
		warplessVoxelMapper->SetScaleArray(scaleUnknownsVisibleAttributeName);
	}
}

/**
 * \brief Moves the highlight marker to the specified point if highlightOn argument is set to true, hides it otherwise
 * \param pointId point where to move the highlight marker
 * \param highlightOn whether to move & display the highlight marker or to hide it
 */
void WarpedSceneVizPipe::SelectOrDeselectVoxel(vtkIdType pointId, bool highlightOn,
                                               const ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* scene) {
	auto scaleArray = dynamic_cast<vtkFloatArray*>(voxelPolydata->GetPointData()->GetArray(
			scaleMode == VOXEL_SCALE_HIDE_UNKNOWNS ? scaleUnknownsHiddenAttributeName
			                                       : scaleUnknownsVisibleAttributeName));

	auto points = voxelPolydata->GetPoints();
	if (highlightOn) {
		selectedVoxelActor->VisibilityOn();
		float selectedVoxelScale = scaleArray->GetValue(pointId);
		double point[3];
		points->GetPoint(pointId, point);
		selectedVoxelActor->SetScale(selectedVoxelScale + 0.01);
		selectedVoxelActor->SetPosition(point);
		PrintVoxelInfromation(pointId, scene);
	} else {
		selectedVoxelActor->VisibilityOff();
	}
}

/**
 * \brief Retrieve initial (pre-warp) coordinates of the voxel corresponding to the specified point index.
 * \param pointId [in] point id (a.k.a. index) of the point corresponding to the voxel.
 * \param initialCoordinates [out] the pre-warp coordinates of the corresponding voxel.
 * \param vizCoordinates [out] the pre-warp coordinates in visualization space (different axis directions)
 */
inline void
WarpedSceneVizPipe::RetrieveInitialCoordinates(vtkIdType pointId, int initialCoordinates[3],
                                             double vizCoordinates[3]) const {
	//retrieve original coordinate before warp
	auto& initialPoints = this->initialNonInterestPoints;
	initialPoints->GetPoint(pointId, vizCoordinates);
	initialCoordinates[0] = static_cast<int>(vizCoordinates[0]);
	initialCoordinates[1] = static_cast<int>(-vizCoordinates[1]);
	initialCoordinates[2] = static_cast<int>(-vizCoordinates[2]);
}

/**
 * \brief Sets the first (or second) extremum for a box-like slice in the canonical sdf.
 * (depending on whether the first one has already been selected)
 * \details Note: the coordinate used for the slice will be the initial coordinate of the voxel in global/canonical space,
 * before application of the warp, so it might not exactly correspond visually to the clicked voxel. Both will be shown
 * in the viewport.
 * \param pointId[in] current (after-warp) coordinate of the canonical voxel whose initial coordinate (before-warp) should serve as the slice extremum.
 * \param continueSliceSelection[out] if this call sets the second extremum of the slice, sets this to false, sets to true otherwise.
 */
void WarpedSceneVizPipe::SetSliceSelection(vtkIdType pointId, bool& continueSliceSelection,
                                           const ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* scene) {
	vtkSmartPointer<vtkActor> extremum;
	bool updateSlicePreview = false;
	int selectedSliceExtremaIndex = 0;
	if (this->firstSliceBoundSelected) {
		extremum = selectedSliceExtrema[1];
		continueSliceSelection = false;
		this->firstSliceBoundSelected = false;
		std::cout << bright_cyan << "Selecting second slice extremum..." << reset << std::endl;
		this->selectedSlicePreview->VisibilityOn();
		updateSlicePreview = true;
		selectedSliceExtremaIndex = 1;
	} else {
		extremum = selectedSliceExtrema[0];
		this->firstSliceBoundSelected = true;
		continueSliceSelection = true;
		std::cout << bright_cyan << "Selecting first slice extremum..." << reset << std::endl;
		selectedSliceExtremaIndex = 0;
	}
	SelectOrDeselectVoxel(pointId, true, scene);
	Vector3i initialCoordinates;
	Vector3d initialCoordinatesViz;
	extremum->VisibilityOn();
	RetrieveInitialCoordinates(pointId, initialCoordinates.values, initialCoordinatesViz.values);
	extremum->SetPosition(initialCoordinatesViz.values);
	selectedSliceExtremaCoordinates[selectedSliceExtremaIndex] = initialCoordinates;
	if (updateSlicePreview) {
		//@formatter:off
		Vector3d center(
				static_cast<double>(selectedSliceExtremaCoordinates[1].x + selectedSliceExtremaCoordinates[0].x) / 2.0,
				static_cast<double>(selectedSliceExtremaCoordinates[1].y + selectedSliceExtremaCoordinates[0].y) / 2.0,
				static_cast<double>(selectedSliceExtremaCoordinates[1].z + selectedSliceExtremaCoordinates[0].z) / 2.0);
		Vector3d centerViz(center.x, -center.y, -center.z);
		Vector3d dimensions(
				std::abs(selectedSliceExtremaCoordinates[1].x - selectedSliceExtremaCoordinates[0].x) + 1.0,
				std::abs(selectedSliceExtremaCoordinates[1].y - selectedSliceExtremaCoordinates[0].y) + 1.0,
				std::abs(selectedSliceExtremaCoordinates[1].z - selectedSliceExtremaCoordinates[0].z) + 1.0);
		//@formatter:on
		selectedSlicePreview->SetPosition(centerViz.values);
		selectedSlicePreview->SetScale(dimensions.values);
		selectedSlicePreview->VisibilityOn();
		haveSliceCoordinates = true;
	}
}

void WarpedSceneVizPipe::ToggleWarpEnabled() {
	if (warpEnabled) {
		GetVoxelActor()->VisibilityOff();
		warpEnabled = false;
		GetVoxelActor()->VisibilityOn();
	} else {
		GetVoxelActor()->VisibilityOff();
		warpEnabled = true;
		GetVoxelActor()->VisibilityOn();
	}
}

/**
 * \brief Clear current slice selection
 */
void WarpedSceneVizPipe::ClearSliceSelection() {
	selectedSlicePreview->VisibilityOff();
	selectedSliceExtrema[0]->VisibilityOff();
	selectedSliceExtrema[1]->VisibilityOff();
	haveSliceCoordinates = false;
}
// endregion
// region ============================================ SCENE MANIPULATION ==============================================

void WarpedSceneVizPipe::MakeSliceFromSelection() {
	if(haveSliceCoordinates){

	}
}

void WarpedSceneVizPipe::TestFunction() {

}


// endregion




