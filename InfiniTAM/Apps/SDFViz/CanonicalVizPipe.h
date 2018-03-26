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
#pragma once

#include "SDFSceneVizPipe.h"
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Utils/ITM3DNestedMap.h"

class CanonicalVizPipe : public SDFSceneVizPipe<ITMVoxelCanonical, ITMVoxelIndex> {
public:
	// ==================== CONSTANTS ==================================================
	static const std::array<double,3> sliceExtremaMarkerColor;

	// ==================== CONSTRUCTORS / DESTRUCTORS =================================
	CanonicalVizPipe(const std::array<double, 4>& positiveTruncatedNonInterestVoxelColor,
	                 const std::array<double, 4>& positiveNonTruncatedNonInterestVoxelColor,
	                 const std::array<double, 4>& negativeNonTruncatedNonInterestVoxelColor,
	                 const std::array<double, 4>& negativeTruncatedNonInterestVoxelColor,
	                 const std::array<double, 4>& unknownNonInterestVoxelColor,
	                 const std::array<double, 4>& positiveInterestVoxelColor,
	                 const std::array<double, 4>& negativeInterestVoxelColor,
	                 const std::array<double, 4>& highlightVoxelColor,
	                 const std::array<double, 3>& hashBlockEdgeColor, int frameIx);

	// ==================== MEMBER FUNCTIONS ===========================================
	void UpdatePointPositionsFromBuffer(void* buffer);
	void UpdateInterestRegionsFromBuffers(void* buffer);

	// *** getter/setter/printer ***
	Vector3d GetHighlightPosition(int hash, int locId) const;
	std::vector<Vector3d> GetHighlightNeighborPositions(int hash, int locId) const;
	void SetInterestRegionInfo(std::vector<int> interestRegionHashes,
	                           ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> highlights);
	void SetFrameIndex(int frameIx);
	vtkSmartPointer<vtkActor>& GetVoxelActor() override;
	vtkSmartPointer<vtkActor>& GetInterestVoxelActor();
	vtkSmartPointer<vtkActor>& GetWarplessVoxelActor();
	vtkSmartPointer<vtkActor>& GetSelectionVoxelActor();
	vtkSmartPointer<vtkActor>& GetSliceSelectionActor(int index);
	vtkSmartPointer<vtkActor>& GetSlicePreviewActor();
	bool GetWarpEnabled() const;
	void PrintHighlightIndexes();

	// *** setup ***
	void PrepareInterestRegions(vtkAlgorithmOutput* voxelSourceGeometry);
	void PrepareWarplessVoxels(vtkAlgorithmOutput* voxelSourceGeometry);
	void PreparePipeline(vtkAlgorithmOutput* voxelSourceGeometry,
	                     vtkAlgorithmOutput* hashBlockSourceGeometry) override;

	// *** modify state ***
	void ToggleScaleMode() override;
	void ToggleWarpEnabled();
	void SelectOrDeselectVoxel(vtkIdType pointId, bool highlightOn);
	void SetSliceSelection(vtkIdType pointId, bool& continueSliceSelection);
	
protected:
	// *** setup ***
	void PreparePointsForRendering() override;
	vtkSmartPointer<vtkPoints> initialNonInterestPoints;
	vtkSmartPointer<vtkPoints> initialInterestPoints;
private:
	// =============== MEMBER FUNCTIONS ================================================================================
	void PrintVoxelInfromation(vtkIdType pointId);
	void RetrieveInitialCoordinates(vtkIdType pointId, int initialCoordinates[3], double vizCoordinates[3]) const;

	// =============== MEMBER VARIABLES ================================================================================
	//frame of the warp
	int frameIx;

	// ** colors **
	std::array<double, 4> negativeInterestVoxelColor;
	std::array<double, 4> positiveInterestVoxelColor;
	std::array<double, 4> highlightVoxelColor;

	// ** state **
	bool interestRegionHashesAreSet = false;
	bool preparePipelineWasCalled = false;
	bool firstSliceBoundSelected = false;
	bool warpEnabled = true;

	// ** interest region info **
	std::vector<int> interestRegionHashes;
	std::set<int> interestRegionHashSet;
	int totalVoxelCount = 0;//includes both interest regions and the rest
	std::vector<std::tuple<int, int>> interestRegionRanges;
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> highlights;
	ITM3DNestedMap<int> highlightIndexes;
	ITM3DNestedMap<std::tuple<int, int>> highlightByNeighbor;
	ITM3DNestedMapOfArrays<int> highlightNeighborIndexes;

	// ** individual voxels **
	vtkSmartPointer<vtkPolyData> interestVoxelPolydata;
	vtkSmartPointer<vtkLookupTable> interestVoxelColorLookupTable;
	vtkSmartPointer<vtkGlyph3DMapper> interestVoxelMapper;
	vtkSmartPointer<vtkActor> interestVoxelActor;
	vtkSmartPointer<vtkPolyData> warplessVoxelPolydata;
	vtkSmartPointer<vtkGlyph3DMapper> warplessVoxelMapper;
	vtkSmartPointer<vtkActor> warplessVoxelActor;


	// ** interaction **
	int selectedVertexDefaultColorIndex;
	vtkSmartPointer<vtkActor> selectedVoxelActor;
	vtkSmartPointer<vtkActor> selectedSliceExtrema[2];
	vtkSmartPointer<vtkActor> selectedSlicePreview;
	Vector3i selectedSliceExtremaCoordinates[2];
};