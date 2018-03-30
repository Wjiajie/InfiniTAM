//  ================================================================
//  7Created by Gregory Kramida on 1/3/18.
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

//#define USE_TEST_SCENE

//VTK
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkObjectFactory.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkLegendBoxActor.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkPointPicker.h>

//local
#include "SDFSceneVizPipe.tpp"

//ITMLib
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Objects/Scene/ITMScene.h"
#include "WarpedSceneVizPipe.h"
#include "HighlightVisualization.h"

using namespace ITMLib;
namespace ITMLib {
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneLogger;
}

class vtkRenderer;
class vtkRenderWindow;
class vtkPoints;
class vtkPolyData;
class vtkImageData;
class vtkStructuredGrid;
class vtkGlyph3D;
class vtkGlyph3DMapper;
class vtkFloatArray;
class vtkAlgorithmOutput;
class vtkLookupTable;
class vtkTextActor;

/**
 * \brief SDF Visualization application main class.
 */
class SDFViz {
	friend class SDFVizInteractorStyle;

public:
	//================= STATIC CONSTANTS ============================
	static const std::array<double, 4> canonicalTrunctedPositiveVoxelColor;
	static const std::array<double, 4> canonicalNonTruncatedPositiveVoxelColor;
	static const std::array<double, 4> canonicalNonTruncatedNegativeVoxelColor;
	static const std::array<double, 4> canonicalTrunctedNegativeVoxelColor;
	static const std::array<double, 4> canonicalUnknownVoxelColor;

	static const std::array<double, 4> canonicalNegativeInterestVoxelColor;
	static const std::array<double, 4> canonicalPositiveInterestVoxelColor;
	static const std::array<double, 4> highlightVoxelColor;
	static const std::array<double, 3> canonicalHashBlockEdgeColor;

	static const std::array<double, 4> liveTruncatedPositiveVoxelColor;
	static const std::array<double, 4> liveNonTruncatedPositiveVoxelColor;
	static const std::array<double, 4> liveNonTruncatedNegativeVoxelColor;
	static const std::array<double, 4> liveTruncatedNegativeVoxelColor;
	static const std::array<double, 4> liveUnknownVoxelColor;

	static const std::array<double, 3> liveHashBlockEdgeColor;

	//================= CONSTRUCTORS/DESTRUCTORS ===================
	SDFViz(std::string pathToScene, bool hideNonInterestCanonicalVoxels, bool hideLiveVoxels,
	       bool hideInterestCanonicalRegions, bool hideUnknownCanonicalVoxels, bool useInitialCoords,
	       Vector3i initialCoords, unsigned int initialFrame);
	virtual ~SDFViz();
	//================= INSTANCE MEMBER FUNCTIONS ==================
	int Run();

private:
	//================= CONSTANTS ==================================
	// Array holding various background colors to cycle through
	static const std::array<std::array<double,4>,4> backgroundColors;

	//================= STATIC FUNCTIONS ===========================
	static Vector3d ComputeCameraRightVector(vtkCamera* camera);

	//================= MEMBER VARIABLES ===========================
	// Data loader
	ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>* sceneLogger;
	// path to root directory (i.e. without the Frame_XXXX prefixes)
	std::string rootPath;

	// Visualization setup
	// for actual voxels
	vtkSmartPointer<vtkRenderer> sdfRenderer;
	// for highlights, slices, floating legends, & other markers / gizmos
	vtkSmartPointer<vtkRenderer> markerRenderer;
	// for overlaying GUI elements
	vtkSmartPointer<vtkRenderer> guiOverlayRenderer;
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;

	// GUI elements
	vtkSmartPointer<vtkTextActor> iterationIndicator;
	vtkSmartPointer<vtkTextActor> frameIndicator;
	vtkSmartPointer<vtkLegendBoxActor> legend;
	vtkSmartPointer<vtkLegendBoxActor> messageBar;
	// for switching the background color
	int currentBackgrounColorIx = 0;

	// Structures for rendering scene geometry with VTK
	WarpedSceneVizPipe canonicalScenePipe;
	SDFSceneVizPipe<ITMVoxelLive, ITMVoxelIndex> liveScenePipe;
	HighlightVisualization highlightVisualizer;
	vtkSmartPointer<vtkSphereSource> sphere;
	vtkSmartPointer<vtkCubeSource> cube;
	// interaction

	//Holds warp & warp update state for the canonical scene
	vtkSmartPointer<vtkFloatArray> allWarpBuffer;
	vtkSmartPointer<vtkFloatArray> interestWarpBuffer;
	bool hasWarpIterationInfo = true;
	bool hasHighlightInfo = false;
	//frame that we're loading scene & warps for
	unsigned int frameIndex;
	//to keep track of iteration number in the optimization
	unsigned int iterationIndex;

	//Visibility / vis. mode states
	bool canonicalVoxelsVisible = true;
	bool canonicalInterestVoxelsVisible = true;
	bool canonicalUnknownVoxelsVisible = true;
	bool canonicalHashBlocksVisible = false;
	bool liveVoxelsVisible = true;
	bool liveUnknownVoxelsVisible = false;
	bool liveHashBlocksVisible = false;

	//Holds highlights in the scene
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> highlights;
	const std::vector<ITMHighlightIterationInfo>* currentHighlight;

	//================ MEMBER FUNCTIONS =====================

	//*** initialization / drawing GUI elements ***
	void InitializeRendering();
	void InitializeWarpBuffers();
	void InitializeWarps();
	void AddActors();
	void DrawLegend();
	void DrawMessageBar();
	void DrawIterationCounter();
	void DrawFrameCounter();
	void DrawDummyMarkers();
	void SetUpGeometrySources();
	void ReinitializePipelines();
	void UpdatePipelineVisibilitiesUsingLocalState();

	//*** advance/retreat warp / optimization iteration ***
	bool AdvanceIteration();
	bool RetreatIteration();
	bool NextNonInterestWarps();
	bool NonInterestWarpsAt(unsigned int iteration);
	bool PreviousNonInterestWarps();
	void InterestWarpBufferHelper();
	bool NextInterestWarps();
	bool PreviousInterestWarps();
	bool InterestWarpsAt(unsigned int iteration);

	//*** control/advance/retreat frame ***
	std::string GenerateExpectedFramePath();
	//read scenes from disk
	void LoadFrameData();
	bool AdvanceFrame();
	bool RetreatFrame();

	//*** visibility / opacity ***
	void ToggleCanonicalHashBlockVisibility();
	void ToggleLiveHashBlockVisibility();

	void ToggleCanonicalVoxelVisibility();
	void ToggleLiveVoxelVisibility();

	void ToggleCanonicalUnknownVoxelVisibility();
	void ToggleLiveUnknownVoxelVisibility();

	void DecreaseCanonicalVoxelOpacity();
	void IncreaseCanonicalVoxelOpacity();

	void ToggleInterestVoxelVisibility();

	void DecreaseLiveVoxelOpacity();
	void IncreaseLiveVoxelOpacity();

	//*** update /change GUI ***
	void UpdateIterationDisplay();
	void PreviousBackgroundColor();
	void NextBackgroundColor();
	void UpdateFrameDisplay();
	void UpdateMessageBar(std::string text);
	void ClearMessageBar();

	//*** viewport navigation ***
	void MoveFocusToHighlightAt(int hash, int localId);
	void MoveFocusToVoxelAt(Vector3d absoluteCoordinates);
	void RefocusAtCurrentHighlight();
	void MoveFocusToNextHighlight();
	void MoveFocusToPreviousHighlight();

	//*** scene manipulation ***
	bool MakeSlice();
};