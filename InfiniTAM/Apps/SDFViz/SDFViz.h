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
#pragma once

//#define USE_TEST_SCENE

//VTK
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkObjectFactory.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkLegendBoxActor.h>
#include <vtk-8.1/vtkSphereSource.h>
#include <vtk-8.1/vtkCubeSource.h>

//local
#include "SDFSceneVizPipe.tpp"

//ITMLib
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Objects/Scene/ITMScene.h"
#include "CanonicalVizPipe.h"
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
	friend class SDFVizKeyPressInteractorStyle;

public:
	//================= STATIC CONSTANTS ============================
	static const std::array<double, 4> canonicalNegativeSDFVoxelColor;
	static const std::array<double, 4> canonicalPositiveSDFVoxelColor;
	static const std::array<double, 4> canonicalNegativeInterestSDFVoxelColor;
	static const std::array<double, 4> canonicalPositiveInterestSDFVoxelColor;
	static const std::array<double, 4> canonicalHighlightSDFVoxelColor;
	static const std::array<double, 3> canonicalHashBlockEdgeColor;
	static const std::array<double, 4> liveNegativeSDFVoxelColor;
	static const std::array<double, 4> livePositiveSDFVoxelColor;
	static const std::array<double, 3> liveHashBlockEdgeColor;

	//================= CONSTRUCTORS/DESTRUCTORS ===================
	SDFViz(std::string pathToScene, bool showNonInterestCanonicalVoxels, bool showLiveVoxels,
	       bool hideInterestCanonicalRegions, bool useInitialCoords, Vector3i initialCoords);
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
	vtkSmartPointer<vtkRenderer> sdfRenderer;
	vtkSmartPointer<vtkRenderer> topRenderer;
	// The render window is the actual GUI window
	// that appears on the computer screen
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	// The render window interactor captures mouse events
	// and will perform appropriate camera or actor manipulation
	// depending on the nature of the events.
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
	// indicator at top left corner showing for which (if any) iteration the data is being shown
	vtkSmartPointer<vtkTextActor> iterationIndicator;
	// for switching the background color
	int currentBackgrounColorIx = 0;
	// legend
	vtkSmartPointer<vtkLegendBoxActor> legend;

	// Structures for rendering scene geometry with VTK
	CanonicalVizPipe canonicalScenePipe;
	SDFSceneVizPipe<ITMVoxelLive, ITMVoxelIndex> liveScenePipe;
	HighlightVisualization highlightVisualizer;
	vtkSmartPointer<vtkSphereSource> sphere;
	vtkSmartPointer<vtkCubeSource> cube;

	//Holds warp & warp update state for the canonical scene
	vtkSmartPointer<vtkFloatArray> allWarpBuffer;
	vtkSmartPointer<vtkFloatArray> interestWarpBuffer;
	bool hasWarpIterationInfo = true;
	bool hasHighlightInfo = false;
	//frame that we're loading scene & warps for
	unsigned int frameIndex;
	//to keep track of iteration number in the optimization
	unsigned int iterationIndex;

	//Holds highlights in the scene
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> highlights;
	const std::vector<ITMHighlightIterationInfo>* currentHighlight;

	//================ MEMBER FUNCTIONS =====================

	//*** initialization / drawing GUI elements ***
	void InitializeRendering();
	void InitializeWarpBuffers();
	void DrawLegend();
	void DrawIterationCounter();
	void DrawDummyMarkers();
	void SetUpGeometrySources();
	void ReinitializePipelines();

	//*** advance/retreat warp / otimization interation ***
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
	bool NextFrame();
	bool PreviousFrame();

	//*** visibility / opacity ***
	void ToggleCanonicalHashBlockVisibility();
	void ToggleLiveHashBlockVisibility();

	void ToggleCanonicalVoxelVisibility();
	void ToggleLiveVoxelVisibility();

	void DecreaseCanonicalVoxelOpacity();
	void IncreaseCanonicalVoxelOpacity();

	void ToggleInterestVoxelVisibility();

	void DecreaseLiveVoxelOpacity();
	void IncreaseLiveVoxelOpacity();

	//*** update /change GUI ***
	void UpdateIterationDisplay();
	void PreviousBackgroundColor();
	void NextBackgroundColor();

	//*** viewport navigation ***
	void MoveFocusToHighlightAt(int hash, int localId);
	void MoveFocusToVoxelAt(Vector3d absoluteCoordinates);
	void RefocusAtCurrentHighlight();
	void MoveFocusToNextHighlight();
	void MoveFocusToPreviousHighlight();
};