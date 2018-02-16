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


class SDFViz;

/**
 * \brief A standard VTK trackball interator style with added functionality for
 * some keyboard keys
 */
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera {


public:
	static KeyPressInteractorStyle* New();

vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);
	SDFViz* parent;

	virtual void OnKeyPress();

};

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
	friend class KeyPressInteractorStyle;

public:
	//================= CONSTANTS ================
	static const std::array<double, 4> canonicalNegativeSDFVoxelColor;
	static const std::array<double, 4> canonicalPositiveSDFVoxelColor;
	static const std::array<double, 4> canonicalNegativeInterestSDFVoxelColor;
	static const std::array<double, 4> canonicalPositiveInterestSDFVoxelColor;
	static const std::array<double, 4> canonicalHighlightSDFVoxelColor;
	static const std::array<double, 3> canonicalHashBlockEdgeColor;
	static const std::array<double, 4> liveNegativeSDFVoxelColor;
	static const std::array<double, 4> livePositiveSDFVoxelColor;
	static const std::array<double, 3> liveHashBlockEdgeColor;


	//================= CONSTRUCTORS/DESTRUCTORS =
	SDFViz(std::string pathToScene);
	virtual ~SDFViz();
	//================= MEMBER FUNCTIONS ==================
	int Run();

private:
	//================= CONSTANTS ================

	//================= MEMBER VARIABLES ===================
	//data loader
	ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>* sceneLogger;

	//visualization setup
	vtkSmartPointer<vtkRenderer> sdfRenderer;
	vtkSmartPointer<vtkRenderer> topRenderer;
	// The render window is the actual GUI window
	// that appears on the computer screen
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	// The render window interactor captures mouse events
	// and will perform appropriate camera or actor manipulation
	// depending on the nature of the events.
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
	vtkSmartPointer<vtkTextActor> iterationIndicator;

	// Structures for rendering scene geometry with VTK
	CanonicalVizPipe canonicalScenePipe;
	SDFSceneVizPipe<ITMVoxelLive, ITMVoxelIndex> liveScenePipe;
	HighlightVisualization highlightVisualizer;

	//Holds warp & warp update state for the canonical scene
	vtkSmartPointer<vtkFloatArray> allWarpBuffer;
	vtkSmartPointer<vtkFloatArray> interestWarpBuffer;
	//frame that we're loading scene & warps for
	int frameIndex;
	//to keep track of iteration number in the optimization
	int iterationIndex;

	//Holds highlights in the scene
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> highlights;
	const std::vector<ITMHighlightIterationInfo>* currentHighlight;

	//================ MEMBER FUNCTIONS =====================
	static Vector3d ComputeCameraRightVector(vtkCamera* camera);
	void InitializeRendering();
	//scene voxel size should be known
	void InitializeWarpBuffers();

	void DrawLegend();
	void DrawIterationCounter();
	void DrawDummyMarkers();

	bool NextNonInterestWarps();
	bool PreviousNonInterestWarps();
	bool NextInterestWarps();
	bool PreviousInterestWarps();

	void MoveFocusToNextHighlight();
	void MoveFocusToPreviousHighlight();

	//** visibility / opacity **
	void ToggleCanonicalHashBlockVisibility();
	void ToggleLiveHashBlockVisibility();

	void ToggleCanonicalVoxelVisibility();
	void ToggleLiveVoxelVisibility();

	void DecreaseCanonicalVoxelOpacity();
	void IncreaseCanonicalVoxelOpacity();

	void UpdateIteration(unsigned int newValue);

	void MoveFocusToHighlightAt(int hash, int localId);
	void RefocusAtCurrentHighlight();
};