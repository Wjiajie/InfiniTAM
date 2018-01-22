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

/**
 * \brief SDF Visualization application main class.
 */
class SDFViz {
	friend class KeyPressInteractorStyle;

public:
	//================= CONSTANTS ================
	static const double canonicalNegativeSDFVoxelColor[4];
	static const double canonicalPositiveSDFVoxelColor[4];
	static const double canonicalHashBlockEdgeColor[3];
	static const double liveNegativeSDFVoxelColor[4];
	static const double livePositiveSDFVoxelColor[4];
	static const double liveHashBlockEdgeColor[3];


	//================= CONSTRUCTORS/DESTRUCTORS =
	SDFViz();
	virtual ~SDFViz();
	//================= METHODS ==================
	int run();

private:
	//================= CONSTANTS ================

	//================= FIELDS ===================
	//data loader
	ITMSceneLogger <ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>* sceneLogger;

	//visualization setup
	// The renderer generates the image
	// which is then displayed on the render window.
	// It can be thought of as a scene to which the actor is added
	vtkSmartPointer<vtkRenderer> renderer;
	// The render window is the actual GUI window
	// that appears on the computer screen
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	// The render window interactor captures mouse events
	// and will perform appropriate camera or actor manipulation
	// depending on the nature of the events.
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;

	// Structures for rendering scene geometry with VTK
	SDFSceneVizPipe<ITMVoxelCanonical, ITMVoxelIndex> canonicalScenePipe;
	SDFSceneVizPipe<ITMVoxelLive, ITMVoxelIndex> liveScenePipe;

	//Holds warp & warp update state for the canonical scene
	vtkSmartPointer<vtkFloatArray> warpBuffer;

	Vector3i minPoint, maxPoint;
	// Rendering limits/boundaries
	// (probably temporary since more elaborate methods of rendering voxel subset will be employed later)
	Vector3i minAllowedPoint;
	Vector3i maxAllowedPoint;

	//================ METHODS =====================
	void InitializeRendering();
	//scene voxel size should be known
	void InitializeWarpBuffers();

	void DrawLegend();

	void SetUpSceneHashBlockMapper(vtkAlgorithmOutput* sourceOutput,
	                               vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                               vtkSmartPointer<vtkPolyData>& pointsPolydata);
	void SetUpSDFColorLookupTable(vtkSmartPointer<vtkLookupTable>& table,
	                              const double rgbaFirstColor[4], const double rgbaSecondColor[4]);
	void SetUpGlyph(vtkAlgorithmOutput* sourceOutput,
	                vtkSmartPointer<vtkPolyData>& polydata, vtkSmartPointer<vtkGlyph3D>& glyph);
	void SetUpSceneVoxelMapper(vtkSmartPointer<vtkPolyDataMapper>& mapper,
	                           vtkSmartPointer<vtkLookupTable>& table,
	                           vtkSmartPointer<vtkGlyph3D>& glyph);
	void SetUpSceneVoxelMapper(vtkAlgorithmOutput* sourceOutput,
	                           vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                           vtkSmartPointer<vtkLookupTable>& table,
	                           vtkSmartPointer<vtkExtractPolyDataGeometry> extractor);
	void SetUpSceneVoxelMapper(vtkAlgorithmOutput* sourceOutput,
	                           vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                           vtkSmartPointer<vtkLookupTable>& table,
	                           vtkSmartPointer<vtkPolyData>& pointsPolydata);


	void UpdateVoxelPositionsFromWarpBuffer();
	bool NextWarps();
	bool PreviousWarps();


	//** visibility / opacity **
	void ToggleCanonicalHashBlockVisibility();
	void ToggleLiveHashBlockVisibility();

	void ToggleCanonicalVoxelVisibility();
	void ToggleLiveVoxelVisibility();

	void DecreaseCanonicalVoxelOpacity();
	void IncreaseCanonicalVoxelOpacity();


};


