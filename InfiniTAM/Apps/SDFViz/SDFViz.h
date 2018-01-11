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

#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkObjectFactory.h>

//local
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Objects/Scene/ITMScene.h"

using namespace ITMLib;
namespace ITMLib {
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneWarpFileIO;
}


class vtkRenderer;
class vtkRenderWindow;
class vtkPoints;
class vtkPolyData;

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

/**
 * \brief SDF Visualization application main class.
 */
class SDFViz {
	friend class KeyPressInteractorStyle;
public:
	//================= CONSTANTS ================
	static const double maxVoxelDrawSize;
	static const double canonicalNegativeSDFColor[4];
	static const double canonicalPositiveSDFColor[4];
	static const double liveNegativeSDFColor[4];
	static const double livePositiveSDFColor[4];

	//================= CONSTRUCTORS/DESTRUCTORS =
	SDFViz();
	virtual ~SDFViz();
	//================= METHODS ==================
	int run();

private:
	//================= CONSTANTS ================
	static const char* colorPointAttributeName;
	static const char* scalePointAttributeName;
	//================= FIELDS ===================
	//data loader
	ITMSceneWarpFileIO <ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>* sceneLogger;

	//data structures
	ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* canonicalScene;
	ITMScene<ITMVoxelLive, ITMVoxelIndex>* liveScene;

	// Structures for rendering scene geometry with VTK
	vtkSmartPointer<vtkPoints> canonicalPoints;
	vtkSmartPointer<vtkPolyData> canonicalPolydata;
	vtkSmartPointer<vtkPoints> livePoints;
	vtkSmartPointer<vtkPolyData> livePolydata;

	//visualization setup
	// The renderer generates the image
	// which is then displayed on the render window.
	// It can be thought of as a scene to which the actor is added
	vtkSmartPointer<vtkRenderer> renderer;
	// The render window is the actual GUI window
	// that appears on the computer screen
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	// Rendering limits/boundaries
	// (probably temporary since more elaborate methods of not rendering distant voxels will be employed later)
	Vector3i minAllowedPoint;
	Vector3i maxAllowedPoint;

	// The render window interactor captures mouse events
	// and will perform appropriate camera or actor manipulation
	// depending on the nature of the events.
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;

	//================ METHODS =====================
	void InitializeRendering();
	bool HashBlockIsAtLeastPartiallyWithinBounds(Vector3i hashBlockPositionVoxels);
	template<typename TVoxel>
	void GenerateInitialScenePoints(ITMScene<TVoxel, ITMVoxelIndex>* scene, vtkSmartPointer<vtkPoints>& points,
	                                vtkSmartPointer<vtkPolyData>& polydata);
	void DrawLegend();

};

