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
#pragma once

//========================= TEMPORARY FLAGS FOR DEBUGGING DYNAMIC FUSION FEATURE =======================================
#define _DEBUG

#ifdef _DEBUG

#include "Utils/FileIO/ITMSceneLogger.h"
#include <opencv2/core/mat.hpp>

//different logging parameters
#define FOCUS_SLICE_RADIUS 3

#define debug_print(something) std::cout << yellow << something << reset << std::endl


#endif //ifdef _DEBUG