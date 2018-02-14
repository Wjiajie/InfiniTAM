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

#define _DEBUG
#ifdef _DEBUG
//*** FLAGS FOR HOW TO HANDLE OSCILLATIONS ***
//#define OLD_UGLY_WAY //simply cuts down warp updates by half



//*** FLANGS FOR MODES OF COMPUTING THE WARP ***
#define WARP_COMPUTE_MODE_FULL 0
#define WARP_COMPUTE_MODE_NO_KILLING 1
#define WARP_COMPUTE_MODE_NO_LEVEL_SET 2
#define WARP_COMPUTE_MODE_DATA_ONLY 3

#define WARP_COMPUTE_MODE WARP_COMPUTE_MODE_NO_LEVEL_SET


//*** LOGGING FOR 3D VISUAL DEBUGGING***
#define _LOGGER
#ifdef _LOGGER

#define OSCILLATION_DETECTION

#include "Utils/ITMSceneLogger.h"
#define FRAME_OF_INTEREST 2
#ifndef STRINGIFY
#define STRINGIFY(x) #x
#endif
#define TOSTRING(x) STRINGIFY(x)
#define SCENE_NAME "scene_snoopy"
#if WARP_COMPUTE_MODE == WARP_COMPUTE_MODE_FULL
#define SCENE_PATH "/media/algomorph/Data/Reconstruction/debug_output/" SCENE_NAME "_frame_" TOSTRING(FRAME_OF_INTEREST)
#elif WARP_COMPUTE_MODE == WARP_COMPUTE_MODE_NO_LEVEL_SET
#define SCENE_PATH "/media/algomorph/Data/Reconstruction/debug_output/" SCENE_NAME "_frame_" TOSTRING(FRAME_OF_INTEREST) "_no_level_set"
#elif WARP_COMPUTE_MODE == WARP_COMPUTE_MODE_NO_KILLING
#define SCENE_PATH "/media/algomorph/Data/Reconstruction/debug_output/" SCENE_NAME "_frame_" TOSTRING(FRAME_OF_INTEREST) "_no_killing"
#elif WARP_COMPUTE_MODE == WARP_COMPUTE_MODE_DATA_ONLY
#define SCENE_PATH "/media/algomorph/Data/Reconstruction/debug_output/" SCENE_NAME "_frame_" TOSTRING(FRAME_OF_INTEREST) "_data_only"
#endif //WARP_COMPUTE_MODE


//#define SAVE_FRAME

#ifdef SAVE_FRAME
#define SAVE_WARP
#ifdef OSCILLATION_DETECTION
#define LOG_HIGHLIGHTS
#endif
#else
#define LOG_INTEREST_REGIONS //loads the scene at the frame and saves warps for interest regions
#ifdef LOG_INTEREST_REGIONS
#define FILTER_HIGHLIGHTS
#ifdef FILTER_HIGHLIGHTS
#define HIGHLIGHT_MIN_RECURRENCES 2
#endif
#else
//#define LOAD_FRAME //simply loads the scene at the frame before optimization
#endif //LOG INTEREST REGIONS
#endif //SAVE FRAME


#endif //ifdef _LOGGER

#include <opencv2/core/mat.hpp>

//*** 2D RASTERIZATION FOR VISUAL DEBUGGING ***

//#define RASTERIZE_CANONICAL_SCENE
//#define RASTERIZE_LIVE_SCENE
//#define DRAW_IMAGE
#if defined(DRAW_IMAGE) || defined(RASTERIZE_CANONICAL_SCENE) || defined(RASTERIZE_LIVE_SCENE)
#include "../../Utils/ITMSceneSliceRasterizer.h"
#endif

//*** DEBUG OUTPUT MESSAGES FOR UPDATE WARP ON CPU ***

//#define PRINT_TIME_STATS //-- needs rearranging of TICs and TOCs
//#define PRINT_SINGLE_VOXEL_RESULT //Caution: very verbose!
#define PRINT_MAX_WARP_AND_UPDATE
#define PRINT_ENERGY_STATS
#define PRINT_ADDITIONAL_STATS
#define PRINT_DEBUG_HISTOGRAM
//#define OPENMP_WARP_UPDATE_COMPUTE_DISABLE
//***

#ifdef PRINT_TIME_STATS
#define TIC(var)\

#define TOC(var)\
    auto end_##var = std::chrono::steady_clock::now();\
    auto diff_##var = end_##var - start_##var;\
    var += std::chrono::duration <double, std::milli> (diff_##var).count();
#else
#define TIC(var)
#define TOC(var)
#endif

#ifdef OSCILLATION_DETECTION
//#define OSCILLATION_TREATMENT
#ifdef OSCILLATION_TREATMENT
#define OLD_UGLY_WAY
#endif
#endif //OSCILLATION_DETECTION


#ifdef PRINT_ENERGY_STATS
#define WRITE_ENERGY_STATS_TO_FILE
#ifdef WRITE_ENERGY_STATS_TO_FILE
#if defined(SAVE_FRAME) || defined(LOG_INTEREST_REGIONS)
#define ENERGY_STATS_FILE_PATH SCENE_PATH "/energy_stats.txt"
#else
#define ENERGY_STATS_FILE_PATH SCENE_PATH "_energy_stats.txt"
#endif
#endif
#endif

#endif //ifdef _DEBUG