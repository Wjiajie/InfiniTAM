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

//========================= TEMPORARY FLAGS FOR DEBUGGING KILLING FUSION FEATURE =======================================

#define _DEBUG
#ifdef _DEBUG



//#define USE_COLOR
//*** FLAGS FOR MODES OF COMPUTING THE WARP ***
#define WARP_COMPUTE_MODE_FULL 0
#define WARP_COMPUTE_MODE_NO_LEVEL_SET 1
#define WARP_COMPUTE_MODE_NO_KILLING 2
#define WARP_COMPUTE_MODE_DATA_ONLY 3

//#define OLD_LEVEL_SET_TERM //Old and seemingly incorrect way to compute the Level Set term
#define TRUNCATION_TREATMENT_DEBUG


#define WARP_COMPUTE_MODE WARP_COMPUTE_MODE_FULL
//#define WARP_COMPUTE_MODE WARP_COMPUTE_MODE_NO_LEVEL_SET
//#define WARP_COMPUTE_MODE WARP_COMPUTE_MODE_NO_KILLING
//#define WARP_COMPUTE_MODE WARP_COMPUTE_MODE_DATA_ONLY



//*** LOGGING FOR 3D VISUAL DEBUGGING***
#define _LOGGER
#ifdef _LOGGER

//#define OSCILLATION_DETECTION

#include "Utils/ITMSceneLogger.h"
#define FRAME_OF_INTEREST 1
#ifndef STRINGIFY
#define STRINGIFY(x) #x
#endif
#define TOSTRING(x) STRINGIFY(x)
#define SCENE_NAME "snoopy"
#if WARP_COMPUTE_MODE == WARP_COMPUTE_MODE_FULL
#define SCENE_POSTFIX "_all_terms"
#elif WARP_COMPUTE_MODE == WARP_COMPUTE_MODE_NO_LEVEL_SET
#define SCENE_POSTFIX "_no_level_set"
#elif WARP_COMPUTE_MODE == WARP_COMPUTE_MODE_NO_KILLING
#define SCENE_POSTFIX "_no_killing"
#elif WARP_COMPUTE_MODE == WARP_COMPUTE_MODE_DATA_ONLY
#define SCENE_POSTFIX "_data_only"
#endif //WARP_COMPUTE_MODE
#define SCENE_PATH "/media/algomorph/Data/Reconstruction/debug_output/" SCENE_NAME "/frame_" TOSTRING(FRAME_OF_INTEREST) SCENE_POSTFIX


//#define SAVE_SCENE_DATA
#ifdef SAVE_SCENE_DATA
// =========================== Step 1 for sdf viz prep =================================================================
//#define SAVE_VOXELS_AND_INDEX
#ifdef SAVE_VOXELS_AND_INDEX
#define SAVE_WARP
#ifdef OSCILLATION_DETECTION
#define LOG_HIGHLIGHTS
#endif
#else
// =========================== Step 2 for sdf viz prep =================================================================
// loads the scene at the frame and saves warps for interest regions
#define LOG_INTEREST_REGIONS

#ifdef LOG_INTEREST_REGIONS
#define FILTER_HIGHLIGHTS
#ifdef FILTER_HIGHLIGHTS
#define HIGHLIGHT_MIN_RECURRENCES 2
#endif
#define RECORD_CONTINOUS_HIGHLIGHTS
#endif //LOG INTEREST REGIONS
#endif //SAVE_VOXELS_AND_INDEX
#endif //SAVE SCENE DATA


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
#define PRINT_SINGLE_VOXEL_RESULT //Caution: very verbose!
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


#ifdef PRINT_ENERGY_STATS
#define WRITE_ENERGY_STATS_TO_FILE
#ifdef WRITE_ENERGY_STATS_TO_FILE
#if defined(SAVE_VOXELS_AND_INDEX) || defined(LOG_INTEREST_REGIONS)
#define ENERGY_STATS_FILE_PATH SCENE_PATH "/energy_stats.txt"
#else
#define ENERGY_STATS_FILE_PATH "/media/algomorph/Data/Reconstruction/debug_output/energy_stats/" SCENE_NAME "_frame_" TOSTRING(FRAME_OF_INTEREST) SCENE_POSTFIX "_energy_stats.txt"
#endif
#endif
#endif

#endif //ifdef _DEBUG