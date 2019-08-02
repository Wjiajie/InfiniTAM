//  ================================================================
//  Created by Gregory Kramida on 7/24/19.
//  Copyright (c) 2019 Gregory Kramida
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

#ifdef __JETBRAINS_IDE__
	#define __host__
	#define __device__
	#define __shared__
	#define __constant__
	#define __global__

	// This is slightly mental, but gets it to properly index device function calls like __popc and whatever.
	#define __CUDACC__
	#include <device_functions.h>

	// These headers are all implicitly present when you compile CUDA with clang. Clion doesn't know that, so
	// we include them explicitly to make the indexer happy. Doing this when you actually build is, obviously,
	// a terrible idea :D
	#include <__clang_cuda_builtin_vars.h>
	#include <__clang_cuda_intrinsics.h>
	#include <__clang_cuda_math_forward_declares.h>
	#include <__clang_cuda_complex_builtins.h>
	#include <__clang_cuda_cmath.h>
#endif // __JETBRAINS_IDE__

