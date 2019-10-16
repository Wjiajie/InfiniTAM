//  ================================================================
//  Created by Gregory Kramida on 10/10/19.
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
#if !defined(__CUDACC__)
#include <atomic>

template<typename T>
inline
T atomicAdd_CPU(std::atomic<T>& variable, T addend) {
	auto current = variable.load();
	while (!variable.compare_exchange_weak(current, current + addend, std::memory_order_relaxed, std::memory_order_relaxed));
	return current;
}

template<>
inline
int atomicAdd_CPU<int>(std::atomic<int>& variable, int addend){
	return variable.fetch_add(addend, std::memory_order_relaxed);
}

template<>
inline
unsigned int atomicAdd_CPU<unsigned int>(std::atomic<unsigned int>& variable, unsigned int addend){
	return variable.fetch_add(addend, std::memory_order_relaxed);
}
#endif

#if defined(__CUDACC__)
template <typename T>
inline void initializeAtomic(T* var, T value){
	ORcudaSafeCall(cudaMalloc((void**)&var, sizeof(T)));
	ORcudaSafeCall(cudaMemcpy(var, &value, sizeof(T), cudaMemcpyHostToDevice));
}
#else
template <typename T>
inline void initializeAtomic(std::atomic<T>& var, T value){
	var.store(value);
}
#endif

#define INITIALIZE_ATOMIC(name, value) initializeAtomic( name , value )

#if defined(__CUDACC__)
// for CUDA device code
#define DECLARE_ATOMIC_INT(name)  int* name
#define DECLARE_ATOMIC_UINT(name) unsigned int* name
#define DECLARE_ATOMIC_FLOAT(name) float* name
#define CLEAN_UP_ATOMIC(name) ORcudaSafeCall (cudaFree(name))
#define GET_ATOMIC_VALUE(name) (* name)
#define ATOMIC_FLOAT_ARGUMENT_TYPE float*
#define ATOMIC_INT_ARGUMENT_TYPE int*
#else
#define DECLARE_ATOMIC_INT(name)  std::atomic<int> name
#define DECLARE_ATOMIC_UINT(name)  std::atomic<unsigned int> name
#define DECLARE_ATOMIC_FLOAT(name) std::atomic<float> name
#define CLEAN_UP_ATOMIC(name) ;
#define GET_ATOMIC_VALUE(name) (name .load())
#define ATOMIC_FLOAT_ARGUMENT_TYPE std::atomic<float>&
#define ATOMIC_INT_ARGUMENT_TYPE std::atomic<int>&
#endif

#if defined(__CUDACC__)
#define ATOMIC_ADD(name, value) atomicAdd( name , value )
#define _DEVICE_WHEN_AVAILABLE_ __device__
#else
#define ATOMIC_ADD(name, value) atomicAdd_CPU( name, value)
#define _DEVICE_WHEN_AVAILABLE_
#endif


