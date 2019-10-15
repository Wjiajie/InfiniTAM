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
#if !(defined(__CUDACC__) && defined(__CUDA_ARCH__))
#include <atomic>

inline
float atomicAdd_CPU(std::atomic<float>& variable, float addend) {
	auto current = variable.load();
	while (!variable.compare_exchange_weak(current, current + addend, std::memory_order_relaxed, std::memory_order_relaxed));
	return current;
}

inline
int atomicAdd_CPU(std::atomic<int>& variable, int addend){
	return variable.fetch_add(addend, std::memory_order_relaxed);
}
#endif

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
inline void initializeAtomicInt(int* var, int value){
	ORcudaSafeCall(cudaMalloc((void**)&var,sizeof(int)));
	ORcudaSafeCall(cudaMemcpy(var, &value, sizeof(int),cudaMemcpyHostToDevice));
}
inline void initializeAtomicFloat(float* var, float value){
	ORcudaSafeCall(cudaMalloc((void**)&var,sizeof(float)));
	ORcudaSafeCall(cudaMemcpy(var, &value, sizeof(float),cudaMemcpyHostToDevice));
}
#else
inline void initializeAtomicInt(std::atomic<int>& var, int value){
	var.store(value);
}
inline void initializeAtomicFloat(std::atomic<float>& var, float value){
	var.store(value);
}
#endif

#define INITIALIZE_ATOMIC_INT(name, value) initializeAtomicInt( name , value )
#define INITIALIZE_ATOMIC_FLOAT(name, value) initializeAtomicFloat( name , value )

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
// for CUDA device code
#define DECLARE_ATOMIC_INT(name)  int* name
#define DECLARE_ATOMIC_FLOAT(name) float* name
#define CLEAN_UP_ATOMIC(name) ORcudaSafeCall (cudaFree(name))
#define GET_ATOMIC_VALUE(name) (* name)
#else
#define DECLARE_ATOMIC_INT(name)  std::atomic<int> name
#define DECLARE_ATOMIC_FLOAT(name) std::atomic<float> name
#define CLEAN_UP_ATOMIC(name) ;
#define GET_ATOMIC_VALUE(name) (name .load())
#endif

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
#define ATOMIC_ADD(name, value) atomicAdd( name , value )
#else
#define ATOMIC_ADD(name, value) atomicAdd_CPU( name, value)
#endif

