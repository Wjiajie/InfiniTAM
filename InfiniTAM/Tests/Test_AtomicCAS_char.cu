//  ================================================================
//  Created by Gregory Kramida on 12/12/19.
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
#include <iostream>
#include "Test_AtomicCAS_char.h"
#include "../ORUtils/JetbrainsCUDASyntax.hpp"
#include "../ORUtils/CUDADefines.h"
#include "../ORUtils/MemoryBlock.h"
//#include "../ITMLib/Utils/ITMCUDAUtils.h"




namespace {

__device__ static inline char atomicAdd(char* address, char val)

{
	auto *base_address = (unsigned int *) ((char *)address - ((size_t)address & 3));
	unsigned int selectors[] = {0x3214, 0x3240, 0x3410, 0x4210};
	unsigned int sel = selectors[(size_t)address & 3];
	unsigned int old, assumed, new_val, replacement;

	old = *base_address;

//
//	printf("thread: %d, block: %d, assumed: 0x%02X, val: 0x%02X, val: 0x%08X, (size_t)address & 3: %d, old: 0x%08X, uint32_val: 0x%08X, uint32_assumed: 0x%08X \n",
//	       threadIdx.x, blockIdx.x,
//	       (int)assumed, (int)val, val,
//	       (size_t)address & 3, old, uint32_val, uint32_assumed);
	do{
		assumed = old;
		// replace bits in old that pertain to the char address with those from val
		new_val = __byte_perm(old, 0, ((size_t)address & 3)) + val;
		replacement = __byte_perm(old, new_val, sel);
		old = atomicCAS(base_address, assumed, replacement);
	}while(old != assumed);
	return __byte_perm(old, 0, ((size_t)address & 3));
}


__device__ static inline bool compare_exchange(char* address, char expected, char desired)

{
	auto *base_address = (unsigned int *) ((char *)address - ((size_t)address & 3));
	unsigned int selectors[] = {0x3214, 0x3240, 0x3410, 0x4210};
	size_t byte_index = (size_t)address & 3;
	unsigned int sel = selectors[byte_index];
	unsigned int old, assumed, new_val, replacement;
	char former_value;

	old = *base_address;
	new_val = desired;

	do {
		assumed = old;
		replacement = __byte_perm(old, new_val, sel);
		old = atomicCAS(base_address, assumed, replacement);
		former_value = (char)((old >> (byte_index * 8)) & 0x000000FF);
	}while(expected == former_value && assumed != old);

	return (expected != old);
}

__global__ void CAS_increment(char* data, char* output){
	int index = threadIdx.x;
	char val = data[index];
	//char old_val = atomicAdd(data + index, 1);

	output[index * blockIdx.x] = compare_exchange(data + index, 0, 14);
}

}

bool run_CAS_test(){
	const int data_size = 4;
	const int count_of_threads_to_modify_each_item = 4;
	dim3 block_size(data_size);
	dim3 grid_size(count_of_threads_to_modify_each_item);
	ORUtils::MemoryBlock<char> data(data_size,true, true);
	ORUtils::MemoryBlock<char> output(data_size * count_of_threads_to_modify_each_item,true, true);
	char* data_CPU = data.GetData(MEMORYDEVICE_CPU);
	memset(data_CPU,0,data_size * sizeof(char));

	std::cout << "data: ";
	for(int i = 0; i < data_size; i++){
		std::cout << (int)data_CPU[i] << ", ";
	}
	std::cout << std::endl;

	data.UpdateDeviceFromHost();
	CAS_increment << < grid_size, block_size >> >(data.GetData(MEMORYDEVICE_CUDA), output.GetData(MEMORYDEVICE_CUDA));
	ORcudaKernelCheck;
	data.UpdateHostFromDevice();
	output.UpdateHostFromDevice();

	std::cout << "data: ";
	for(int i = 0; i < data_size; i++){
		std::cout << (int)data_CPU[i] << ", ";
	}
	std::cout << std::endl;

	char* output_CPU = output.GetData(MEMORYDEVICE_CPU);
	std::cout << "output: ";
	for(int i = 0; i < data_size * count_of_threads_to_modify_each_item; i++){
		std::cout << (int)output_CPU[i] << ", ";
	}
	std::cout << std::endl;


	return true;
}