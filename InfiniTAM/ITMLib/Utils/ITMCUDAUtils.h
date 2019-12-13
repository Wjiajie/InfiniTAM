// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once


template<class T>
inline __device__ void warpReduce(volatile T* sdata, int tid) {
	sdata[tid] += sdata[tid + 32];
	sdata[tid] += sdata[tid + 16];
	sdata[tid] += sdata[tid + 8];
	sdata[tid] += sdata[tid + 4];
	sdata[tid] += sdata[tid + 2];
	sdata[tid] += sdata[tid + 1];
}

inline __device__ void warpReduce3(volatile float* sdata, int tid) {
	sdata[3*tid+0] += sdata[3*(tid + 32)+0];
	sdata[3*tid+1] += sdata[3*(tid + 32)+1];
	sdata[3*tid+2] += sdata[3*(tid + 32)+2];
	sdata[3*tid+0] += sdata[3*(tid + 16)+0];
	sdata[3*tid+1] += sdata[3*(tid + 16)+1];
	sdata[3*tid+2] += sdata[3*(tid + 16)+2];
	sdata[3*tid+0] += sdata[3*(tid + 8)+0];
	sdata[3*tid+1] += sdata[3*(tid + 8)+1];
	sdata[3*tid+2] += sdata[3*(tid + 8)+2];
	sdata[3*tid+0] += sdata[3*(tid + 4)+0];
	sdata[3*tid+1] += sdata[3*(tid + 4)+1];
	sdata[3*tid+2] += sdata[3*(tid + 4)+2];
	sdata[3*tid+0] += sdata[3*(tid + 2)+0];
	sdata[3*tid+1] += sdata[3*(tid + 2)+1];
	sdata[3*tid+2] += sdata[3*(tid + 2)+2];
	sdata[3*tid+0] += sdata[3*(tid + 1)+0];
	sdata[3*tid+1] += sdata[3*(tid + 1)+1];
	sdata[3*tid+2] += sdata[3*(tid + 1)+2];
}

template <typename T> 
__device__ int computePrefixSum_device(uint element, T *sum, int localSize, int localId)
{
	// TODO: should be localSize...
	__shared__ uint prefixBuffer[16 * 16];
	__shared__ uint groupOffset;

	prefixBuffer[localId] = element;
	__syncthreads();

	int s1, s2;

	for (s1 = 1, s2 = 1; s1 < localSize; s1 <<= 1)
	{
		s2 |= s1;
		if ((localId & s2) == s2) prefixBuffer[localId] += prefixBuffer[localId - s1];
		__syncthreads();
	}

	for (s1 >>= 2, s2 >>= 1; s1 >= 1; s1 >>= 1, s2 >>= 1)
	{
		if (localId != localSize - 1 && (localId & s2) == s2) prefixBuffer[localId + s1] += prefixBuffer[localId];
		__syncthreads();
	}

	if (localId == 0 && prefixBuffer[localSize - 1] > 0) groupOffset = atomicAdd(sum, prefixBuffer[localSize - 1]);
	__syncthreads();

	int offset;// = groupOffset + prefixBuffer[localId] - 1;
	if (localId == 0) {
		if (prefixBuffer[localId] == 0) offset = -1;
		else offset = groupOffset;
	} else {
		if (prefixBuffer[localId] == prefixBuffer[localId - 1]) offset = -1;
		else offset = groupOffset + prefixBuffer[localId-1];
	}

	return offset;
}


__device__ static inline void atomicMin(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fminf(val, __int_as_float(assumed))));
	} while (assumed != old);
}

__device__ static inline void atomicMax(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fmaxf(val, __int_as_float(assumed))));
	} while (assumed != old);
}

__device__ static inline void atomicMin(double* address, double val)
{
	unsigned long long int* address_as_i = (unsigned long long int*)address;
	unsigned long long int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
		                  __double_as_longlong(::fmin(val, __longlong_as_double(assumed))));
	} while (assumed != old);
}

__device__ static inline void atomicMax(double* address, double val)
{
	unsigned long long int* address_as_i = (unsigned long long int*)address;
	unsigned long long int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
		                  __double_as_longlong(::fmax(val, __longlong_as_double(assumed))));
	} while (assumed != old);
}
#if !defined(__CUDA_ARCH__) || __CUDA_ARCH__ >= 600
#else
__device__ double atomicAdd(double* address, double val)
{
	unsigned long long int* address_as_ull = (unsigned long long int*)address;
	unsigned long long int old = *address_as_ull, assumed;
	do {
		assumed = old;
		old = atomicCAS(address_as_ull, assumed,
		                __double_as_longlong(val + __longlong_as_double(assumed)));
	} while (assumed != old);
	return __longlong_as_double(old);
}
#endif

__device__ static inline char atomicCAS(char* address, char assumed, char val)

{
	unsigned int *base_address = (unsigned int *)((char *)address - ((size_t)address & 3));
	unsigned int selectors[] = {0x3214, 0x3240, 0x3410, 0x4210};
	unsigned int sel = selectors[(size_t)address & 3];
	unsigned int _old, _new, _assumed;

	_old = *base_address;
	_new = val;
	_assumed = assumed;

	// replace bits in _old that pertain to the char address with those from val
	unsigned int uint32_val = __byte_perm(_old, _new, sel);
	// replace bits in _old that pertain to the char address with those from assumed
	unsigned int uint32_assumed = __byte_perm(_old, _assumed, sel);

	printf("thread: %d, block: %d, assumed: %d, val: %d, _new: %d, (size_t)address & 3: %d, _old: %d, uint32_val: %d, uint32_assumed: %d \n",
			threadIdx.x, blockIdx.x,
           (int)assumed, (int)val, _new,
			(size_t)address & 3, _old, uint32_val, uint32_assumed);

	_old = atomicCAS(base_address, uint32_assumed, uint32_val);

	return (char)__byte_perm(_old, 0, ((size_t) address & 3));
}

template<typename T>
__global__ void memsetKernel_device(T *devPtr, const T val, size_t nwords)
{
	size_t offset = threadIdx.x + blockDim.x * blockIdx.x;
	if (offset >= nwords) return;
	devPtr[offset] = val;
}

template<typename T>
__global__ void memsetKernelLarge_device(T *devPtr, const T val, size_t nwords)
{
	size_t offset = threadIdx.x + blockDim.x * (blockIdx.x + blockIdx.y * gridDim.x);
	if (offset >= nwords) return;
	devPtr[offset] = val;
}

template<typename T>
inline void memsetKernel(T *devPtr, const T val, size_t nwords)
{
	dim3 blockSize(256);
	dim3 gridSize((int)ceil((float)nwords / (float)blockSize.x));
	if (gridSize.x <= 65535) {
		memsetKernel_device<T> <<<gridSize,blockSize>>>(devPtr, val, nwords);
		ORcudaKernelCheck;
	} else {
		gridSize.x = (int)ceil(sqrt((float)gridSize.x));
		gridSize.y = (int)ceil((float)nwords / (float)(blockSize.x * gridSize.x));
		memsetKernelLarge_device<T> <<<gridSize,blockSize>>>(devPtr, val, nwords);
		ORcudaKernelCheck;
	}
}

template<typename T>
__global__ void fillArrayKernel_device(T *devPtr, size_t nwords)
{
	size_t offset = threadIdx.x + blockDim.x * blockIdx.x;
	if (offset >= nwords) return;
	devPtr[offset] = offset;
}

template<typename T>
inline void fillArrayKernel(T *devPtr, size_t nwords)
{
	dim3 blockSize(256);
	dim3 gridSize((int)ceil((float)nwords / (float)blockSize.x));
	fillArrayKernel_device<T> <<<gridSize,blockSize>>>(devPtr, nwords);
	ORcudaKernelCheck;
}

