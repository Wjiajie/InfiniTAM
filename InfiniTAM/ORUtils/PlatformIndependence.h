// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#if !defined(__METALC__)
#include <cstdio>
#include <stdexcept>
#endif

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
// for CUDA device code
#define _CPU_AND_GPU_CODE_ __device__
#else
#define _CPU_AND_GPU_CODE_ 
#endif

#if defined(__CUDACC__)
// for CUDA device code
#define _CPU_AND_GPU_CODE_TEMPLATE_ __device__
#else
#define _CPU_AND_GPU_CODE_TEMPLATE_
#endif

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
// for CUDA device code
#define _CPU_AND_GPU_CONSTANT_ __constant__
#else
#define _CPU_AND_GPU_CONSTANT_
#endif

// for METAL device code
#if defined(__METALC__)
#define THREADPTR(x) thread x
#define DEVICEPTR(x) device x
#define THREADGRPPTR(x) threadgroup x
#define CONSTPTR(x) constant x
#else
#define THREADPTR(x) x
#define DEVICEPTR(x) x
#define THREADGROUPPTR(x) x
#define CONSTPTR(x) x
#endif

#define TOSTRING(s) TOSTRING_INTERNAL(s)
#define TOSTRING_INTERNAL(s) #s

#ifdef ANDROID
#define DIEWITHEXCEPTION(x) { fprintf(stderr, "%s\n", x); exit(-1); }
#define DIEWITHEXCEPTION_REPORTLOCATION(x) { fprintf(stderr, "%s\n", x x __FILE__ TOSTRING(__LINE__)); exit(-1); }
#else
#define DIEWITHEXCEPTION(x) throw std::runtime_error(x)
#define DIEWITHEXCEPTION_REPORTLOCATION(x) throw std::runtime_error( x "\n[" __FILE__ ":" TOSTRING(__LINE__) "]")
#endif
