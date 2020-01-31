// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "VoxelBlockHash.h"
#include "VoxelTypes.h"

template<typename T> _CPU_AND_GPU_CODE_ inline int HashCodeFromBlockPosition(const THREADPTR(T) & blockPos) {
	return (((uint)blockPos.x * 73856093u) ^ ((uint)blockPos.y * 19349669u) ^ ((uint)blockPos.z * 83492791u)) & (uint)VOXEL_HASH_MASK;
}


_CPU_AND_GPU_CODE_ inline int pointToVoxelBlockPos(const THREADPTR(Vector3i) & point, THREADPTR(Vector3i) &blockPos) {
	blockPos.x = ((point.x < 0) ? point.x - VOXEL_BLOCK_SIZE + 1 : point.x) / VOXEL_BLOCK_SIZE;
	blockPos.y = ((point.y < 0) ? point.y - VOXEL_BLOCK_SIZE + 1 : point.y) / VOXEL_BLOCK_SIZE;
	blockPos.z = ((point.z < 0) ? point.z - VOXEL_BLOCK_SIZE + 1 : point.z) / VOXEL_BLOCK_SIZE;

	// Vector3i locPos = point - blockPos * VOXEL_BLOCK_SIZE;
	// return locPos.x + locPos.y * VOXEL_BLOCK_SIZE + locPos.z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	// The two comment lines above are arithmetically simplified to:
	return point.x + (point.y - blockPos.x) * VOXEL_BLOCK_SIZE + (point.z - blockPos.y) * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE - blockPos.z * VOXEL_BLOCK_SIZE3;
}

_CPU_AND_GPU_CODE_ inline int pointToVoxelBlockPos(const THREADPTR(Vector3i) & point, THREADPTR(Vector3s) &blockPos) {
	blockPos.x = ((point.x < 0) ? point.x - VOXEL_BLOCK_SIZE + 1 : point.x) / VOXEL_BLOCK_SIZE;
	blockPos.y = ((point.y < 0) ? point.y - VOXEL_BLOCK_SIZE + 1 : point.y) / VOXEL_BLOCK_SIZE;
	blockPos.z = ((point.z < 0) ? point.z - VOXEL_BLOCK_SIZE + 1 : point.z) / VOXEL_BLOCK_SIZE;

	// Vector3i locPos = point - blockPos * VOXEL_BLOCK_SIZE;
	// return locPos.x + locPos.y * VOXEL_BLOCK_SIZE + locPos.z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	// The two comment lines above are arithmetically simplified to:
	return point.x + (point.y - blockPos.x) * VOXEL_BLOCK_SIZE + (point.z - blockPos.y) * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE - blockPos.z * VOXEL_BLOCK_SIZE3;
}

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::VoxelBlockHash::IndexData) *voxelIndex, const THREADPTR(Vector3i) & point,
	THREADPTR(int) &vmIndex, THREADPTR(ITMLib::VoxelBlockHash::IndexCache) & cache)
{
	Vector3i blockPos;
	short linearIdx = pointToVoxelBlockPos(point, blockPos);

	if IS_EQUAL3(blockPos, cache.blockPos)
	{
		vmIndex = true;
		return cache.blockPtr + linearIdx;
	}

	int hashIdx = HashCodeFromBlockPosition(blockPos);

	while (true)
	{
		ITMHashEntry hashEntry = voxelIndex[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0)
		{
			vmIndex = true;
			cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * VOXEL_BLOCK_SIZE3;
			return cache.blockPtr + linearIdx;
		}

		if (hashEntry.offset < 1) break;
		hashIdx = ORDERED_LIST_SIZE + hashEntry.offset - 1;
	}

	vmIndex = false;
	return -1;
}

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::VoxelBlockHash::IndexData) *voxelIndex, Vector3i point, THREADPTR(int) &vmIndex)
{
	ITMLib::VoxelBlockHash::IndexCache cache;
	return findVoxel(voxelIndex, point, vmIndex, cache);
}

_CPU_AND_GPU_CODE_ inline int
findVoxel(const CONSTPTR(ITMLib::VoxelBlockHash::IndexData) *voxelIndex, Vector3i point, THREADPTR(bool) &foundPoint)
{
	int vmIndex;
	ITMLib::VoxelBlockHash::IndexCache cache;
	int result = findVoxel(voxelIndex, point, vmIndex, cache);
	foundPoint = vmIndex != 0;
	return result;
}


template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(ITMLib::VoxelBlockHash::IndexData) *voxelIndex,
	const THREADPTR(Vector3i) & point, THREADPTR(int) &vmIndex, THREADPTR(ITMLib::VoxelBlockHash::IndexCache) & cache)
{
	Vector3i blockPos;
	int linearIdx = pointToVoxelBlockPos(point, blockPos);

	if IS_EQUAL3(blockPos, cache.blockPos)
	{
		vmIndex = true;
		return voxelData[cache.blockPtr + linearIdx];
	}

	int hashIdx = HashCodeFromBlockPosition(blockPos);

	while (true)
	{
		ITMHashEntry hashEntry = voxelIndex[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0)
		{
			cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * VOXEL_BLOCK_SIZE3;
			vmIndex = hashIdx + 1; // add 1 to support legacy true / false operations for isFound

			return voxelData[cache.blockPtr + linearIdx];
		}

		if (hashEntry.offset < 1) break;
		hashIdx = ORDERED_LIST_SIZE + hashEntry.offset - 1;
	}

	vmIndex = false;
	return TVoxel();
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(ITMLib::VoxelBlockHash::IndexData) *voxelIndex,
	Vector3i point, THREADPTR(int) &vmIndex)
{
	ITMLib::VoxelBlockHash::IndexCache cache;
	return readVoxel(voxelData, voxelIndex, point, vmIndex, cache);
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(ITMLib::VoxelBlockHash::IndexData) *voxelIndex,
	Vector3i point, THREADPTR(bool) &foundPoint)
{
	int vmIndex;
	ITMLib::VoxelBlockHash::IndexCache cache;
	TVoxel result = readVoxel(voxelData, voxelIndex, point, vmIndex, cache);
	foundPoint = vmIndex != 0;
	return result;
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex)
{
	TVoxel res = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)), vmIndex);
	return TVoxel::valueToFloat(res.sdf);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache)
{
	TVoxel res = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)), vmIndex, cache);
	return TVoxel::valueToFloat(res.sdf);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache)
{
	float res1, res2, v1, v2;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
		v1 = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
		v2 = voxel.sdf;
	}
	res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
		v1 = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
		v2 = voxel.sdf;
	}
	res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
		v1 = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
		v2 = voxel.sdf;
	}
	res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
		v1 = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
		v2 = voxel.sdf;
	}
	res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	vmIndex = true;
	return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readWithConfidenceFromSDF_float_interpolated(THREADPTR(float) &confidence, const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache)
{
	float res1, res2, v1, v2;
	float res1_c, res2_c, v1_c, v2_c;
	TVoxel voxel;

	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache); v1 = voxel.sdf; v1_c = voxel.w_depth;
	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache); v2 = voxel.sdf; v2_c = voxel.w_depth;
	res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;
	res1_c = (1.0f - coeff.x) * v1_c + coeff.x * v2_c;

	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache); v1 = voxel.sdf; v1_c = voxel.w_depth;
	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache); v2 = voxel.sdf; v2_c = voxel.w_depth;
	res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
	res1_c = (1.0f - coeff.y) * res1_c + coeff.y * ((1.0f - coeff.x) * v1_c + coeff.x * v2_c);

	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache); v1 = voxel.sdf; v1_c = voxel.w_depth;
	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache); v2 = voxel.sdf; v2_c = voxel.w_depth;
	res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;
	res2_c = (1.0f - coeff.x) * v1_c + coeff.x * v2_c;

	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache); v1 = voxel.sdf; v1_c = voxel.w_depth;
	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache); v2 = voxel.sdf; v2_c = voxel.w_depth;
	res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
	res2_c = (1.0f - coeff.y) * res2_c + coeff.y * ((1.0f - coeff.x) * v1_c + coeff.x * v2_c);

	vmIndex = true;

	confidence = (1.0f - coeff.z) * res1_c + coeff.z * res2_c;

	return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}


template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readWithConfidenceFromSDF_float_interpolated_semantic(THREADPTR(float) &confidence, const CONSTPTR(TVoxel) *voxelData,
                 const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache)
{
	float res1, res2, v1, v2;
	float res1_c, res2_c, v1_c, v2_c;
	TVoxel voxel;

	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache); v1 = voxel.sdf;
	v1_c = (voxel.flags == ITMLib::VOXEL_NONTRUNCATED || (voxel.flags == ITMLib::VOXEL_TRUNCATED && v1 > 0));
	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache); v2 = voxel.sdf;
	v2_c = (voxel.flags == ITMLib::VOXEL_NONTRUNCATED || (voxel.flags == ITMLib::VOXEL_TRUNCATED && v2 > 0));
	res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;
	res1_c = (1.0f - coeff.x) * v1_c + coeff.x * v2_c;

	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache); v1 = voxel.sdf;
	v1_c = (voxel.flags == ITMLib::VOXEL_NONTRUNCATED || (voxel.flags == ITMLib::VOXEL_TRUNCATED && v1 > 0));
	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache); v2 = voxel.sdf;
	v2_c = (voxel.flags == ITMLib::VOXEL_NONTRUNCATED || (voxel.flags == ITMLib::VOXEL_TRUNCATED && v2 > 0));
	res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
	res1_c = (1.0f - coeff.y) * res1_c + coeff.y * ((1.0f - coeff.x) * v1_c + coeff.x * v2_c);

	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache); v1 = voxel.sdf;
	v1_c = (voxel.flags == ITMLib::VOXEL_NONTRUNCATED || (voxel.flags == ITMLib::VOXEL_TRUNCATED && v1 > 0));
	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache); v2 = voxel.sdf;
	v2_c = (voxel.flags == ITMLib::VOXEL_NONTRUNCATED || (voxel.flags == ITMLib::VOXEL_TRUNCATED && v2 > 0));
	res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;
	res2_c = (1.0f - coeff.x) * v1_c + coeff.x * v2_c;

	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache); v1 = voxel.sdf;
	v1_c = (voxel.flags == ITMLib::VOXEL_NONTRUNCATED || (voxel.flags == ITMLib::VOXEL_TRUNCATED && v1 > 0));
	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache); v2 = voxel.sdf;
	v2_c = (voxel.flags == ITMLib::VOXEL_NONTRUNCATED || (voxel.flags == ITMLib::VOXEL_TRUNCATED && v2 > 0));
	res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
	res2_c = (1.0f - coeff.y) * res2_c + coeff.y * ((1.0f - coeff.x) * v1_c + coeff.x * v2_c);

	vmIndex = true;

	confidence = (1.0f - coeff.z) * res1_c + coeff.z * res2_c;

	return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache, int & maxW)
{
	float res1, res2, v1, v2;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
		v1 = v.sdf;
		maxW = v.w_depth;
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
		v2 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
		v1 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
		v2 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
		v1 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
		v2 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
		v1 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
		v2 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	vmIndex = true;
	return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline Vector4f readFromSDF_color4u_interpolated(const CONSTPTR(TVoxel) *voxelData,
																	const CONSTPTR(TIndex) *voxelIndex,
																	const THREADPTR(Vector3f) & point,
																	THREADPTR(TCache) & cache)
{
	TVoxel resn; Vector3f ret(0.0f); Vector4f ret4; int vmIndex;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
	ret += (coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
	ret += (1.0f - coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
	ret += (coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
	ret += (coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
	ret += (1.0f - coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
	ret += (coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

	ret4.x = ret.x; ret4.y = ret.y; ret4.z = ret.z; ret4.w = 255.0f;

	return ret4 / 255.0f;
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline Vector4f readFromSDF_color4u_interpolated(const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(TIndex) *voxelIndex, const THREADPTR(Vector3f) & point, THREADPTR(TCache) & cache, int & maxW)
{
	TVoxel resn; Vector3f ret(0.0f); Vector4f ret4; int vmIndex;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
	maxW = resn.w_depth;
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (1.0f - coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (1.0f - coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

	ret4.x = ret.x; ret4.y = ret.y; ret4.z = ret.z; ret4.w = 255.0f;

	return ret4 / 255.0f;
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline Vector3f computeSingleNormalFromSDF(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(TIndex) *voxelIndex, const THREADPTR(Vector3f) &point)
{
	int vmIndex;

	Vector3f ret;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);
	Vector3f ncoeff(1.0f - coeff.x, 1.0f - coeff.y, 1.0f - coeff.z);

	// all 8 values are going to be reused several times
	Vector4f front, back;
	{
	const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex);
	front.x = voxel.sdf;
	}
	{
	const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex);
	front.y = voxel.sdf;
	}
	{
	const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex);
	front.z = voxel.sdf;
	}
	{
	const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex);
	front.w = voxel.sdf;
	}
	{
	const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex);
	back.x = voxel.sdf;
	}
	{
	const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex);
	back.y = voxel.sdf;
	}
	{
	const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex);
	back.z = voxel.sdf;
	}
	{
	const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex);
	back.w = voxel.sdf;
	}

	Vector4f tmp;
	float p1, p2, v1;
	// gradient x
	p1 = front.x * ncoeff.y * ncoeff.z +
		front.z *  coeff.y * ncoeff.z +
		back.x  * ncoeff.y *  coeff.z +
		back.z  *  coeff.y *  coeff.z;
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 0), vmIndex);
		tmp.x = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 0), vmIndex);
		tmp.y = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 1), vmIndex);
		tmp.z = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 1), vmIndex);
		tmp.w = voxel.sdf;
	}
	p2 = tmp.x * ncoeff.y * ncoeff.z +
		tmp.y *  coeff.y * ncoeff.z +
		tmp.z * ncoeff.y *  coeff.z +
		tmp.w *  coeff.y *  coeff.z;
	v1 = p1 * coeff.x + p2 * ncoeff.x;

	p1 = front.y * ncoeff.y * ncoeff.z +
		front.w *  coeff.y * ncoeff.z +
		back.y  * ncoeff.y *  coeff.z +
		back.w  *  coeff.y *  coeff.z;
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 0, 0), vmIndex);
		tmp.x = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 1, 0), vmIndex);
		tmp.y = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 0, 1), vmIndex);
		tmp.z = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 1, 1), vmIndex);
		tmp.w = voxel.sdf;
	}
	p2 = tmp.x * ncoeff.y * ncoeff.z +
		tmp.y *  coeff.y * ncoeff.z +
		tmp.z * ncoeff.y *  coeff.z +
		tmp.w *  coeff.y *  coeff.z;

	ret.x = TVoxel::valueToFloat(p1 * ncoeff.x + p2 * coeff.x - v1);

	// gradient y
	p1 = front.x * ncoeff.x * ncoeff.z +
		front.y *  coeff.x * ncoeff.z +
		back.x  * ncoeff.x *  coeff.z +
		back.y  *  coeff.x *  coeff.z;
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 0), vmIndex);
		tmp.x = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 0), vmIndex);
		tmp.y = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 1), vmIndex);
		tmp.z = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 1), vmIndex);
		tmp.w = voxel.sdf;
	}
	p2 = tmp.x * ncoeff.x * ncoeff.z +
		tmp.y *  coeff.x * ncoeff.z +
		tmp.z * ncoeff.x *  coeff.z +
		tmp.w *  coeff.x *  coeff.z;
	v1 = p1 * coeff.y + p2 * ncoeff.y;

	p1 = front.z * ncoeff.x * ncoeff.z +
		front.w *  coeff.x * ncoeff.z +
		back.z  * ncoeff.x *  coeff.z +
		back.w  *  coeff.x *  coeff.z;
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 2, 0), vmIndex);
		tmp.x = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 2, 0), vmIndex);
		tmp.y = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 2, 1), vmIndex);
		tmp.z = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 2, 1), vmIndex);
		tmp.w = voxel.sdf;
	}
	p2 = tmp.x * ncoeff.x * ncoeff.z +
		tmp.y *  coeff.x * ncoeff.z +
		tmp.z * ncoeff.x *  coeff.z +
		tmp.w *  coeff.x *  coeff.z;

	ret.y = TVoxel::valueToFloat(p1 * ncoeff.y + p2 * coeff.y - v1);

	// gradient z
	p1 = front.x * ncoeff.x * ncoeff.y +
		front.y *  coeff.x * ncoeff.y +
		front.z * ncoeff.x *  coeff.y +
		front.w *  coeff.x *  coeff.y;
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, -1), vmIndex);
		tmp.x = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, -1), vmIndex);
		tmp.y = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, -1), vmIndex);
		tmp.z = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, -1), vmIndex);
		tmp.w = voxel.sdf;
	}
	p2 = tmp.x * ncoeff.x * ncoeff.y +
		tmp.y *  coeff.x * ncoeff.y +
		tmp.z * ncoeff.x *  coeff.y +
		tmp.w *  coeff.x *  coeff.y;
	v1 = p1 * coeff.z + p2 * ncoeff.z;

	p1 = back.x * ncoeff.x * ncoeff.y +
		back.y *  coeff.x * ncoeff.y +
		back.z * ncoeff.x *  coeff.y +
		back.w *  coeff.x *  coeff.y;
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 2), vmIndex);
		tmp.x = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 2), vmIndex);
		tmp.y = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 2), vmIndex);
		tmp.z = voxel.sdf;
	}
	{
		const TVoxel& voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 2), vmIndex);
		tmp.w = voxel.sdf;
	}
	p2 = tmp.x * ncoeff.x * ncoeff.y +
		tmp.y *  coeff.x * ncoeff.y +
		tmp.z * ncoeff.x *  coeff.y +
		tmp.w *  coeff.x *  coeff.y;

	ret.z = TVoxel::valueToFloat(p1 * ncoeff.z + p2 * coeff.z - v1);

	return ret;
}

template<bool hasColor, class TVoxel, class TIndex> struct VoxelColorReader;

template<class TVoxel, class TIndex>
struct VoxelColorReader<false, TVoxel, TIndex> {
	_CPU_AND_GPU_CODE_ static Vector4f interpolate(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
		const THREADPTR(Vector3f) & point)
	{
		return Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
	}
};

template<class TVoxel, class TIndex>
struct VoxelColorReader<true, TVoxel, TIndex> {
	_CPU_AND_GPU_CODE_ static Vector4f interpolate(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
		const THREADPTR(Vector3f) & point)
	{
		typename TIndex::IndexCache cache;
		return readFromSDF_color4u_interpolated(voxelData, voxelIndex, point, cache);
	}
};

#ifndef __METALC__

#include "PlainVoxelArray.h"

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::PlainVoxelArray::IndexData) *voxelIndex, const THREADPTR(Vector3i) & point_orig,
	THREADPTR(int) &vmIndex)
{
	Vector3i point = point_orig - voxelIndex->offset;

	if ((point.x < 0) || (point.x >= voxelIndex->size.x) ||
		(point.y < 0) || (point.y >= voxelIndex->size.y) ||
		(point.z < 0) || (point.z >= voxelIndex->size.z)) {
		vmIndex = false;
		return -1;
	}

	int linearIdx = point.x + point.y * voxelIndex->size.x + point.z * voxelIndex->size.x * voxelIndex->size.y;

	vmIndex = true;
	return linearIdx;
}

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::PlainVoxelArray::IndexData) *voxelIndex, const THREADPTR(Vector3i) & point_orig,
	THREADPTR(int) &vmIndex, THREADPTR(ITMLib::PlainVoxelArray::IndexCache) & cache)
{
	return findVoxel(voxelIndex, point_orig, vmIndex);
}


template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(ITMLib::PlainVoxelArray::IndexData) *voxelIndex,
	const THREADPTR(Vector3i) & point_orig, THREADPTR(int) &vmIndex)
{
	int voxelAddress = findVoxel(voxelIndex, point_orig, vmIndex);
	return vmIndex ? voxelData[voxelAddress] : TVoxel();
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(ITMLib::PlainVoxelArray::IndexData) *voxelIndex,
	const THREADPTR(Vector3i) & point_orig, THREADPTR(int) &vmIndex, THREADPTR(ITMLib::PlainVoxelArray::IndexCache) & cache)
{
	return readVoxel(voxelData, voxelIndex, point_orig, vmIndex);
}

/**
* \brief The specialisations of this struct template can be used to write/read colours to/from surfels.
*
* \tparam hasColour  Whether or not the surfel type can store colour information.
*/
template <bool hasColour> struct SurfelColourManipulator;

/**
* \brief This template specialisation can be used to write/read dummy colours to/from surfels.
*
* It is intended for use with surfel types that cannot store colour information.
*/
template <>
struct SurfelColourManipulator<false>
{
	/**
	* \brief Simulates the reading of a colour from the specified surfel.
	*
	* \param surfel  The surfel.
	* \return        A dummy colour (black).
	*/
	template <typename TSurfel>
	_CPU_AND_GPU_CODE_
		static Vector3u read(const TSurfel& surfel)
	{
		return Vector3u((uchar)0);
	}

	/**
	* \brief Simulates the writing of a colour to the specified surfel.
	*
	* In practice, this is just a no-op, since the surfel can't store a colour.
	*
	* \param surfel  The surfel.
	* \param colour  The colour.
	*/
	template <typename TSurfel>
	_CPU_AND_GPU_CODE_
		static void write(TSurfel& surfel, const Vector3u& colour)
	{
		// No-op
	}
};

/**
* \brief This template specialisation can be used to write/read actual colours to/from surfels.
*
* It is intended for use with surfel types that can store colour information.
*/
template <>
struct SurfelColourManipulator<true>
{
	/**
	* \brief Gets the colour of the specified surfel.
	*
	* \param surfel  The surfel.
	* \return        The surfel's colour.
	*/
	template <typename TSurfel>
	_CPU_AND_GPU_CODE_
		static Vector3u read(const TSurfel& surfel)
	{
		return surfel.colour;
	}

	/**
	* \brief Sets the colour of the specified surfel.
	*
	* \param surfel  The surfel.
	* \param colour  The surfel's new colour.
	*/
	template <typename TSurfel>
	_CPU_AND_GPU_CODE_
		static void write(TSurfel& surfel, const Vector3u& colour)
	{
		surfel.colour = colour;
	}
};

#endif
