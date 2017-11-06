// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMMath.h"

/** \brief
    Stores the information of a single voxel in the volume
*/
struct ITMVoxel_f_rgb
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

	static const CONSTPTR(bool) hasColorInformation = true;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasTrilinearWeightInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;

	_CPU_AND_GPU_CODE_ ITMVoxel_f_rgb()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		clr = Vector3u((uchar)0);
		w_color = 0;
	}
};

/** \brief
    Stores the information of a single voxel in the volume
*/
struct ITMVoxel_s_rgb
{
	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short floatToValue(float x) { return (short)((x) * 32767.0f); }

	static const CONSTPTR(bool) hasColorInformation = true;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasTrilinearWeightInformation = false;

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;

	_CPU_AND_GPU_CODE_ ITMVoxel_s_rgb()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		clr = Vector3u((uchar)0);
		w_color = 0;
	}
};

struct ITMVoxel_s
{
	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short floatToValue(float x) { return (short)((x) * 32767.0f); }

	static const CONSTPTR(bool) hasColorInformation = false;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasTrilinearWeightInformation = false;

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;

	_CPU_AND_GPU_CODE_ ITMVoxel_s()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
	}
};

struct ITMVoxel_f
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

	static const CONSTPTR(bool) hasColorInformation = false;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasTrilinearWeightInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;

	_CPU_AND_GPU_CODE_ ITMVoxel_f()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
	}
};

struct ITMVoxel_f_conf
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

	static const CONSTPTR(bool) hasColorInformation = false;
	static const CONSTPTR(bool) hasConfidenceInformation = true;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasTrilinearWeightInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	float confidence;

	_CPU_AND_GPU_CODE_ ITMVoxel_f_conf()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		confidence = 0.0f;
	}
};


struct ITMVoxel_s_rgb_conf
{
	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short floatToValue(float x) { return (short)((x) * 32767.0f); }

	static const CONSTPTR(bool) hasColorInformation = true;
	static const CONSTPTR(bool) hasConfidenceInformation = true;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasTrilinearWeightInformation = false;

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;
	float confidence;

	_CPU_AND_GPU_CODE_ ITMVoxel_s_rgb_conf()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		confidence = 0.0f;
		clr = Vector3u((uchar)0);
		w_color = 0;
	}
};

struct ITMVoxel_f_rgb_conf
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

	static const CONSTPTR(bool) hasColorInformation = true;
	static const CONSTPTR(bool) hasConfidenceInformation = true;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasTrilinearWeightInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;
	float confidence;

	_CPU_AND_GPU_CODE_ ITMVoxel_f_rgb_conf()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		confidence = 0.0f;
		clr = Vector3u((uchar)0);
		w_color = 0;
	}
};

struct ITMVoxel_f_dynamic
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

	static const CONSTPTR(bool) hasColorInformation = true;
	static const CONSTPTR(bool) hasConfidenceInformation = true;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasTrilinearWeightInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Trilinear weight information stored for this voxel. //TODO: remove, with the check --NOT REALLY NEEDED, can be computed on the fly -Greg (GitHub:Algomorph) */
	//Vector3u trilienar_weights;
	/** Number of observations that made up @p clr. */
	uchar w_color;
	float confidence;
	/** vector translating the current point to a different location **/
	Vector3f warp_t;
	/** used for intermediate update results **/
	Vector3s warp_t_update;

	_CPU_AND_GPU_CODE_ ITMVoxel_f_dynamic()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		confidence = 0.0f;
		clr = Vector3u((uchar)0);
		//trilienar_weights = Vector3u((uchar)0);
		w_color = 0;
		warp_t = Vector3f(0.f);
		warp_t_update = Vector3s((short)0);
	}
};
