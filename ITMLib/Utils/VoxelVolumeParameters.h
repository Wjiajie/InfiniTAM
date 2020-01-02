// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

//boost
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
namespace po = boost::program_options;
namespace pt = boost::property_tree;

namespace ITMLib
{
	/** \brief
	    Stores parameters of a scene like voxel size
	*/
	class VoxelVolumeParameters
	{
	public:
		/// Size of a voxel, usually given in meters.
		const float voxel_size;

		/** @{ */
		/** \brief
		    Fallback parameters: consider only parts of the
		    scene from @p near_clipping_distance in front of the camera
		    to a distance of @p far_clipping_distance. Usually the
		    actual depth range should be determined
		    automatically by a ITMLib::Engine::ITMVisualisationEngine.
		*/
		const float near_clipping_distance, far_clipping_distance;

		/** @} */
		/** \brief
		    Encodes the width of the band of the truncated
		    signed distance transform that is actually stored
		    in the volume. This is again usually specified in
		    meters. The resulting width in voxels is @ref narrow_band_half_width
		    divided by @ref voxel_size.
		*/
		const float narrow_band_half_width;

		/** \brief
		    Up to @ref max_integration_weight observations per voxel are averaged.
		    Beyond that a sliding average is computed.
		*/
		const int max_integration_weight;

		/** Stop integration/fusion once max_integration_weight has been reached. */
		const bool stop_integration_at_max_weight;

		VoxelVolumeParameters();
		explicit VoxelVolumeParameters(const po::variables_map& vm);
		static VoxelVolumeParameters BuildFromPTree(const pt::ptree& tree);
		pt::ptree ToPTree() const;

		friend bool operator== (const VoxelVolumeParameters &p1, const VoxelVolumeParameters &p2);
		friend std::ostream& operator<<(std::ostream& out, const VoxelVolumeParameters& parameters);

		/**
		 * \brief standard constructor setting all elements to passed-in values
		 * \param narrow_band_half_width width of the band (in meters) of the truncated signed distance transform
		 * \param max_integration_weight maximum number of observations per voxel which are averaged, after this a sliding average is computed
		 * \param voxel_size voxel_size (in meters)
		 * \param near_clipping_distance distance (in meters) to near clipping plane of the view frustum, closer than which nothing is considered
		 * \param far_clipping_distance distance (in meters) to far clipping plane of the view frustum, farther than which nothing is considered
		 * \param stop_integration_at_max_weight defines behavior after max_integration_weight observations have been gathered for a specific point
		 */
		VoxelVolumeParameters(float narrow_band_half_width, int max_integration_weight, float voxel_size,
		                      float near_clipping_distance, float far_clipping_distance, bool stop_integration_at_max_weight);





	};

	bool operator==(const VoxelVolumeParameters &p1, const VoxelVolumeParameters &p2);
	std::ostream& operator<<(std::ostream& out, const VoxelVolumeParameters& parameters);
}
