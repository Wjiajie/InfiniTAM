//  ================================================================
//  Created by Gregory Kramida on 12/10/19.
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
#define BOOST_TEST_MODULE ExpandAllocation
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>
#include <iostream>

//local
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Utils/Configuration.h"
//local - CPU
#include "../ITMLib/Engines/Indexing/VBH/CPU/ITMIndexingEngine_CPU_VoxelBlockHash.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
//local - CUDA
#ifndef COMPLIE_WITHOUT_CUDA
#include "../ITMLib/Engines/Indexing/VBH/CUDA/ITMIndexingEngine_CUDA_VoxelBlockHash.h"
#include "../ITMLib/Engines/Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"
#include "CUDAAtomicTesting.h"
#endif


using namespace ITMLib;

BOOST_AUTO_TEST_CASE(ExpandVolume_CPU) {
	ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>& indexing_engine
			= ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::Instance();

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> volume1(&Configuration::get().scene_parameters,
	                                                    Configuration::get().swapping_mode ==
	                                                    Configuration::SWAPPINGMODE_ENABLED,
	                                                    MEMORYDEVICE_CPU,
	                                                    {1200, 0x20000});
	ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash>::Inst().ResetScene(&volume1);

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> volume2(&Configuration::get().scene_parameters,
	                                                    Configuration::get().swapping_mode ==
	                                                    Configuration::SWAPPINGMODE_ENABLED,
	                                                    MEMORYDEVICE_CPU,
	                                                    {1200, 0x20000});
	ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash>::Inst().ResetScene(&volume2);

	int hash_code = -1;
	Vector3s initial_block_pos(0, 0, 0);
	BOOST_REQUIRE(indexing_engine.AllocateHashBlockAt(&volume1, initial_block_pos, hash_code));

	int search_hash_code = -2;
	ITMHashEntry entry = indexing_engine.FindHashEntry(volume1.index, initial_block_pos, search_hash_code);
	BOOST_REQUIRE_EQUAL(initial_block_pos, entry.pos);
	BOOST_REQUIRE_EQUAL(hash_code, search_hash_code);

	indexing_engine.AllocateUsingOtherVolumeExpanded(&volume2, &volume1);

	for (short zOffset = -1; zOffset < 2; zOffset++) {
		for (short yOffset = -1; yOffset < 2; yOffset++) {
			for (short xOffset = -1; xOffset < 2; xOffset++) {
				Vector3s neighbor_position(xOffset, yOffset, zOffset);
				ITMHashEntry entry = indexing_engine.FindHashEntry(volume2.index, neighbor_position, search_hash_code);
				BOOST_REQUIRE_EQUAL(entry.pos, neighbor_position);
				BOOST_REQUIRE(search_hash_code != -1);
			}
		}
	}

	for (short z = -2; z < 3; z++) {
		for (short y = -2; y < 3; y++) {
			for (short x = -2; x < 3; x++) {
				if (x < -1 || x > 1 || y < -1 || y > 1 || z < -1 || z > 1) {
					Vector3s neighbor_position(x, y, z);
					ITMHashEntry entry = indexing_engine.FindHashEntry(volume2.index, neighbor_position,
					                                                   search_hash_code);
					BOOST_REQUIRE_EQUAL(search_hash_code, -1);
				}
			}
		}
	}

	// swap roles of volume1 and volume2, volume2 now becomes the source.
	Vector3s new_block_pos(2, 2, 2);
	BOOST_REQUIRE(indexing_engine.AllocateHashBlockAt(&volume2, new_block_pos, hash_code));

	indexing_engine.AllocateUsingOtherVolumeExpanded(&volume1, &volume2);

	for (short zOffset = -2; zOffset < 3; zOffset++) {
		for (short yOffset = -2; yOffset < 3; yOffset++) {
			for (short xOffset = -2; xOffset < 3; xOffset++) {
				Vector3s neighbor_position(xOffset, yOffset, zOffset);
				ITMHashEntry entry = indexing_engine.FindHashEntry(volume1.index, neighbor_position, search_hash_code);
				BOOST_REQUIRE_EQUAL(entry.pos, neighbor_position);
				BOOST_REQUIRE(search_hash_code != -1);
			}
		}
	}

	for (short zOffset = 2; zOffset < 4; zOffset++) {
		for (short yOffset = 2; yOffset < 4; yOffset++) {
			for (short xOffset = 2; xOffset < 4; xOffset++) {
				Vector3s neighbor_position(xOffset, yOffset, zOffset);
				ITMHashEntry entry = indexing_engine.FindHashEntry(volume1.index, neighbor_position, search_hash_code);
				BOOST_REQUIRE_EQUAL(entry.pos, neighbor_position);
				BOOST_REQUIRE(search_hash_code != -1);
			}
		}
	}

	for (short z = -2; z < 3; z++) {
		for (short y = -2; y < 3; y++) {
			for (short x = -2; x < 3; x++) {
				if (x < -1 || x > 3 || y < -1 || y > 3 || z < -1 || z > 3) {
					Vector3s neighbor_position(x, y, z);
					ITMHashEntry entry = indexing_engine.FindHashEntry(volume2.index, neighbor_position,
					                                                   search_hash_code);
					BOOST_REQUIRE_EQUAL(search_hash_code, -1);
				}
			}
		}
	}

	Vector3s test_position(0,2,2);
	entry = indexing_engine.FindHashEntry(volume2.index, test_position, search_hash_code);
	BOOST_REQUIRE_EQUAL(search_hash_code, -1);
}

#ifndef COMPILE_WITHOUT_CUDA

BOOST_AUTO_TEST_CASE(ExpandVolume_CUDA) {
	ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CUDA>& indexing_engine
			= ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CUDA>::Instance();

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> volume1(&Configuration::get().scene_parameters,
	                                                    Configuration::get().swapping_mode ==
	                                                    Configuration::SWAPPINGMODE_ENABLED,
	                                                    MEMORYDEVICE_CUDA,
	                                                    {1200, 0x20000});
	ITMSceneManipulationEngine_CUDA<ITMVoxel, ITMVoxelBlockHash>::Inst().ResetScene(&volume1);

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> volume2(&Configuration::get().scene_parameters,
	                                                    Configuration::get().swapping_mode ==
	                                                    Configuration::SWAPPINGMODE_ENABLED,
	                                                    MEMORYDEVICE_CUDA,
	                                                    {1200, 0x20000});
	ITMSceneManipulationEngine_CUDA<ITMVoxel, ITMVoxelBlockHash>::Inst().ResetScene(&volume2);

	int hash_code = -1;
	Vector3s initial_block_pos(0, 0, 0);
	BOOST_REQUIRE(indexing_engine.AllocateHashBlockAt(&volume1, initial_block_pos, hash_code));

	int search_hash_code = -2;
	ITMHashEntry entry = indexing_engine.FindHashEntry(volume1.index, initial_block_pos, search_hash_code);
	BOOST_REQUIRE_EQUAL(initial_block_pos, entry.pos);
	BOOST_REQUIRE_EQUAL(hash_code, search_hash_code);

	indexing_engine.AllocateUsingOtherVolumeExpanded(&volume2, &volume1);

	for (short zOffset = -1; zOffset < 2; zOffset++) {
		for (short yOffset = -1; yOffset < 2; yOffset++) {
			for (short xOffset = -1; xOffset < 2; xOffset++) {
				Vector3s neighbor_position(xOffset, yOffset, zOffset);
				ITMHashEntry entry = indexing_engine.FindHashEntry(volume2.index, neighbor_position, search_hash_code);
				BOOST_REQUIRE(search_hash_code != -1);
				BOOST_REQUIRE_EQUAL(entry.pos, neighbor_position);
			}
		}
	}

	for (short z = -2; z < 3; z++) {
		for (short y = -2; y < 3; y++) {
			for (short x = -2; x < 3; x++) {
				if (x < -1 || x > 1 || y < -1 || y > 1 || z < -1 || z > 1) {
					Vector3s neighbor_position(x, y, z);
					ITMHashEntry entry = indexing_engine.FindHashEntry(volume2.index, neighbor_position,
					                                                   search_hash_code);
					BOOST_REQUIRE_EQUAL(search_hash_code, -1);
				}
			}
		}
	}

	// swap roles of volume1 and volume2, volume2 now becomes the source.
	Vector3s new_block_pos(2, 2, 2);
	BOOST_REQUIRE(indexing_engine.AllocateHashBlockAt(&volume2, new_block_pos, hash_code));

	indexing_engine.AllocateUsingOtherVolumeExpanded(&volume1, &volume2);

	for (short zOffset = -2; zOffset < 3; zOffset++) {
		for (short yOffset = -2; yOffset < 3; yOffset++) {
			for (short xOffset = -2; xOffset < 3; xOffset++) {
				Vector3s neighbor_position(xOffset, yOffset, zOffset);
				ITMHashEntry entry = indexing_engine.FindHashEntry(volume1.index, neighbor_position, search_hash_code);
				BOOST_REQUIRE_EQUAL(entry.pos, neighbor_position);
				BOOST_REQUIRE(search_hash_code != -1);
			}
		}
	}

	for (short zOffset = 2; zOffset < 4; zOffset++) {
		for (short yOffset = 2; yOffset < 4; yOffset++) {
			for (short xOffset = 2; xOffset < 4; xOffset++) {
				Vector3s neighbor_position(xOffset, yOffset, zOffset);
				ITMHashEntry entry = indexing_engine.FindHashEntry(volume1.index, neighbor_position, search_hash_code);
				BOOST_REQUIRE_EQUAL(entry.pos, neighbor_position);
				BOOST_REQUIRE(search_hash_code != -1);
			}
		}
	}

	for (short z = -2; z < 3; z++) {
		for (short y = -2; y < 3; y++) {
			for (short x = -2; x < 3; x++) {
				if (x < -1 || x > 3 || y < -1 || y > 3 || z < -1 || z > 3) {
					Vector3s neighbor_position(x, y, z);
					ITMHashEntry entry = indexing_engine.FindHashEntry(volume2.index, neighbor_position,
					                                                   search_hash_code);
					BOOST_REQUIRE_EQUAL(search_hash_code, -1);
				}
			}
		}
	}

	Vector3s test_position(0,2,2);
	entry = indexing_engine.FindHashEntry(volume2.index, test_position, search_hash_code);
	BOOST_REQUIRE_EQUAL(search_hash_code, -1);
}

#endif