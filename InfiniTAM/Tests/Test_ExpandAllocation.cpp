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
#include "../ITMLib/Engines/Indexing/VBH/CPU/ITMIndexingEngine_CPU_VoxelBlockHash.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Utils/Configuration.h"


using namespace ITMLib;

BOOST_AUTO_TEST_CASE(ExpandVolume_CPU){
	ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>& indexing_engine
		= ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::Instance();

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash> volume(&Configuration::get().scene_parameters,
	                                                   Configuration::get().swapping_mode ==
	                                                   Configuration::SWAPPINGMODE_ENABLED,
	                                                   MEMORYDEVICE_CPU);
	ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash>::Inst().ResetScene(&volume);

	int hash_code = -1;
	Vector3s initial_block_pos(0, 0, 0);
	BOOST_REQUIRE(indexing_engine.AllocateHashBlockAt(&volume, initial_block_pos, hash_code));

	int search_hash_code = -2;
	ITMHashEntry entry = indexing_engine.FindHashEntry(volume.index, initial_block_pos, search_hash_code);
	BOOST_REQUIRE_EQUAL(initial_block_pos, entry.pos);
	BOOST_REQUIRE_EQUAL(hash_code, search_hash_code);

	indexing_engine.ExpandAllocation(&volume);

	for (short zOffset = -1; zOffset < 2; zOffset++) {
		for (short yOffset = -1; yOffset < 2; yOffset++) {
			for (short xOffset = -1; xOffset < 2; xOffset++) {
				Vector3s neighbor_position(xOffset, yOffset, zOffset);
				if (neighbor_position != Vector3s(0, 0, 0)) {
					ITMHashEntry entry = indexing_engine.FindHashEntry(volume.index, neighbor_position, search_hash_code);
					BOOST_REQUIRE_EQUAL(entry.pos, neighbor_position);
					BOOST_REQUIRE(search_hash_code != -1);
				}
			}
		}
	}

}