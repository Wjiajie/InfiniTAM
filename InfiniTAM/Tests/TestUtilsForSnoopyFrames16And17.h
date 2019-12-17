//  ================================================================
//  Created by Gregory Kramida on 12/1/19.
//  Copyright (c)  2019 Gregory Kramida
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

#include "../ITMLib/Objects/Scene/ITMPlainVoxelArray.h"
#include "../ITMLib/Objects/Scene/ITMVoxelBlockHash.h"

using namespace ITMLib;

struct Frame16And17Fixture {
	template<typename TIndex>
	static typename TIndex::InitializationParameters InitParams();
};

template<>
ITMPlainVoxelArray::InitializationParameters Frame16And17Fixture::InitParams<ITMPlainVoxelArray>() {
	return {Vector3i(80, 96, 248), Vector3i(-64, -24, 64)};
}

template<>
ITMVoxelBlockHash::InitializationParameters Frame16And17Fixture::InitParams<ITMVoxelBlockHash>() {
	return {1800, 0x20000};
}