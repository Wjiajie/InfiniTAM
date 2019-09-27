//  ================================================================
//  Created by Gregory Kramida on 9/27/19.
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

#include "../../ORUtils/PlatformIndependence.h"
#include "ITMMath.h"

_CPU_AND_GPU_CODE_
inline
bool isPointInBounds(const Vector3i& point, const Vector6i& bounds){
	return point.x >= bounds.min_x &&
	       point.y >= bounds.min_y &&
	       point.z >= bounds.min_z &&
	       point.x < bounds.max_x &&
	       point.y < bounds.max_y &&
	       point.z < bounds.max_z;
}

