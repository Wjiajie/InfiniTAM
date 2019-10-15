//  ================================================================
//  Created by Gregory Kramida on 10/15/19.
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
#include "../../../ORUtils/PlatformIndependentAtomics.h"

struct AdditionalGradientAggregates{
	AdditionalGradientAggregates(){
		INITIALIZE_ATOMIC_FLOAT(cumulativeCanonicalSdf, 0.f);
		INITIALIZE_ATOMIC_FLOAT(cumulativeLiveSdf,0.f);
		INITIALIZE_ATOMIC_FLOAT(cumulativeSdfDiff, 0.f);
		INITIALIZE_ATOMIC_FLOAT(cumulativeWarpDist, 0.f);

		INITIALIZE_ATOMIC_INT(consideredVoxelCount, 0);
		INITIALIZE_ATOMIC_INT(dataVoxelCount, 0);
		INITIALIZE_ATOMIC_INT(levelSetVoxelCount, 0);
	}

	DECLARE_ATOMIC_FLOAT(cumulativeCanonicalSdf);
	DECLARE_ATOMIC_FLOAT(cumulativeLiveSdf);
	DECLARE_ATOMIC_FLOAT(cumulativeSdfDiff);
	DECLARE_ATOMIC_FLOAT(cumulativeWarpDist);

	DECLARE_ATOMIC_INT(consideredVoxelCount);
	DECLARE_ATOMIC_INT(dataVoxelCount);
	DECLARE_ATOMIC_INT(levelSetVoxelCount);
};

struct ComponentEnergies{
	ComponentEnergies(){
		INITIALIZE_ATOMIC_FLOAT(totalDataEnergy, 0.f);
		INITIALIZE_ATOMIC_FLOAT(totalLevelSetEnergy,0.f);
		INITIALIZE_ATOMIC_FLOAT(totalTikhonovEnergy,0.f);
		INITIALIZE_ATOMIC_FLOAT(totalRigidityEnergy,0.f);
		INITIALIZE_ATOMIC_FLOAT(totalSmoothnessEnergy,0.f);
	}
	DECLARE_ATOMIC_FLOAT(totalDataEnergy);
	DECLARE_ATOMIC_FLOAT(totalLevelSetEnergy);
	DECLARE_ATOMIC_FLOAT(totalTikhonovEnergy);
	DECLARE_ATOMIC_FLOAT(totalRigidityEnergy);
	DECLARE_ATOMIC_FLOAT(totalSmoothnessEnergy);
};

