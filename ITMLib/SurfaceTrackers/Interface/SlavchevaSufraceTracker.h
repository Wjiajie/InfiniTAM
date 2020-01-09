//  ================================================================
//  Created by Gregory Kramida on 11/13/19.
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

//boost
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>

namespace po = boost::program_options;
namespace pt = boost::property_tree;

namespace ITMLib {

class SlavchevaSurfaceTracker {
public:
#define PARAMETERS_STRUCT_DESCRIPTION Parameters, \
        (float, gradientDescentLearningRate, 0.1f, PRIMITIVE), \
        (float, rigidityEnforcementFactor, 0.1f, PRIMITIVE), \
        (float, weightDataTerm, 1.0f, PRIMITIVE), \
        (float, weightSmoothingTerm, 0.2f, PRIMITIVE), \
        (float, weightLevelSetTerm, 0.2f, PRIMITIVE), \
        (float, epsilon, 1e-5f, PRIMITIVE)

	DECLARE_SERIALIZABLE_STRUCT(PARAMETERS_STRUCT_DESCRIPTION);

#define SWITCHES_STRUCT_DESCRIPTION Switches, \
		(bool, enableDataTerm, true, PRIMITIVE) \
		(bool, enableLevelSetTerm, false, PRIMITIVE) \
		(bool, enableSmoothingTerm, true, PRIMITIVE) \
		(bool, enableKillingRigidityEnforcementTerm, false, PRIMITIVE) \
		(bool, enableSobolevGradientSmoothing, true, PRIMITIVE)

	DECLARE_SERIALIZABLE_STRUCT(SWITCHES_STRUCT_DESCRIPTION);

	const Parameters parameters;
	const Switches switches;

	explicit SlavchevaSurfaceTracker();
	SlavchevaSurfaceTracker(Switches switches, Parameters parameters = Parameters());

private:
	void PrintSettings();
};
}//namespace ITMLib
