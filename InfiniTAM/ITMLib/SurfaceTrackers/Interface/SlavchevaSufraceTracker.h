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

//TODO: move to separate header
class SlavchevaSurfaceTracker {
public:

	enum ConfigurationMode {
		KILLING_FUSION,
		SOBOLEV_FUSION
	};

	struct Parameters {
		Parameters();
		explicit Parameters(ConfigurationMode mode);
		explicit Parameters(const po::variables_map& vm);
		Parameters(float gradientDescentLearningRate,
		           float rigidityEnforcementFactor,
		           float weightDataTerm,
		           float weightSmoothingTerm,
		           float weightLevelSetTerm,
		           float epsilon);
		static Parameters BuildFromPTree(const pt::ptree& tree, ConfigurationMode mode = SOBOLEV_FUSION);
		pt::ptree ToPTree() const;
		friend bool operator== (const Parameters &p1, const Parameters &p2);

		const float gradientDescentLearningRate;// = 0.1f;
		const float rigidityEnforcementFactor;// = 0.1f;
		const float weightDataTerm;// = 1.0f
		const float weightSmoothingTerm;// = 0.2f; //0.2 is default for SobolevFusion, 0.5 is default for KillingFusion
		const float weightLevelSetTerm;// = 0.2f;
		const float epsilon;// = 1e-5f;

	private:

		Parameters(const po::variables_map& vm, ConfigurationMode mode);
	};

	struct Switches {
		Switches();
		explicit Switches(ConfigurationMode mode);
		explicit Switches(const po::variables_map& vm);
		Switches(bool enableDataTerm, bool enableLevelSetTerm, bool enableSmoothingTerm,
		         bool enableKillingRigidityEnforcementTerm, bool enableSobolevGradientSmoothing);
		static Switches BuildFromPTree(const pt::ptree& tree, ConfigurationMode mode = SOBOLEV_FUSION);
		friend bool operator== (const Switches &s1, const Switches &s2);
		friend std::ostream& operator<<(std::ostream& out, const Switches& s);

		pt::ptree ToPTree() const;
		const bool enableDataTerm;
		const bool enableLevelSetTerm;
		const bool enableSmoothingTerm;
		const bool enableKillingRigidityEnforcementTerm;
		const bool enableSobolevGradientSmoothing;
	private:
		Switches(const po::variables_map& vm, ConfigurationMode mode);
	};

	const Parameters parameters;
	const Switches switches;

	explicit SlavchevaSurfaceTracker();
	SlavchevaSurfaceTracker(Switches switches, Parameters parameters = Parameters());

private:
	void PrintSettings();
};

bool operator== (const SlavchevaSurfaceTracker::Parameters &p1, const SlavchevaSurfaceTracker::Parameters &p2);
std::ostream& operator<<(std::ostream& out, const SlavchevaSurfaceTracker::Switches& s);
bool operator== (const SlavchevaSurfaceTracker::Switches &s1, const SlavchevaSurfaceTracker::Switches &s2);
std::ostream& operator<<(std::ostream& out, const SlavchevaSurfaceTracker::Parameters& p);
}//namespace ITMLib
