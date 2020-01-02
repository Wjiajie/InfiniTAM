//  ================================================================
//  Created by Gregory Kramida on 11/8/19.
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

//stdlib
#include <unordered_map>
#include <string>

//local
#include "SlavchevaSufraceTracker.h"
#include "../../Utils/Configuration.h"
#include "../../Utils/json_utils.h"

using namespace ITMLib;

SlavchevaSurfaceTracker::SlavchevaSurfaceTracker() :
		parameters(Configuration::get().slavcheva_parameters),
		switches(Configuration::get().slavcheva_switches) {
	PrintSettings();
}

SlavchevaSurfaceTracker::SlavchevaSurfaceTracker(Switches switches, Parameters parameters) :
		parameters(parameters), switches(switches) {}

void SlavchevaSurfaceTracker::PrintSettings() {
	std::cout << bright_cyan << "*** Scene Motion Tracker Settings: ***" << reset << std::endl;
	std::cout << "Data term enabled: " << printBool(this->switches.enableDataTerm) << std::endl;
	std::cout << "Smoothing term enabled: " << printBool(this->switches.enableSmoothingTerm) << std::endl;
	std::cout << "Level Set term enabled: " << printBool(this->switches.enableLevelSetTerm) << std::endl;
	std::cout << "Killing term enabled: " << printBool(this->switches.enableKillingRigidityEnforcementTerm)
	          << std::endl;
	std::cout << "Gradient smoothing enabled: " << printBool(this->switches.enableSobolevGradientSmoothing) << std::endl
	          << std::endl;

	std::cout << "Gradient descent learning rate: " << this->parameters.gradientDescentLearningRate << std::endl;
	std::cout << "Rigidity enforcement factor: " << this->parameters.rigidityEnforcementFactor << std::endl;
	std::cout << "Weight of the data term: " << this->parameters.weightDataTerm << std::endl;
	std::cout << "Weight of the smoothness term: " << this->parameters.weightSmoothingTerm << std::endl;
	std::cout << "Weight of the level set term: " << this->parameters.weightLevelSetTerm << std::endl;
	std::cout << "Epsilon for the level set term: " << this->parameters.epsilon << std::endl;
	std::cout << bright_cyan << "*** *********************************** ***" << reset << std::endl;
}

//Default preconfigured for Slavecheva's SobolevFusion
SlavchevaSurfaceTracker::Parameters::Parameters() :
		gradientDescentLearningRate(0.1f),
		rigidityEnforcementFactor(0.1f),
		weightDataTerm(1.0f), // for an easier balancing of the terms (will be removed in optimized version)
		weightSmoothingTerm(0.2f), //0.2 is default for SobolevFusion, 0.5 is default for KillingFusion
		weightLevelSetTerm(0.2f),
		epsilon(1e-5f) {}

SlavchevaSurfaceTracker::Parameters::Parameters(SlavchevaSurfaceTracker::ConfigurationMode mode) :
		gradientDescentLearningRate(0.1f),
		rigidityEnforcementFactor(0.1f),
		weightDataTerm(1.0f), // for an easier balancing of the terms (will be removed in optimized version)
		weightSmoothingTerm(mode == KILLING_FUSION ? 0.5f
		                                           : 0.2f), //0.2 is default for SobolevFusion, 0.5 is default for KillingFusion
		weightLevelSetTerm(0.2f),
		epsilon(1e-5f) {}

static SlavchevaSurfaceTracker::ConfigurationMode determine_slavcheva_configuration_mode(const po::variables_map& vm) {
	SlavchevaSurfaceTracker::ConfigurationMode slavchevaConfigurationMode = SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION;
	std::unordered_map<std::string, SlavchevaSurfaceTracker::ConfigurationMode> slavchevaConfigurationModeMap(
			{
					{"KillingFusion", SlavchevaSurfaceTracker::ConfigurationMode::KILLING_FUSION},
					{"SobolevFusion", SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION},
			}
	);
	if (!vm["slavcheva_preset"].empty()) {
		std::string key = vm["slavcheva_preset"].as<std::string>();
		if (slavchevaConfigurationModeMap.find(key) != slavchevaConfigurationModeMap.end()) {
			slavchevaConfigurationMode = slavchevaConfigurationModeMap[key];
		} else {
			std::cerr << "Unrecognized slavcheva_preset: " << key << ". Can be one of: ";
			bool first = true;
			for (auto& pair : slavchevaConfigurationModeMap) {
				std::cerr << (first ? "" : ",") << pair.first << std::endl;
				first = false;
			}
		}
	}
	return slavchevaConfigurationMode;
}

SlavchevaSurfaceTracker::Parameters::Parameters(const po::variables_map& vm, ConfigurationMode mode) :
		gradientDescentLearningRate(
				vm["learning_rate"].empty() ?
				Parameters(mode).gradientDescentLearningRate : vm["learning_rate"].as<float>()),
		rigidityEnforcementFactor(
				vm["rigidity_enforcement_factor"].empty() ?
				Parameters(mode).rigidityEnforcementFactor : vm["rigidity_enforcement_factor"].as<float>()),
		weightDataTerm(
				vm["weight_data_term"].empty() ?
				Parameters(mode).weightDataTerm : vm["weight_data_term"].as<float>()),
		weightSmoothingTerm(
				vm["weight_smoothing_term"].empty() ?
				Parameters(mode).weightSmoothingTerm : vm["weight_smoothing_term"].as<float>()),
		weightLevelSetTerm(
				vm["weight_level_set_term"].empty() ?
				Parameters(mode).weightLevelSetTerm : vm["weight_level_set_term"].as<float>()),
		epsilon(
				vm["level_set_epsilon"].empty() ?
				Parameters(mode).epsilon : vm["level_set_epsilon"].as<float>()) {}

SlavchevaSurfaceTracker::Parameters::Parameters(const po::variables_map& vm) :
		Parameters(vm, determine_slavcheva_configuration_mode(vm)) {}

SlavchevaSurfaceTracker::Parameters
SlavchevaSurfaceTracker::Parameters::BuildFromPTree(const pt::ptree& tree, ConfigurationMode mode) {
	boost::optional<float> gradientDescentLearningRate = tree.get_optional<float>("learning_rate");
	boost::optional<float> rigidityEnforcementFactor = tree.get_optional<float>("rigidity_enforcement_factor");
	boost::optional<float> weightDataTerm = tree.get_optional<float>("weight_data_term");
	boost::optional<float> weightSmoothingTerm = tree.get_optional<float>("weight_smoothing_term");
	boost::optional<float> weightLevelSetTerm = tree.get_optional<float>("weight_level_set_term");
	boost::optional<float> epsilon = tree.get_optional<float>("level_set_epsilon");

	SlavchevaSurfaceTracker::Parameters default_parameters(mode);

	return {gradientDescentLearningRate ? gradientDescentLearningRate.get()
	                                    : default_parameters.gradientDescentLearningRate,
	        rigidityEnforcementFactor ? rigidityEnforcementFactor.get() : default_parameters.rigidityEnforcementFactor,
	        weightDataTerm ? weightDataTerm.get() : default_parameters.weightDataTerm,
	        weightSmoothingTerm ? weightSmoothingTerm.get() : default_parameters.weightSmoothingTerm,
	        weightLevelSetTerm ? weightLevelSetTerm.get() : default_parameters.weightLevelSetTerm,
	        epsilon ? epsilon.get() : default_parameters.epsilon};
}

SlavchevaSurfaceTracker::Parameters::Parameters(float gradientDescentLearningRate,
                                                float rigidityEnforcementFactor,
                                                float weightDataTerm,
                                                float weightSmoothingTerm,
                                                float weightLevelSetTerm,
                                                float epsilon) :
		gradientDescentLearningRate(gradientDescentLearningRate),
		rigidityEnforcementFactor(rigidityEnforcementFactor),
		weightDataTerm(weightDataTerm),
		weightSmoothingTerm(weightSmoothingTerm),
		weightLevelSetTerm(weightLevelSetTerm),
		epsilon(epsilon) {}

pt::ptree SlavchevaSurfaceTracker::Parameters::ToPTree() const {
	pt::ptree tree;
	tree.add("learning_rate", gradientDescentLearningRate);
	tree.add("rigidity_enforcement_factor", rigidityEnforcementFactor);
	tree.add("weight_data_term", weightDataTerm);
	tree.add("weight_smoothing_term", weightSmoothingTerm);
	tree.add("weight_level_set_term", weightLevelSetTerm);
	tree.add("level_set_epsilon", epsilon);
	return tree;
}

//Default preconfigured for Slavecheva's SobolevFusion
SlavchevaSurfaceTracker::Switches::Switches() : Switches(SOBOLEV_FUSION) {}

SlavchevaSurfaceTracker::Switches::Switches(SlavchevaSurfaceTracker::ConfigurationMode mode) :
		enableDataTerm(true),
		enableLevelSetTerm(mode == KILLING_FUSION),
		enableSmoothingTerm(true),
		enableKillingRigidityEnforcementTerm(mode == KILLING_FUSION),
		enableSobolevGradientSmoothing(mode == SOBOLEV_FUSION) {}

SlavchevaSurfaceTracker::Switches::Switches(const po::variables_map& vm, ConfigurationMode mode) :
		enableDataTerm(
				vm["disable_data_term"].empty() ? Switches(mode).enableDataTerm : !vm["disable_data_term"].as<bool>()),
		enableLevelSetTerm(vm["enable_level_set_term"].empty() ? Switches(mode).enableLevelSetTerm
		                                                       : vm["enable_level_set_term"].as<bool>()),
		enableSmoothingTerm(vm["disable_smoothing_term"].empty() ? Switches(mode).enableSmoothingTerm
		                                                         : !vm["disable_smoothing_term"].as<bool>()),
		enableKillingRigidityEnforcementTerm(
				vm["enable_killing_term"].empty() ? Switches(mode).enableKillingRigidityEnforcementTerm
				                                  : vm["enable_killing_term"].as<bool>()),
		enableSobolevGradientSmoothing(
				vm["disable_gradient_smoothing"].empty() ? Switches(mode).enableSobolevGradientSmoothing
				                                         : !vm["disable_gradient_smoothing"].as<bool>()) {}

SlavchevaSurfaceTracker::Switches::Switches(const po::variables_map& vm) : Switches(vm,
                                                                                    determine_slavcheva_configuration_mode(
		                                                                                    vm)) {}

SlavchevaSurfaceTracker::Switches::Switches(bool enableDataTerm, bool enableLevelSetTerm, bool enableSmoothingTerm,
                                            bool enableKillingRigidityEnforcementTerm,
                                            bool enableSobolevGradientSmoothing) :
		enableDataTerm(enableDataTerm),
		enableLevelSetTerm(enableLevelSetTerm),
		enableSmoothingTerm(enableSmoothingTerm),
		enableKillingRigidityEnforcementTerm(enableKillingRigidityEnforcementTerm),
		enableSobolevGradientSmoothing(enableSobolevGradientSmoothing) {}

SlavchevaSurfaceTracker::Switches
SlavchevaSurfaceTracker::Switches::BuildFromPTree(const pt::ptree& tree, ConfigurationMode mode) {
	boost::optional<bool> disableDataTerm = tree.get_optional<bool>("disable_data_term");
	boost::optional<bool> enableLevelSetTerm = tree.get_optional<bool>("enable_level_set_term");
	boost::optional<bool> disableSmoothingTerm = tree.get_optional<bool>("disable_smoothing_term");
	boost::optional<bool> enableKillingRigidityEnforcementTerm = tree.get_optional<bool>("enable_killing_term");
	boost::optional<bool> disableSobolevGradientSmoothing = tree.get_optional<bool>("disable_gradient_smoothing");

	SlavchevaSurfaceTracker::Switches default_switches(mode);

	return {disableDataTerm ? !disableDataTerm.get() : default_switches.enableDataTerm,
	        enableLevelSetTerm ? enableLevelSetTerm.get() : default_switches.enableLevelSetTerm,
	        disableSmoothingTerm ? !disableSmoothingTerm.get() : default_switches.enableSmoothingTerm,
	        enableKillingRigidityEnforcementTerm ? enableKillingRigidityEnforcementTerm.get()
	                                             : default_switches.enableKillingRigidityEnforcementTerm,
	        disableSobolevGradientSmoothing ? !disableSobolevGradientSmoothing.get()
	                                        : default_switches.enableSobolevGradientSmoothing};
}

pt::ptree SlavchevaSurfaceTracker::Switches::ToPTree() const {
	pt::ptree tree;
	tree.add("disable_data_term", !enableDataTerm);
	tree.add("enable_level_set_term", enableLevelSetTerm);
	tree.add("disable_smoothing_term", !enableSmoothingTerm);
	tree.add("enable_killing_term", enableKillingRigidityEnforcementTerm);
	tree.add("disable_gradient_smoothing", !enableSobolevGradientSmoothing);
	return tree;
};

namespace ITMLib{
bool operator==(const SlavchevaSurfaceTracker::Parameters& p1, const SlavchevaSurfaceTracker::Parameters& p2) {
	return p1.gradientDescentLearningRate == p2.gradientDescentLearningRate &&
	       p1.rigidityEnforcementFactor == p2.rigidityEnforcementFactor &&
	       p1.weightDataTerm == p2.weightDataTerm &&
	       p1.weightSmoothingTerm == p2.weightSmoothingTerm &&
	       p1.weightLevelSetTerm == p2.weightLevelSetTerm &&
	       p1.epsilon == p2.epsilon;
}
std::ostream& operator<<(std::ostream& out, const SlavchevaSurfaceTracker::Switches& s){
	pt::ptree tree(s.ToPTree());
	pt::write_json_no_quotes(out, tree, true);
	return out;
}

bool operator==(const SlavchevaSurfaceTracker::Switches& s1, const SlavchevaSurfaceTracker::Switches& s2) {
	return s1.enableDataTerm == s2.enableDataTerm &&
	       s1.enableLevelSetTerm == s2.enableLevelSetTerm &&
	       s1.enableSmoothingTerm == s2.enableSmoothingTerm &&
	       s1.enableKillingRigidityEnforcementTerm == s2.enableKillingRigidityEnforcementTerm &&
	       s1.enableSobolevGradientSmoothing == s2.enableSobolevGradientSmoothing;
}
std::ostream& operator<<(std::ostream& out, const SlavchevaSurfaceTracker::Parameters& p){
	pt::ptree tree(p.ToPTree());
	pt::write_json_no_quotes(out, tree, true);
	return out;
}
}//namespace ITMLib