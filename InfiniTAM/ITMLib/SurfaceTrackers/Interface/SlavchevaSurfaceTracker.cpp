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

#include <unordered_map>
#include <string>
#include "SurfaceTrackerInterface.h"
#include "../../Utils/Configuration.h"
#include "../../Utils/ITMSceneParameters.h"

using namespace ITMLib;

SlavchevaSurfaceTracker::SlavchevaSurfaceTracker() :
		parameters(Configuration::Instance().slavcheva_parameters),
		switches(Configuration::Instance().slavcheva_switches) {
	PrintSettings();
}

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
	std::cout << "Unity scaling factor: " << this->parameters.unity << std::endl;
	std::cout << bright_cyan << "*** *********************************** ***" << reset << std::endl;
}

//Default preconfigured for Slavecheva's SobolevFusion
SlavchevaSurfaceTracker::Parameters::Parameters() :
		gradientDescentLearningRate(0.1f),
		rigidityEnforcementFactor(0.1f),
		weightDataTerm(1.0f), // for an easier balancing of the terms (will be removed in optimized version)
		weightSmoothingTerm(0.2f), //0.2 is default for SobolevFusion, 0.5 is default for KillingFusion
		weightLevelSetTerm(0.2f),
		epsilon(1e-5f),
		unity(0.1f) // voxelSize/mu, i.e. 1/[narrow-band half-width in voxels] or [voxel size in metric units]/[narrow-band half-width in metric units]
{}

SlavchevaSurfaceTracker::Parameters::Parameters(SlavchevaSurfaceTracker::ConfigurationMode mode, float unity) :
		gradientDescentLearningRate(0.1f),
		rigidityEnforcementFactor(0.1f),
		weightDataTerm(1.0f), // for an easier balancing of the terms (will be removed in optimized version)
		weightSmoothingTerm(mode == KILLING_FUSION ? 0.5f
		                                           : 0.2f), //0.2 is default for SobolevFusion, 0.5 is default for KillingFusion
		weightLevelSetTerm(0.2f),
		epsilon(1e-5f),
		unity(unity) // voxelSize/mu, i.e. 1/[narrow-band half-width in voxels] or [voxel size in metric units]/[narrow-band half-width in metric units]
{}

static SlavchevaSurfaceTracker::ConfigurationMode determine_slavcheva_configuration_mode(const po::variables_map& vm){
	SlavchevaSurfaceTracker::ConfigurationMode slavchevaConfigurationMode = SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION;
	std::unordered_map<std::string, SlavchevaSurfaceTracker::ConfigurationMode> slavchevaConfigurationModeMap(
			{
					{"KillingFusion", SlavchevaSurfaceTracker::ConfigurationMode::KILLING_FUSION},
					{"SobolevFusion", SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION},
			}
	);
	if(!vm["preset_mode"].empty()){
		std::string key = vm["preset_mode"].as<std::string>();
		if(slavchevaConfigurationModeMap.find(key) != slavchevaConfigurationModeMap.end()){
			slavchevaConfigurationMode = slavchevaConfigurationModeMap[key];
		}else{
			std::cerr << "Unrecognized preset_mode: " << key << ". Can be one of: ";
			bool first = true;
			for (auto& pair : slavchevaConfigurationModeMap){
				std::cerr << (first ? "" : ",") << pair.first << std::endl;
				first = false;
			}
		}
	}
	return slavchevaConfigurationMode;
}

SlavchevaSurfaceTracker::Parameters::Parameters(const po::variables_map& vm, ConfigurationMode mode, float unity) :
		gradientDescentLearningRate(
				vm["learning_rate"].empty() ?
				Parameters(mode, unity).gradientDescentLearningRate : vm["learning_rate"].as<float>()),
		rigidityEnforcementFactor(
				vm["rigidity_enforcement_factor"].empty() ?
				Parameters(mode, unity).rigidityEnforcementFactor : vm["rigidity_enforcement_factor"].as<float>()),
		weightDataTerm(
				vm["weight_data_term"].empty() ?
				Parameters(mode, unity).weightDataTerm : vm["weight_data_term"].as<float>()),
		weightSmoothingTerm(
				vm["weight_smoothing_term"].empty() ?
				Parameters(mode, unity).weightSmoothingTerm : vm["weight_smoothing_term"].as<float>()),
		weightLevelSetTerm(
				!vm["weight_level_set_term"].empty() ?
				Parameters(mode, unity).weightLevelSetTerm : vm["weight_level_set_term"].as<float>()),
		epsilon(
				!vm["level_set_epsilon"].empty() ?
				Parameters(mode, unity).epsilon : vm["level_set_epsilon"].as<float>()),
		unity(unity)
{}

SlavchevaSurfaceTracker::Parameters::Parameters(const po::variables_map& vm) :
Parameters(vm, determine_slavcheva_configuration_mode(vm), ITMSceneParameters(vm).voxelSize / ITMSceneParameters(vm).mu )
{}

//Default preconfigured for Slavecheva's SobolevFusion
SlavchevaSurfaceTracker::Switches::Switches() : Switches(SOBOLEV_FUSION) {}

SlavchevaSurfaceTracker::Switches::Switches(SlavchevaSurfaceTracker::ConfigurationMode mode) :
		enableDataTerm(true),
		enableLevelSetTerm(mode == KILLING_FUSION),
		enableSmoothingTerm(true),
		enableKillingRigidityEnforcementTerm(mode == KILLING_FUSION),
		enableSobolevGradientSmoothing(mode == SOBOLEV_FUSION) {}

SlavchevaSurfaceTracker::Switches::Switches(const po::variables_map& vm, ConfigurationMode mode) :
		enableDataTerm(vm["disable_data_term"].empty() ? Switches(mode).enableDataTerm : !vm["disable_data_term"].as<bool>()),
		enableLevelSetTerm(vm["enable_level_set_term"].empty() ? Switches(mode).enableLevelSetTerm : vm["enable_level_set_term"].as<bool>()),
		enableSmoothingTerm(vm["disable_smoothing_term"].empty() ? Switches(mode).enableSmoothingTerm : !vm["disable_smoothing_term"].as<bool>()),
		enableKillingRigidityEnforcementTerm(vm["enable_killing_term"].empty() ?  Switches(mode).enableKillingRigidityEnforcementTerm : vm["enable_killing_term"].as<bool>()),
		enableSobolevGradientSmoothing(vm["disable_gradient_smoothing"].empty() ?  Switches(mode).enableSobolevGradientSmoothing : !vm["disable_gradient_smoothing"].as<bool>())
{}

SlavchevaSurfaceTracker::Switches::Switches(const po::variables_map& vm) : Switches(vm, determine_slavcheva_configuration_mode(vm)){};

