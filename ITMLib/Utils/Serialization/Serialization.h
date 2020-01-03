//  ================================================================
//  Created by Gregory Kramida on 1/2/20.
//  Copyright (c) 2020 Gregory Kramida
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
#include "SerializationDetails.h"
/// Macros for making serializable stucts and enums

/**
 * \brief Generate declaration and definition of a  struct that has utilities to be generated from program arguments or
 * a .json tree
 * \details default constructor, constructor with all members in order, constructor based
 * on boost::program_options::variables_map, BuildFromPropertyTree static member function that returns an instance,
 * and a ToPtree static member function, that returns a boost::property_tree::ptree holding all the data.
 *
 * Usage:
 * @ref GENERATE_SERIALIZABLE_STRUCT( MyWonderfulUnicorn, (float, age, 5.0f), (float, horn_length, 0.33f) )
 * Generates a struct called "MyWonderfulUnicorn" with two fields, i.e. public member variables, called
 * "age" and "horn_length". Both are typed as floats and have their respective default values set at 5.0f and 0.33f.
 */
#define GENERATE_SERIALIZABLE_STRUCT( struct_name, ... )              \
	SERIALIZABLE_STRUCT_IMPL( struct_name, __VA_ARGS__)

