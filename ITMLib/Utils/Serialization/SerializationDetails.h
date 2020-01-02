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

//boost
#include <boost/program_options/variables_map.hpp>
#include <boost/property_tree/ptree.hpp>

// local
#include "SerializationSequenceMacros.h"
#include "../json_utils.h"


// region ==== LOW-LEVEL MACROS
#define SERIALIZABLE_STRUCT_IMPL_SEMICOLON() ;
#define SERIALIZABLE_STRUCT_IMPL_COMMA() ,
#define SERIALIZABLE_STRUCT_IMPL_AND() &&
#define SERIALIZABLE_STRUCT_IMPL_NOTHING()

#define SERIALIZABLE_STRUCT_IMPL_NARG(...)                                                                             \
  SERIALIZABLE_STRUCT_IMPL_NARG_(__VA_ARGS__, SERIALIZABLE_STRUCT_IMPL_RSEQ_N())
#define SERIALIZABLE_STRUCT_IMPL_NARG_(...) SERIALIZABLE_STRUCT_IMPL_ARG_N(__VA_ARGS__)

#define SERIALIZABLE_STRUCT_IMPL_PRIMITIVE_CAT(a, ...) a##__VA_ARGS__
#define SERIALIZABLE_STRUCT_IMPL_CAT(a, ...) SERIALIZABLE_STRUCT_IMPL_PRIMITIVE_CAT(a, __VA_ARGS__)

// endregion
// region ==== PER-FIELD MACROS ===========
#define SERIALIZABLE_STRUCT_IMPL_FIELD_DECL(_, type, field_name, ...) type field_name = __VA_ARGS__;
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT(struct_name, type, field_name, default_value)                           \
	field_name(vm[ #field_name ].empty() ? struct_name (). field_name : vm[#field_name].as<type>())
#define SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD(_, type, field_name, ...) type field_name
#define SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG(_, type, field_name, ...) field_name ( field_name )
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE(_, type, field_name, ...)                                    \
	boost::optional< type > field_name = tree.get_optional< type > ( #field_name );
#define SERIALIZABLE_STRUCT_IMPL_FIELD_FROM_OPTIONAL(_, type, field_name, ...)                                         \
	field_name ? field_name.get() : default_instance. field_name
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE(_, type, field_name, ...)                                           \
	tree.add( #field_name , field_name );
#define SERIALIZABLE_STRUCT_IMPL_FIELD_COMPARISON(_, type, field_name, ...)                                           \
	instance1. field_name == instance2. field_name
// endregion

// region ==== TOP-LEVEL MACROS ===========

#define SERIALIZABLE_STRUCT_IMPL( struct_name, ...)                                                                    \
	SERIALIZABLE_STRUCT_IMPL_2(struct_name, SERIALIZABLE_STRUCT_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_IMPL_2( struct_name, field_count, ...)                                                     \
	SERIALIZABLE_STRUCT_IMPL_3(struct_name, field_count,                                                               \
							 SERIALIZABLE_STRUCT_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_LOOP_, field_count), __VA_ARGS__)


#define SERIALIZABLE_STRUCT_IMPL_3( struct_name, field_count, loop, ...)                                               \
	struct struct_name {                                                                                               \
		loop(SERIALIZABLE_STRUCT_IMPL_FIELD_DECL, _, SERIALIZABLE_STRUCT_IMPL_NOTHING, __VA_ARGS__)                    \
		struct_name () = default;                                                                                      \
		struct_name(loop(SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD, _, SERIALIZABLE_STRUCT_IMPL_COMMA, __VA_ARGS__)):       \
			loop(SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG, _, SERIALIZABLE_STRUCT_IMPL_COMMA, __VA_ARGS__)              \
			{}                                                                                                         \
		explicit struct_name (const boost::program_options::variables_map& vm) :                                       \
			loop(SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT, struct_name, SERIALIZABLE_STRUCT_IMPL_COMMA, __VA_ARGS__)     \
		{}                                                                                                             \
		static struct_name BuildFromPTree(const boost::property_tree::ptree& tree){                                    \
			loop(SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE, _, SERIALIZABLE_STRUCT_IMPL_NOTHING, __VA_ARGS__)  \
			struct_name default_instance;                                                                              \
			return {                                                                                                   \
				loop(SERIALIZABLE_STRUCT_IMPL_FIELD_FROM_OPTIONAL, _, SERIALIZABLE_STRUCT_IMPL_COMMA, __VA_ARGS__)     \
			};																									       \
		}																											   \
		boost::property_tree::ptree ToPTree() const {                                                                  \
			boost::property_tree::ptree tree;                                                                          \
			loop(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE, _, SERIALIZABLE_STRUCT_IMPL_NOTHING, __VA_ARGS__)         \
			return tree;                                                                                               \
		}                                                                                                              \
		friend bool operator==(const struct_name & instance1, const struct_name & instance2){                          \
			return                                                                                                     \
			loop(SERIALIZABLE_STRUCT_IMPL_FIELD_COMPARISON, _, SERIALIZABLE_STRUCT_IMPL_AND, __VA_ARGS__);             \
		}                                                                                                              \
		friend std::ostream& operator<<(std::ostream& out, const struct_name& instance) {                              \
			boost::property_tree::ptree tree(instance.ToPTree());                                                      \
			boost::property_tree::write_json_no_quotes(out, tree, true);                                               \
			return out;                                                                                                \
		}                                                                                                              \
	};

//endregion

