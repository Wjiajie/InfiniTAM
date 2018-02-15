//  ================================================================
//  Created by Gregory Kramida on 2/14/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include <ostream>
#include <tuple>
namespace ITMLib {
template<typename Type, unsigned N, unsigned Last>
struct tuple_printer {

	static void print(std::ostream& out, const Type& value) {
		out << std::get<N>(value) << ", ";
		tuple_printer<Type, N + 1, Last>::print(out, value);
	}
};

template<typename Type, unsigned N>
struct tuple_printer<Type, N, N> {

	static void print(std::ostream& out, const Type& value) {
		out << std::get<N>(value);
	}

};

template<typename... Types>
std::ostream& operator<<(std::ostream& out, const std::tuple<Types...>& value) {
	out << "(";
	tuple_printer<std::tuple<Types...>, 0, sizeof...(Types) - 1>::print(out, value);
	out << ")";
	return out;
}
}