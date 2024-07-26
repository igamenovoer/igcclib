#pragma once
#include "UndirectedGraph.hpp"
#include <igcclib/core/igcclib_eigen_def.hpp>

namespace _NS_UTILITY
{
	struct PositionalVertex_3
	{
		fVECTOR_3 position;
		PositionalVertex_3() :position(3) {
			position.setZero();
		}

		PositionalVertex_3(float_type x, float_type y, float_type z) : position{ x,y,z } {}
	};

	struct WeightedEdge
	{
		double weight;
	};

	using StaticGeometryGraph_3 = StaticUndirectedGraph<PositionalVertex_3, WeightedEdge>;	
};
