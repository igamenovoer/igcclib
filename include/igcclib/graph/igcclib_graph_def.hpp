#pragma once
#include <igcclib/igcclib_master.hpp>

namespace _NS_UTILITY
{
	//the meaning of the edge weight
	enum class EdgeWeightType {
		GENERAL,	//no meaning
		HOP,	// all 1, representing hop
		L2	// representing l2 distance between nodes
	};
};
