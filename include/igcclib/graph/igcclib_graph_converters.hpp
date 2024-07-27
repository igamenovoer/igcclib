#pragma once

#include <igcclib/graph/igcclib_graph_def.hpp>
#include <igcclib/graph/GeometricGraph.hpp>
#include <igcclib/geometry/TriangularMesh.hpp>

namespace _NS_UTILITY {
	/** \brief convert triangular mesh into geometry graph */
	inline void graph_from_mesh(
		StaticGeometryGraph_3& out_graph, 
		const TriangularMesh& mesh, 
		EdgeWeightType weight_type = EdgeWeightType::L2);
}

namespace _NS_UTILITY {
	void graph_from_mesh(
		StaticGeometryGraph_3& out_graph, 
		const TriangularMesh& mesh, 
		EdgeWeightType weight_type /*= EdgeWeightType::L2*/)
	{
		out_graph = StaticGeometryGraph_3();	//clear the content

		//add vertices
		auto v = mesh.get_vertices();
		for (decltype(v.rows()) i = 0; i < v.rows(); i++) {
			out_graph.add_vertex({ v(i,0), v(i,1), v(i,2) });
		}

		//add faces
		const auto& f = mesh.get_faces();
		for (decltype(f.rows()) i = 0; i < f.rows(); i++) {
			auto a = f(i, 0);
			auto b = f(i, 1);
			auto c = f(i, 2);

			WeightedEdge w_ab, w_bc, w_ac;
			if (weight_type == EdgeWeightType::HOP || weight_type == EdgeWeightType::GENERAL){
				w_ab.weight = 1;
				w_bc.weight = 1;
				w_ac.weight = 1;
			}
			else if (weight_type == EdgeWeightType::L2) {
				w_ab.weight = (v.row(a) - v.row(b)).norm();
				w_bc.weight = (v.row(b) - v.row(c)).norm();
				w_ac.weight = (v.row(a) - v.row(c)).norm();
			}

			if (out_graph.get_edge(a, b) < 0)
				out_graph.add_edge(a, b, w_ab);
			if (out_graph.get_edge(b, c) < 0)
				out_graph.add_edge(b, c, w_bc);
			if (out_graph.get_edge(a, c) < 0)
				out_graph.add_edge(a, c, w_ac);
		}
	}
}