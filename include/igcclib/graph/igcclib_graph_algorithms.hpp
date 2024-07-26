#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/transform_value_property_map.hpp>

#include "igcclib_graph_def.hpp"
#include "UndirectedGraph.hpp"

namespace _NS_UTILITY
{
	/**
	* \brief find the shortest path that passes through specified sites in order.
	*
	* \param g	the graph
	* \param sites	the sites to be passed in order, from sites[0] to sites.back(). It must contain at least 2 sites.
	* \param out_path	the shortest that including all sites
	* \param out_distance	the distance of the whole path. Will be negative if a path
	going through all sites does not exist.
	*/
	template<typename VERTEX_INFO, typename EDGE_INFO>
	inline void find_shortest_path_multi_step(
		const StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>& g,
		const std::vector<int>& sites, std::vector<int>* out_path, double* out_distance);

	/// <summary>
	/// compute multi-source shortest distances over a graph.
	/// The graph's edge info must contain a member called "weight" which defines the distance between vertices
	/// </summary>
	/// <param name="g">the graph</param>
	/// <param name="source_vertices">source vertices id</param>
	/// <param name="output_distances">the shortest distance from each vertex to any vertex in source vertices.
	/// output_distances[i] is the distance from vertex i to any vertex in source_vertices</param>
	template<typename VERTEX_INFO, typename EDGE_INFO>
	inline void multi_source_shortest_distance(
		const StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>& g, 
		const std::vector<int>& source_vertices, std::vector<double>& output_distances);

	/**
	* \brief find shortest path between two vertices in a graph
	*
	* \param g the input graph
	* \param source_vertex	index of the source vertex
	* \param target_vertex	index of the target vertex
	* \param out_path	the path from source to target, including the end points
	* \param out_distance	the distance of the shortest path
	*/
	template<typename VERTEX_INFO, typename EDGE_INFO>
	inline void find_shortest_path(
		const StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>& g,
		int source_vertex, int target_vertex, std::vector<int>* out_path, double* out_distance);

	// ======================================================================================================

	/// <summary>
	/// compute multi-source shortest distances over a graph.
	/// The graph's edge info must contain a member called "weight" which defines the distance between vertices
	/// </summary>
	/// <param name="g">the graph</param>
	/// <param name="source_vertices">source vertices id</param>
	/// <param name="output_distances">the shortest distance from each vertex to any vertex in source vertices.
	/// output_distances[i] is the distance from vertex i to any vertex in source_vertices</param>
	template<typename VERTEX_INFO, typename EDGE_INFO>
	inline void multi_source_shortest_distance(const StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>& g, const std::vector<int>& source_vertices, std::vector<double>& output_distances)
	{
		typedef StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO> GRAPH;
		const auto& b_graph = g.get_graph();

		//create weight map
		auto edge_weight_func = [](const GRAPH::_bgl_edge_info& edge) {return edge.data.weight; };
		auto _edge2weight = boost::make_transform_value_property_map(edge_weight_func, get(boost::edge_bundle, b_graph));

		//create vertex index map
		auto vertex_index_func = [](const GRAPH::_bgl_vertex_info& v) {return v.index; };
		auto _vertex2index = boost::make_transform_value_property_map(vertex_index_func, get(boost::vertex_bundle, b_graph));

		//create distance storage
		output_distances.resize(g.get_num_vertices());
		auto _vertex2distance = boost::make_iterator_property_map(output_distances.begin(), _vertex2index);

		//get sources
		std::vector<GRAPH::VertexDescriptor_t> vdlist(source_vertices.size());
		for (int i = 0; i < source_vertices.size(); i++)
			vdlist[i] = g.get_bgl_vertex_descriptor(source_vertices[i]);

		boost::dijkstra_shortest_paths(b_graph, vdlist.begin(), vdlist.end(),
			boost::dummy_property_map(),
			_vertex2distance,
			_edge2weight,
			_vertex2index,
			std::less<double>(),
			boost::closed_plus<double>(),
			std::numeric_limits<double>::max(),
			0.0,
			boost::dijkstra_visitor<>());

		for (auto& x : output_distances)
			if (x == std::numeric_limits<double>::max())
				x = -1;
	}

	/**
	* \brief find shortest path between two vertices in a graph
	*
	* \param g the input graph
	* \param source_vertex	index of the source vertex
	* \param target_vertex	index of the target vertex
	* \param out_path	the path from source to target, including the end points
	* \param out_distance	the distance of the shortest path
	*/
	template<typename VERTEX_INFO, typename EDGE_INFO>
	inline void find_shortest_path(
		const StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>& g,
		int source_vertex, int target_vertex, std::vector<int>* out_path, double* out_distance) 
	{
		find_shortest_path_multi_step(g, { source_vertex, target_vertex }, out_path, out_distance);
	}

	/**
	* \brief find the shortest path that passes through specified sites in order.
	*
	* \param g	the graph
	* \param sites	the sites to be passed in order, from sites[0] to sites.back(). It must contain at least 2 sites.
	* \param out_path	the shortest that including all sites
	* \param out_distance	the distance of the whole path. Will be negative if a path 
	going through all sites does not exist.
	*/
	template<typename VERTEX_INFO, typename EDGE_INFO>
	inline void find_shortest_path_multi_step(
		const StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>& g,
		const std::vector<int>& sites, std::vector<int>* out_path, double* out_distance) 
	{
		assert_throw(sites.size() >= 2, "must have at least 2 sites");

		typedef StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO> GRAPH;
		const auto& b_graph = g.get_graph();

		//create weight map
		auto edge_weight_func = [](const GRAPH::_bgl_edge_info& edge) {
			return edge.data.weight;
		};
		auto _edge2weight = boost::make_transform_value_property_map(edge_weight_func, get(boost::edge_bundle, b_graph));

		//create vertex index map
		auto vertex_index_func = [](const GRAPH::_bgl_vertex_info& v) {
			return v.index;
		};
		auto _vertex2index = boost::make_transform_value_property_map(vertex_index_func, get(boost::vertex_bundle, b_graph));

		//early stop control
		struct EndOfSearch {};
		struct EarlyStopVisitor : boost::default_dijkstra_visitor {
			typename GRAPH::VertexDescriptor_t target;
			EarlyStopVisitor(typename GRAPH::VertexDescriptor_t dst) : target{ dst } {}
			void finish_vertex(typename GRAPH::VertexDescriptor_t u, decltype(b_graph)& g) {
				if (u == target)
					throw EndOfSearch();
			}
		};

		//predecessor map
		std::vector<size_t> predmap(boost::num_vertices(b_graph));

		//distance output
		std::vector<double> distmap(boost::num_vertices(b_graph));

		auto _predmap = boost::make_iterator_property_map(predmap.begin(), _vertex2index);
		auto _distmap = boost::make_iterator_property_map(distmap.begin(), _vertex2index);
		double inf_value = std::numeric_limits<double>::max();

		std::vector<int> final_path;
		double final_distance = 0;
		final_path.push_back(sites[0]);
		bool has_path = true;

		for (size_t i = 0; i < sites.size()-1; i++) {
			auto source_vertex = sites[i];
			auto target_vertex = sites[i + 1];
			auto v_source = g.get_bgl_vertex_descriptor(source_vertex);
			auto v_target = g.get_bgl_vertex_descriptor(target_vertex);

			EarlyStopVisitor vis(v_target);

			//run dijkstra
			try {
				boost::dijkstra_shortest_paths(b_graph, v_source,
					boost::weight_map(_edge2weight)
					.vertex_index_map(_vertex2index)
					.predecessor_map(_predmap)
					.distance_map(_distmap)
					.distance_inf(inf_value)
					.visitor(vis)
				);
			}
			catch (EndOfSearch&) {
				//early stop
			}

			if (distmap[target_vertex] == inf_value) //dead path
			{
				has_path = false;
				break;
			}

			//extract path
			std::vector<int> sub_path;
			sub_path.push_back(target_vertex);
			int x = target_vertex;
			while (x != source_vertex) {
				x = (int)predmap[x];
				sub_path.push_back(x);
			}
			std::reverse(sub_path.begin(), sub_path.end());

			//append it
			final_path.insert(final_path.end(), sub_path.begin() + 1, sub_path.end());
			final_distance += distmap[target_vertex];
		}

		if (out_path)
			*out_path = final_path;
		if (out_distance)
			*out_distance = final_distance;
	}
};