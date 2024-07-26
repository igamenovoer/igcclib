#pragma once
#include <map>
#include <boost/graph/adjacency_list.hpp>
#include <vector>

#include "igcclib_graph_def.hpp"

//#include <boost/property_map/dynamic_property_map.hpp>

namespace _NS_UTILITY
{
	template<typename VERTEX_INFO, typename EDGE_INFO>
	class Graph
	{
	public:
		/// <summary>
		/// add a vertex to the graph
		/// </summary>
		/// <param name="data">the data associated with the vertex</param>
		/// <returns>vertex id</returns>
		virtual int add_vertex(const VERTEX_INFO& data) = 0;

		/// <summary>
		/// add an edge between two vertices
		/// </summary>
		/// <param name="v1">source vertex id</param>
		/// <param name="v2">target vertex id</param>
		/// <param name="data">the associated edge data</param>
		/// <returns>edge id for the added edge. -1 if the edge cannot be added</returns>
		virtual int add_edge(int v1, int v2, const EDGE_INFO& data) =0;

		/// <summary>
		/// set the new data associated with a vertex
		/// </summary>
		/// <param name="v">vertex id</param>
		/// <param name="data">the associated data</param>
		virtual void set_vertex_info(int v, const VERTEX_INFO& data) =0;

		/// <summary>
		/// set the new data associated with an edge
		/// </summary>
		/// <param name="edge">edge id</param>
		/// <param name="data">the associated data</param>
		virtual void set_edge_info(int edge, const EDGE_INFO& data) =0;

		/// <summary>
		/// get the edge between two vertices
		/// </summary>
		/// <param name="v1">the source vertex</param>
		/// <param name="v2">the target vertex</param>
		/// <returns>the edge id representing the edge between two vertices, a negative number if the edge does not exist</returns>
		virtual int get_edge(int v1, int v2) const =0;

		/// <summary>
		/// get the edge based on edge id
		/// </summary>
		/// <param name="edge_id">the edge id</param>
		/// <returns>a pair of vertex id (source, destination)</returns>
		virtual std::pair<int, int> get_edge(int edge_id) const = 0;

		/// <summary>
		/// retrieve a reference to the vertex data
		/// </summary>
		/// <param name="v">vertex id</param>
		/// <returns>the reference to the vertex data</returns>
		virtual VERTEX_INFO& get_vertex_info(int v) =0;

		/// <summary>
		/// retrieve a reference to the vertex data
		/// </summary>
		/// <param name="v">vertex id</param>
		/// <returns>the reference to the vertex data</returns>
		virtual const VERTEX_INFO& get_vertex_info(int v) const =0;

		/// <summary>
		/// retrieve a reference to the edge data
		/// </summary>
		/// <param name="edge">edge id</param>
		/// <returns>the reference to edge data</returns>
		virtual EDGE_INFO& get_edge_info(int edge) =0;

		/// <summary>
		/// retrieve a reference to the edge data
		/// </summary>
		/// <param name="edge">edge id</param>
		/// <returns>the reference to edge data</returns>
		virtual const EDGE_INFO& get_edge_info(int edge) const =0;

		/// <summary>
		/// get the edges originating from a vertex
		/// </summary>
		/// <param name="v">vertex id</param>
		/// <param name="output">a vector for storing the output edge id</param>
		virtual void get_edges_around_vertex(int v, std::vector<int>& output) const = 0;

		/// <summary>
		/// get the edges originating from a vertex
		/// </summary>
		/// <param name="v">vertex id</param>
		/// <returns>a list of edge id</returns>
		virtual std::vector<int> get_edges_around_vertex(int v) const
		{
			std::vector<int> output;
			get_edges_around_vertex(v, output);
			return output;
		}

		/// <summary>
		/// get the vertices neighboring a given vertex
		/// </summary>
		/// <param name="v">the vertex id</param>
		/// <param name="output">a vector for storing the output vertex ids</param>
		virtual void get_vertices_around_vertex(int v, std::vector<int>& output) const = 0;

		/// <summary>
		/// get the vertices neighboring a given vertex
		/// </summary>
		/// <param name="v">the vertex id</param>
		/// <returns>a list of vertex id</returns>
		virtual std::vector<int> get_vertices_around_vertex(int v) const
		{
			std::vector<int> output;
			get_vertices_around_vertex(v, output);
			return output;
		}

		/// <summary>
		/// get the number of vertices
		/// </summary>
		/// <returns>the number of vertices</returns>
		virtual int get_num_vertices() const = 0;

		/// <summary>
		/// get the number of edges
		/// </summary>
		/// <returns>the number edges</returns>
		virtual int get_num_edges() const = 0;
	};

	struct EmptyVertexInfo {};
	struct EmptyEdgeInfo {};

	/// <summary>
	/// Undirected graph that is fast for access vertex and edge, but slow for modification
	/// </summary>
	template<typename VERTEX_INFO, typename EDGE_INFO>
	class StaticUndirectedGraph: public Graph<VERTEX_INFO, EDGE_INFO>
	{
	public:
		virtual ~StaticUndirectedGraph() {}

	public:
		//vertex and edge information used in boost graph
		struct _bgl_vertex_info
		{
			VERTEX_INFO data;
			int index;
			_bgl_vertex_info():index(0) {};
			_bgl_vertex_info(const VERTEX_INFO& x, int idx) :data(x), index(idx) {}
		};

		struct _bgl_edge_info
		{
			EDGE_INFO data;
			int index;
			_bgl_edge_info():index(0) {};
			_bgl_edge_info(const EDGE_INFO& x, int idx) : data(x), index(idx) {}
		};

		//the BGL graph
	public:
		using Graph_t = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, _bgl_vertex_info, _bgl_edge_info>;
		using VertexDescriptor_t = typename Graph_t::vertex_descriptor;
		using EdgeDescriptor_t = typename Graph_t::edge_descriptor;
		using VertexInfo_t = VERTEX_INFO;
		using EdgeInfo_t = EDGE_INFO;

	protected:
		Graph_t m_graph;

		//relating vertices and indices
		std::vector<VertexDescriptor_t> m_vertex_list;
		std::vector<EdgeDescriptor_t> m_edge_list;

	public:
		virtual int add_vertex(const VERTEX_INFO& data) override;
		virtual int add_edge(int v1, int v2, const EDGE_INFO& data) override;

		virtual void set_vertex_info(int v, const VERTEX_INFO& data) override;
		virtual void set_edge_info(int edge, const EDGE_INFO& data) override;

		virtual int get_edge(int v1, int v2) const override;
		virtual std::pair<int, int> get_edge(int edge_id) const override;

		virtual int get_num_vertices() const override { return (int)m_vertex_list.size(); }
		virtual int get_num_edges() const override { return (int)m_edge_list.size(); }

		virtual VERTEX_INFO& get_vertex_info(int v) override;
		virtual const VERTEX_INFO& get_vertex_info(int v) const override;

		virtual EDGE_INFO& get_edge_info(int edge) override;
		virtual const EDGE_INFO& get_edge_info(int edge) const override;

		using Graph<VERTEX_INFO, EDGE_INFO>::get_edges_around_vertex;
		virtual void get_edges_around_vertex(int v, std::vector<int>& output) const override;

		using Graph<VERTEX_INFO, EDGE_INFO>::get_vertices_around_vertex;
		virtual void get_vertices_around_vertex(int v, std::vector<int>& output) const override;

		VertexDescriptor_t get_bgl_vertex_descriptor(int vertex_id) const {
			return m_vertex_list[vertex_id];
		}

		EdgeDescriptor_t get_bgl_edge_descriptor(int edge_id) const {
			return m_edge_list[edge_id];
		}

		/// <summary>
		/// get the internal graph, which is a boost graph, for advance operations
		/// </summary>
		/// <returns>the boost graph</returns>
		Graph_t& get_graph()
		{
			return m_graph;
		}

		/// <summary>
		/// get the internal graph, which is a boost graph, for advance operations
		/// </summary>
		/// <returns>the boost graph</returns>
		const Graph_t& get_graph() const
		{
			return m_graph;
		}
	};

	template<typename VERTEX_INFO, typename EDGE_INFO>
	std::pair<int, int> StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>::get_edge(int edge_id) const
	{
		auto ed = m_edge_list[edge_id];
		auto v1 = source(ed, m_graph);
		auto v2 = target(ed, m_graph);

		return{ m_graph[v1].index, m_graph[v2].index };
	}

	template<typename VERTEX_INFO, typename EDGE_INFO>
	void StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>::get_vertices_around_vertex(int v, std::vector<int>& output) const
	{
		output.clear();
		auto vd = m_vertex_list[v];
		auto res = boost::adjacent_vertices(vd, m_graph);
		for (auto x = res.first; x != res.second; ++x)
		{
			auto vertex_id = m_graph[*x].index;
			output.push_back(vertex_id);
		}
	}

	template<typename VERTEX_INFO, typename EDGE_INFO>
	void StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>::get_edges_around_vertex(int v, std::vector<int>& output) const
	{
		output.clear();
		auto vd = m_vertex_list[v];
		auto res = boost::adjacent_vertices(vd, m_graph);
		for (auto x = res.first; x != res.second; ++x)
		{
			auto ed = edge(vd, *x, m_graph);
			auto edge_id = m_graph[ed.first].index;
			output.push_back(edge_id);
		}
	}

	template<typename VERTEX_INFO, typename EDGE_INFO>
	const EDGE_INFO& StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>::get_edge_info(int edge) const
	{
		auto ed = m_edge_list[edge];
		return m_graph[ed].data;
	}

	template<typename VERTEX_INFO, typename EDGE_INFO>
	EDGE_INFO& StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>::get_edge_info(int edge)
	{
		auto ed = m_edge_list[edge];
		return m_graph[ed].data;
	}

	template<typename VERTEX_INFO, typename EDGE_INFO>
	const VERTEX_INFO& StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>::get_vertex_info(int v) const
	{
		auto vd = m_vertex_list[v];
		return m_graph[vd].data;
	}

	template<typename VERTEX_INFO, typename EDGE_INFO>
	VERTEX_INFO& StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>::get_vertex_info(int v)
	{
		auto vd = m_vertex_list[v];
		return m_graph[vd].data;
	}

	template<typename VERTEX_INFO, typename EDGE_INFO>
	int StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>::get_edge(int v1, int v2) const
	{
		auto x1 = m_vertex_list[v1];
		auto x2 = m_vertex_list[v2];
		auto res = edge(x1, x2, m_graph);
		if (res.second)
			return m_graph[res.first].index;
		else
			return -1;
	}

	template<typename VERTEX_INFO, typename EDGE_INFO>
	void StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>::set_edge_info(int edge, const EDGE_INFO& data)
	{
		auto ed = m_edge_list[edge];
		m_graph[ed].data = data;
	}

	template<typename VERTEX_INFO, typename EDGE_INFO>
	void StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>::set_vertex_info(int v, const VERTEX_INFO& data)
	{
		auto vd = m_vertex_list[v];
		m_graph[vd].data = data;
	}

	template<typename VERTEX_INFO, typename EDGE_INFO>
	int StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>::add_edge(int v1, int v2, const EDGE_INFO& data)
	{
		auto vd_1 = m_vertex_list[v1];
		auto vd_2 = m_vertex_list[v2];
		auto ed = boost::add_edge(vd_1, vd_2, m_graph);

		//check if vertex is successfully added
		assert_throw(ed.second, "failed to add edge");

		//record the edge info
		_bgl_edge_info info(data, (int)m_edge_list.size());
		m_graph[ed.first] = info;
		m_edge_list.push_back(ed.first);

		return info.index;
	}

	template<typename VERTEX_INFO, typename EDGE_INFO>
	int StaticUndirectedGraph<VERTEX_INFO, EDGE_INFO>::add_vertex(const VERTEX_INFO& data)
	{
		_bgl_vertex_info info(data, (int)m_vertex_list.size());
		auto vd = boost::add_vertex(info, m_graph);
		m_vertex_list.push_back(vd);

		return info.index;
	}

	using StaticUndirectedGraphNoInfo = StaticUndirectedGraph<EmptyVertexInfo, EmptyEdgeInfo>;
};