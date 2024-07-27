#include <igcclib/graph/igcclib_graph_algorithms.hpp>
#include <igcclib/graph/GeometricGraph.hpp>

using namespace _NS_UTILITY;
int main()
{
	StaticGeometryGraph_3 g;
	std::vector<double> distance;
	StaticGeometryGraph_3::VertexInfo_t v(0, 0, 0);
	StaticGeometryGraph_3::EdgeInfo_t einfo;
	einfo.weight = 1.0;

	std::vector<std::pair<int, int>> edgelist = {
		{0,1},{0,2},{1,3},{2,3},{3,4}
	};

	std::vector<int> vdlist;
	for (int i = 0; i < 5; i++)
		vdlist.push_back(g.add_vertex(v));

	for (auto x : edgelist)
		g.add_edge(x.first, x.second, einfo);

	multi_source_shortest_distance(g, {0,4}, distance);

	for (int i = 0; i < distance.size(); i++)
		printf("%d: %f\n", i, distance[i]);

	return 0;
}