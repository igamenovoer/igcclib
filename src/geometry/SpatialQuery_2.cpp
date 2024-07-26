#include <igcclib/geometry/SpatialQuery_2.hpp>
#include <CGAL/Fuzzy_iso_box.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>

namespace _NS_UTILITY
{
	using QueryBox = CGAL::Fuzzy_iso_box<SpatialQuery_2::Traits>;
	using QuerySphere = CGAL::Fuzzy_sphere<SpatialQuery_2::Traits>;
	using NeighborSearch = CGAL::Orthogonal_k_neighbor_search<SpatialQuery_2::Traits>;

	SpatialQuery_2::SpatialQuery_2()
	{
		m_tree.reset(new KdTree());
	}

	void SpatialQuery_2::init_with_points(const std::vector<POINT_2>& pts)
	{
		m_tree.reset(new KdTree());
		m_points = pts;

		for (size_t i = 0; i < pts.size(); i++)
		{
			PointWithIndex p{ pts[i], i };
			m_tree->insert(p);
		}
	}

	size_t SpatialQuery_2::insert_point(const POINT_2& p)
	{
		m_points.push_back(p);
		m_tree->insert({ p, m_points.size() - 1 });
		return m_points.size() - 1;
	}

	std::vector<size_t> SpatialQuery_2::query_by_aabb(const POINT_2& minc, const POINT_2& maxc) const
	{
		QueryBox box(minc, maxc);
		std::vector<PointWithIndex> _output;
		m_tree->search(std::back_inserter(_output), box);

		std::vector<size_t> output(_output.size());
		for (size_t i = 0; i < _output.size(); i++)
			output[i] = boost::get<1>(_output[i]);
		return output;
	}

	std::vector<size_t> SpatialQuery_2::query_by_radius(const POINT_2& center, double r) const
	{
		QuerySphere ball(center, r);
		std::vector<PointWithIndex> _output;
		m_tree->search(std::back_inserter(_output), ball);

		std::vector<size_t> output(_output.size());
		for (size_t i = 0; i < _output.size(); i++)
			output[i] = boost::get<1>(_output[i]);
		return output;
	}

	std::vector<size_t> SpatialQuery_2::query_by_point(const POINT_2& p, int k, std::vector<double>* out_distance/*=nullptr*/) const
	{
		NeighborSearch search(*m_tree, p, k);
		std::vector<size_t> out_indices;
		std::vector<double> out_dist;

		for (auto it = search.begin(); it != search.end(); ++it)
		{
			auto idx = boost::get<1>(it->first);
			out_indices.push_back(idx);
			out_dist.push_back(std::sqrt(std::abs(it->second)));
		}

		if (out_distance)
			* out_distance = out_dist;
		return out_indices;
	}

	const std::vector<_NS_UTILITY::POINT_2>& SpatialQuery_2::get_points() const
	{
		return m_points;
	}
}
