#pragma once
#include <vector>
#include <igcclib/core/igcclib_eigen_def.hpp>
#include <igcclib/geometry/igcclib_cgal_def.hpp>

#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_adapter.h>
#include <boost/tuple/tuple.hpp>

namespace _NS_UTILITY
{
	class SpatialQuery_2
	{
	public:
		using TraitsBase = CGAL::Search_traits_2<KERNEL>;
		using PointWithIndex = boost::tuple<POINT_2, size_t>;
		using Traits = CGAL::Search_traits_adapter<PointWithIndex,
			CGAL::Nth_of_tuple_property_map<0, PointWithIndex>,
			TraitsBase>;
		using KdTree = CGAL::Kd_tree<Traits>;


	protected:
		std::shared_ptr<KdTree> m_tree;

		//keep a record of the points
		std::vector<POINT_2> m_points;

	public:
		SpatialQuery_2();
		virtual ~SpatialQuery_2() {};

		bool is_empty() const { return m_tree->size() == 0; }

		/** \brief initialize with a list of 2d points */
		void init_with_points(const std::vector<POINT_2>& pts);

		/** \brief add a new point to the query structure, return the index of the point */
		size_t insert_point(const POINT_2& p);

		/** \brief find points in a bounding box, return the point indices */
		std::vector<size_t> query_by_aabb(const POINT_2& minc, const POINT_2& maxc) const;

		/** \brief find all the points whose distance to a center is no more than r */
		std::vector<size_t> query_by_radius(const POINT_2& center, double r) const;

		/**
		* \brief find k-nearest-neighbor of a given point p 
		*
		* \param p	the query input point
		* \param k	number of nearest neighbor to find
		* \param out_distance	output distances of the found points, out_distance[i] is the distance to the i-th returned point
		* \return std::vector<size_t>	indices of the found points, sorted by distance from min to max
		*/
		std::vector<size_t> query_by_point(const POINT_2& p, int k, std::vector<double>* out_distance=nullptr) const;

		/** \brief get all the points */
		const std::vector<POINT_2>& get_points() const;
	};
}


