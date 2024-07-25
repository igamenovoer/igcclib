#pragma once

#include <vector>

#include <igcclib/igcclib_master.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Cartesian.h>

//AABB tree
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/AABB_segment_primitive.h>

//triangle mesh
#include <CGAL/Surface_mesh.h>

//triangulation
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>

//polyhedron
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_2.h>

//kernel operations
#include <CGAL/intersections.h>
/*
//kdtree
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Orthogonal_incremental_neighbor_search.h>
*/


namespace _NS_UTILITY
{
	constexpr double CVAL_PI = 3.1415926535897932384626433832795;
	class _Epick
		: public CGAL::Filtered_kernel_adaptor<
		CGAL::Type_equality_wrapper< 
		CGAL::Simple_cartesian<float_type>::Base<_Epick>::Type, _Epick >,true >
	{};
	using KERNEL = _Epick; //this is the Exact_predicates_inexact_constructions_kernel for float_type
	//typedef CGAL::Exact_predicates_inexact_constructions_kernel KERNEL;
	typedef KERNEL::Point_3 POINT_3;
	typedef KERNEL::Point_2 POINT_2;
	typedef KERNEL::Triangle_3 TRI_3;
	typedef KERNEL::Plane_3 PLANE;
	typedef KERNEL::Ray_3 RAY_3;
	typedef KERNEL::Vector_3 VEC_3;
	typedef KERNEL::Segment_3 SEGMENT_3;
	typedef KERNEL::Line_3 LINE_3;
	
	using POLYGON_2 = CGAL::Polygon_2 <KERNEL>;	

	typedef std::vector<POINT_3> POINTLIST_3;
	typedef std::vector<POINT_2> POINTLIST_2;

	typedef std::vector<SEGMENT_3> SEGLIST_3;
	typedef std::vector<POINTLIST_3> POLYLINES_3;
	typedef std::vector<TRI_3> TRILIST_3;

	//for polygonal faces
	typedef std::vector<CGAL::SM_Vertex_index> POLYFACE; //indices of vertices that make up a polygon face
	typedef std::vector<POLYFACE> POLYFACELIST;

	//mesh
	typedef CGAL::Surface_mesh<POINT_2> TRIMESH_2;
	typedef CGAL::Surface_mesh<POINT_3> TRIMESH_3;

	//AABB tree definitions
	//triangle tree
	template<typename CGAL_KERNEL, typename STD_CONTAINER_TYPE>
	class AABB_Triangle
	{
	public:
		typedef typename STD_CONTAINER_TYPE::iterator iterator;
		typedef CGAL::AABB_triangle_primitive<CGAL_KERNEL, iterator> primitive;
		typedef CGAL::AABB_traits<CGAL_KERNEL, primitive> traits;
		typedef CGAL::AABB_tree<traits> tree;
	};
	typedef AABB_Triangle<KERNEL, TRILIST_3>::tree FACETREE_3;

	//segment tree
	template<typename CGAL_KERNEL, typename STD_CONTAINER_TYPE>
	class AABB_Segment
	{
	public:
		typedef typename STD_CONTAINER_TYPE::iterator iterator;
		typedef CGAL::AABB_segment_primitive<CGAL_KERNEL, iterator> primitive;
		typedef CGAL::AABB_traits<CGAL_KERNEL, primitive> traits;
		typedef CGAL::AABB_tree<traits> tree;
	};
	typedef AABB_Segment<KERNEL, SEGLIST_3>::tree SEGTREE_3;
	typedef CGAL::Polyhedron_3<KERNEL> POLYHEDRON_3;
	/*
	//3d kdtree with index
	typedef boost::tuple<POINT_3, int> _KDPoint3;
	typedef CGAL::Search_traits_adapter<_KDPoint3,
		CGAL::Nth_of_tuple_property_map<0, _KDPoint3>,
		CGAL::Search_traits_3<KERNEL> > _KDTraits3;
	typedef CGAL::Orthogonal_incremental_neighbor_search<_KDTraits3> KDSearchInc3;
	typedef KDSearchInc3::Tree KDTREE_3;

	//2d kdtree with index
	typedef boost::tuple<POINT_2, int> _KDPoint2;
	typedef CGAL::Search_traits_adapter<_KDPoint2,
		CGAL::Nth_of_tuple_property_map<0, _KDPoint2>,
		CGAL::Search_traits_2<KERNEL> > _KDTraits2;
	typedef CGAL::Orthogonal_incremental_neighbor_search<_KDTraits2> KDSearchInc2;
	typedef KDSearchInc2::Tree KDTREE_2;
	*/

	//triangulations
	namespace Triangulate_2
	{
		struct FaceInfo
		{
			//face in which hole?
			int nesting_level = -1;

			//face is visited in traversal?
			bool visited = false;

			//whatever you want to attach
			void* data = 0;
		};

		struct VertexInfo
		{
			//index of the vertex
			int index = -1;

			//custom data
			void* data = 0;
		};

		typedef CGAL::Triangulation_vertex_base_with_info_2<VertexInfo, KERNEL> Vb;
		typedef CGAL::Delaunay_mesh_vertex_base_2<KERNEL, Vb> VERTEX_BASE;

		typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo, KERNEL> Fb;
		typedef CGAL::Constrained_triangulation_face_base_2<KERNEL, Fb> Fbb;
		typedef CGAL::Constrained_Delaunay_triangulation_face_base_2<KERNEL,Fbb> Fbbb;
		typedef CGAL::Delaunay_mesh_face_base_2<KERNEL, Fbbb> FACE_BASE;

		typedef CGAL::Triangulation_data_structure_2<VERTEX_BASE, FACE_BASE> TDS;
		typedef CGAL::Constrained_Delaunay_triangulation_2<KERNEL, TDS> CONSTRAINED_DELAUNAY_2;
		typedef CGAL::Delaunay_mesh_size_criteria_2<CONSTRAINED_DELAUNAY_2> CRITERIA_CDT_2;
	}
};

//cereal support for some object
namespace cereal {
	// ============== for plane =================
	template<typename Archive_t>
	void save(Archive_t& ar, const _NS_UTILITY::PLANE& p) {
		ar(p.a(), p.b(), p.c());
		ar(p.d());
	}

	template<typename Archive_t>
	void load(Archive_t& ar, _NS_UTILITY::PLANE& p) {
		using FT = decltype(p.a());
		FT a, b, c, d;
		ar(a, b, c);
		ar(d);
		p = _NS_UTILITY::PLANE(a, b, c, d);
	}
}