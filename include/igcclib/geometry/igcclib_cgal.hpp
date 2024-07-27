#pragma once

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Barycentric_coordinates_2/triangle_coordinates_2.h> 

#include <igcclib/geometry/igcclib_cgal_def.hpp>

// for mean value coordinate
// #include <CGAL/Barycentric_coordinates_2/Mean_value_2.h>

namespace _NS_UTILITY
{
	/// <summary>
	/// compute mean value coordinates in 2d
	/// </summary>
	/// <param name="polygon_pts">the polygon point sequence, the first and last point DO NOT have to be the same,
	/// as the polygon will be automatically closed</param>
	/// <param name="query_pts">points against which mean value coordinates are computed</param>
	/// <param name="output">output mean value coordinates for each point</param>
	IGCCLIB_API void compute_mean_value_coordinate_2(
		const POINTLIST_2& polygon_pts, const POINTLIST_2& query_pts,
		std::vector<std::vector<float_type>>& output);

	//compute barycentric coordinate given a 3d point and a 3d triangle
	IGCCLIB_API void compute_barycentric_coordinate(const TRI_3& tri,
		const POINTLIST_3& pts, POINTLIST_3& output);

	//compute barycentric coordinate given a 3d point and a 3d triangle
	IGCCLIB_API POINT_3 compute_barycentric_coordinate(const TRI_3& tri, POINT_3 pt);

	//re label all vertices
	IGCCLIB_API void re_label_all_vertices(Triangulate_2::CONSTRAINED_DELAUNAY_2& cdt);

	template<typename _POINT>
	void make_trimesh(const std::vector<_POINT>& pts,
		const POLYFACELIST& faces, CGAL::Surface_mesh<_POINT>* outmesh)
	{
		typedef CGAL::Surface_mesh<_POINT> _TMESH;
		typedef typename _TMESH::vertex_index VI;
		typedef typename _TMESH::face_index FI;

		//create mesh
		_TMESH trimesh;
		if (outmesh)
			outmesh->clear();
		else
			outmesh = &trimesh;

		//outmesh->reserve(pts.size(), faces.size() * 3, 0);
		for (size_t i = 0; i < pts.size(); i++)
			outmesh->add_vertex(pts[i]);

		for (size_t i = 0; i < faces.size(); i++)
		{
			outmesh->add_face(faces[i]);
		}
	}

	//update the in_domain mark for each triangle
	IGCCLIB_API void mark_triangles_out_of_domain(Triangulate_2::CONSTRAINED_DELAUNAY_2* inout);

	IGCCLIB_API void mark_domain(
		Triangulate_2::CONSTRAINED_DELAUNAY_2& inout,
		Triangulate_2::CONSTRAINED_DELAUNAY_2::Face_handle seedface,
		bool is_seed_in_domain,
		bool reset_visited_flag);

	IGCCLIB_API void mark_domain_alternating(
		Triangulate_2::CONSTRAINED_DELAUNAY_2& inout);

	IGCCLIB_API void triangulate_polygon_2(const std::vector<POINTLIST_2>& ptslist, Triangulate_2::CONSTRAINED_DELAUNAY_2* out);

	//pts is the polygon, with or without end point duplication
	IGCCLIB_API void triangulate_polygon_2(const POINTLIST_2& pts, Triangulate_2::CONSTRAINED_DELAUNAY_2* out);

	//optimize triangulation
	IGCCLIB_API void optimize_triangulation(Triangulate_2::CONSTRAINED_DELAUNAY_2& cdt, double precision = 1e-6, int max_iter = 0);

	//refine triangulation.
	//optimize = true iff the mesh is to be optimized so that angles of the triangles are near 60 degrees
	IGCCLIB_API void refine_triangulation(
		Triangulate_2::CONSTRAINED_DELAUNAY_2& cdt, 
		double max_segment_length);

	template<typename P_TYPE>
	void read_vertex_face(const CGAL::Surface_mesh<P_TYPE>& trimesh, std::vector<P_TYPE>* vertices, POLYFACELIST* faces)
	{
		typedef CGAL::Surface_mesh<P_TYPE> TRIMESH;
		typedef typename TRIMESH::vertex_index VI;
		typedef typename TRIMESH::face_index FI;

		//read vertices
		if (vertices)
		{
			vertices->clear();
			BOOST_FOREACH(VI v, trimesh.vertices())
				vertices->push_back(trimesh.point(v));
		}

		//read faces
		if (faces)
		{
			faces->clear();
			BOOST_FOREACH(FI f, trimesh.faces())
			{
				faces->emplace_back();
				auto h = trimesh.halfedge(f);
				BOOST_FOREACH(VI v, trimesh.vertices_around_face(h))
					faces->back().push_back(v);
			}
		}
	}

	template<typename HDS>
	class PolyhedronFromVertexFace : public CGAL::Modifier_base<HDS>
	{
	public:
		typedef typename HDS::Traits::Point_3 POINT;
		std::vector<POINT> m_vertices;
		POLYFACELIST m_faces;
		bool m_verbose;
		PolyhedronFromVertexFace(const std::vector<POINT>& pts, const POLYFACELIST& faces, bool verbose = false)
		{
			m_vertices = pts;
			m_faces = faces;
			m_verbose = verbose;
		}

		void operator()(HDS& hds)
		{
			CGAL::Polyhedron_incremental_builder_3<HDS> builder(hds);

			builder.begin_surface(m_vertices.size(), m_faces.size());

			//add vertices
			for (auto& p : m_vertices)
				builder.add_vertex(p);

			//add faces
			for (auto& f : m_faces)
				builder.add_facet(f.begin(), f.end());

			builder.end_surface();
		}
	};

	IGCCLIB_API void make_polyhedron(const POINTLIST_3& pts, const POLYFACELIST& faces, POLYHEDRON_3& outpoly, bool verbose = false);

	IGCCLIB_API void to_surface_mesh(const POLYHEDRON_3& poly, TRIMESH_3& trimesh, bool triangulate = false);

	IGCCLIB_API void compute_vertex_normals(const TRIMESH_3& trimesh, std::vector<VEC_3>& output);
	IGCCLIB_API void compute_face_normals(const TRIMESH_3& trimesh, std::vector<VEC_3>& output);
}