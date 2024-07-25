#include "util_cgal.h"
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <boost/property_map/function_property_map.hpp>
#include <CGAL/lloyd_optimize_mesh_2.h>
//#include <CGAL/Barycentric_coordinates_2/Mean_value_coordinates_2.h>

namespace _NS_UTILITY {

	template<typename PH> //PH = polyhedron type
	void _make_polyhedron(const std::vector<typename PH::HalfedgeDS::Traits::Point_3>& pts,
		const POLYFACELIST& faces, PH& outpoly, bool verbose /*= false*/)
	{
		typedef typename PH::HalfedgeDS HDS;
		using namespace CGAL;

		//prepare point data
		typedef typename PolyhedronFromVertexFace<HDS>::POINT POINT;
		std::vector<POINT> vertices(pts.size());
		for (size_t i = 0; i < pts.size(); i++)
			vertices[i] = POINT(pts[i].x(), pts[i].y(), pts[i].z());

		//create builder and make the polyhedron
		PolyhedronFromVertexFace<HDS> builder(vertices, faces, verbose);
		outpoly.clear();
		outpoly.delegate(builder);
	}

	template<typename PH>
	void _to_surface_mesh(const PH& poly,
		CGAL::Surface_mesh<typename PH::Traits::Point_3>& trimesh,
		bool triangulate /*= false*/)
	{
		namespace pmp = CGAL::Polygon_mesh_processing;
		typedef typename PH::Point_3 _POINT;
		typedef typename PH::HalfedgeDS HDS;
		typedef typename PH::Vertex_const_handle VH;
		typedef typename CGAL::Surface_mesh<typename PH::Traits::Point_3>::vertex_index _VD;
		typedef typename _VD::size_type v_size_type;
		using _FD = typename CGAL::Surface_mesh<typename PH::Traits::Point_3>::face_index;

		//create all vertices
		for (typename PH::Vertex_const_handle it = poly.vertices_begin(); it != poly.vertices_end(); ++it)
		{
			trimesh.add_vertex(it->point());
		}

		//create all faces
		std::unordered_map<VH, size_t> vhash;
		size_t k = 0;
		for (auto it = poly.vertices_begin(); it != poly.vertices_end(); ++it)
			vhash[it] = k++;

		for (typename PH::Face_const_iterator it = poly.facets_begin(); it != poly.facets_end(); ++it)
		{
			std::vector<_VD> face;
			auto f = it->facet_begin();
			while (true)
			{
				//auto idx = distance(poly.vertices_begin(), f->vertex());
				v_size_type idx = (v_size_type)vhash[f->vertex()];
				face.push_back(_VD(idx));
				f++;
				if (f == it->facet_begin())
					break;
			}
			trimesh.add_face(face);
		}

		if (triangulate)
			pmp::triangulate_faces(trimesh);
	}

	void triangulate_polygon_2(const POINTLIST_2& pts, Triangulate_2::CONSTRAINED_DELAUNAY_2* out)
	{
		out->clear();
		bool has_duplicate = pts.front() == pts.back();
		typedef Triangulate_2::CONSTRAINED_DELAUNAY_2::Vertex_handle VH;
		typedef Triangulate_2::CONSTRAINED_DELAUNAY_2::Face_handle FH;

		std::vector<VH> vhlist;
		for (size_t i = 0; i < pts.size() - has_duplicate; i++)
		{
			auto vh = out->insert(pts[i]);
			vh->info().index = (int)i;
			vhlist.push_back(vh);
		}

		//add constraint edges
		for (size_t i = 0; i < vhlist.size(); i++)
		{
			auto i_next = (i + 1) % vhlist.size();
			out->insert_constraint(vhlist[i], vhlist[i_next]);
		}

		//explore faces, mark them as in-domain or out-of-domain
		for (auto it = out->all_faces_begin(); it != out->all_faces_end(); ++it)
		{
			it->info().visited = false;
			it->set_in_domain(true);
		}

		//mark_domain(*out, out->infinite_face(), true, false);
		mark_domain_alternating(*out);

		//find a face not visitied yet, and start from that
		if (false)
		{
			std::queue<FH> candfaces;
			candfaces.push(out->infinite_face());
			while (!candfaces.empty())
			{
				auto f_cur = candfaces.front();
				candfaces.pop();

				//this face must be in unbounded domain
				f_cur->info().visited = true;
				f_cur->set_in_domain(false);

				//visit its neighbors, do not go over the constrained edges
				for (int i = 0; i < 3; i++)
				{
					//check the edge going from this face to its neighbor
					Triangulate_2::CONSTRAINED_DELAUNAY_2::Edge e(f_cur, i);
					if (out->is_constrained(e)) //constrained edge detected, do not go over it
						continue;

					//this neighbor is an out-of-domain face
					auto f_nb = f_cur->neighbor(i);
					if (!f_nb->info().visited)
						candfaces.push(f_nb);
				}
			}
		}
		//add one point as seed
		//note that this will modify the vertices, so we need to re-index the vertices
		//CGAL::refine_Delaunay_mesh_2(*out, seedpts.begin(), seedpts.end(), Triangulate_2::CRITERIA_CDT_2(0,0), false);

		//re-index the vertices
		//int k = 0;
		//for (auto it = out->all_vertices_begin(); it != out->all_vertices_end(); ++it)
		//{
		//	it->info() = k++;
		//}
	}

	void triangulate_polygon_2(const std::vector<POINTLIST_2>& ptslist, Triangulate_2::CONSTRAINED_DELAUNAY_2* out)
	{
		out->clear();
		typedef Triangulate_2::CONSTRAINED_DELAUNAY_2::Vertex_handle VH;
		typedef Triangulate_2::CONSTRAINED_DELAUNAY_2::Face_handle FH;

		//insert all polygons
		for (const auto& pts : ptslist)
		{
			bool has_duplicate = pts.front() == pts.back();
			std::vector<VH> vhlist;
			for (size_t i = 0; i < pts.size() - has_duplicate; i++)
			{
				auto vh = out->insert(pts[i]);
				vh->info().index = (int)i;
				vhlist.push_back(vh);
			}

			//add constraint edges
			for (size_t i = 0; i < vhlist.size(); i++)
			{
				auto i_next = (i + 1) % vhlist.size();
				out->insert_constraint(vhlist[i], vhlist[i_next]);
			}
		}

		//mark domain
		mark_domain_alternating(*out);
	}

	void optimize_triangulation(Triangulate_2::CONSTRAINED_DELAUNAY_2& cdt, double precision /*= 1e-6*/, int max_iter /*= 0*/)
	{
		//CGAL::lloyd_optimize_mesh_2(cdt,
		//	CGAL::parameters::convergence = precision,
		//	CGAL::parameters::freeze_bound = precision,
		//	CGAL::parameters::max_iteration_number = max_iter
		//);
		throw std::logic_error("this function is disabled due to compilation bug");
	}

	void refine_triangulation(Triangulate_2::CONSTRAINED_DELAUNAY_2& cdt, double max_segment_length)
	{
		Triangulate_2::CRITERIA_CDT_2 criteria(0.125, max_segment_length);
		CGAL::refine_Delaunay_mesh_2(cdt, criteria, true);
	}

	void make_polyhedron(const POINTLIST_3& pts, const POLYFACELIST& faces, POLYHEDRON_3& outpoly, bool verbose /*= false*/)
	{
		_make_polyhedron(pts, faces, outpoly, verbose);
	}

	void to_surface_mesh(const POLYHEDRON_3& poly, TRIMESH_3& trimesh, bool triangulate /*= false*/)
	{
		_to_surface_mesh<POLYHEDRON_3>(poly, trimesh, triangulate);
	}

	void compute_mean_value_coordinate_2(const POINTLIST_2& polygon_pts, const POINTLIST_2& query_pts, std::vector<std::vector<float_type>>& output)
	{
		//construct mean value coordinate object
		using MEAN_VALUE = CGAL::Barycentric_coordinates::Mean_value_2<KERNEL>;
		using MEAN_VALUE_COORD = CGAL::Barycentric_coordinates::Generalized_barycentric_coordinates_2<MEAN_VALUE, KERNEL>;
		MEAN_VALUE_COORD mvc_computer(polygon_pts.begin(), polygon_pts.end());
		output.resize(query_pts.size());
		for (auto& x : output)
		{
			x.clear();
			x.reserve(polygon_pts.size());
		}

		for (int i = 0; i < query_pts.size(); i++)
		{
			//CGAL::Barycentric_coordinates::mean_value_coordinates_2(
			//	polygon_pts, query_pts[i], std::back_inserter(output[i]));
			mvc_computer(query_pts[i], std::back_inserter(output[i]));
		}
	}

	_NS_UTILITY::POINT_3 compute_barycentric_coordinate(const TRI_3& tri, POINT_3 pt)
	{
		typedef CGAL::Barycentric_coordinates::Triangle_coordinates_2<KERNEL> BC_COMPUTE;
		std::array<double, 3> buf;

		auto plane = tri.supporting_plane();
		auto v1 = plane.to_2d(tri.vertex(0));
		auto v2 = plane.to_2d(tri.vertex(1));
		auto v3 = plane.to_2d(tri.vertex(2));
		BC_COMPUTE bc_coord(v1, v2, v3);
		auto q = plane.to_2d(pt);
		bc_coord(q, buf.begin());
		return POINT_3(buf[0], buf[1], buf[2]);
	}

	void compute_barycentric_coordinate(const TRI_3& tri, const POINTLIST_3& pts, POINTLIST_3& output)
	{
		typedef CGAL::Barycentric_coordinates::Triangle_coordinates_2<KERNEL> BC_COMPUTE;
		std::array<double, 3> buf;
		output.resize(pts.size());

		auto plane = tri.supporting_plane();
		auto v1 = plane.to_2d(tri.vertex(0));
		auto v2 = plane.to_2d(tri.vertex(1));
		auto v3 = plane.to_2d(tri.vertex(2));
		BC_COMPUTE bc_coord(v1, v2, v3);

		for (size_t i = 0; i < pts.size(); i++)
		{
			auto p = pts[i];
			auto q = plane.to_2d(p);
			bc_coord(q, buf.begin());
			output[i] = POINT_3(buf[0], buf[1], buf[2]);
		}
	}

	void re_label_all_vertices(Triangulate_2::CONSTRAINED_DELAUNAY_2& cdt)
	{
		int k = 0;
		for (auto it = cdt.finite_vertices_begin(); it != cdt.finite_vertices_end(); ++it)
			it->info().index = k++;
	}

	void mark_triangles_out_of_domain(Triangulate_2::CONSTRAINED_DELAUNAY_2* inout)
	{
		typedef Triangulate_2::CONSTRAINED_DELAUNAY_2::Vertex_handle VH;
		typedef Triangulate_2::CONSTRAINED_DELAUNAY_2::Face_handle FH;

		//explore faces, mark them as in-domain or out-of-domain
		for (auto it = inout->all_faces_begin(); it != inout->all_faces_end(); ++it)
		{
			it->info().visited = false;
			it->set_in_domain(true);
		}

		std::queue<FH> candfaces;
		candfaces.push(inout->infinite_face());
		while (!candfaces.empty())
		{
			auto f_cur = candfaces.front();
			candfaces.pop();

			//this face must be in unbounded domain
			f_cur->info().visited = true;
			f_cur->set_in_domain(false);

			//visit its neighbors, do not go over the constrained edges
			for (int i = 0; i < 3; i++)
			{
				//check the edge going from this face to its neighbor
				Triangulate_2::CONSTRAINED_DELAUNAY_2::Edge e(f_cur, i);
				if (inout->is_constrained(e)) //constrained edge detected, do not go over it
					continue;

				//this neighbor is an out-of-domain face
				auto f_nb = f_cur->neighbor(i);
				if (!f_nb->info().visited)
					candfaces.push(f_nb);
			}
		}
	}

	void mark_domain(Triangulate_2::CONSTRAINED_DELAUNAY_2& inout, Triangulate_2::CONSTRAINED_DELAUNAY_2::Face_handle seedface, bool is_seed_in_domain, bool reset_visited_flag)
	{
		typedef Triangulate_2::CONSTRAINED_DELAUNAY_2::Face_handle FH;
		std::queue<FH> candfaces;

		if (reset_visited_flag)
		{
			for (auto it = inout.all_faces_begin(); it != inout.all_faces_end(); ++it)
			{
				it->info().visited = false;
			}
		}

		candfaces.push(seedface);
		while (!candfaces.empty())
		{
			auto f_cur = candfaces.front();
			candfaces.pop();

			//this face must be in unbounded domain
			f_cur->info().visited = true;
			f_cur->set_in_domain(is_seed_in_domain);

			//visit its neighbors, do not go over the constrained edges
			for (int i = 0; i < 3; i++)
			{
				//check the edge going from this face to its neighbor
				Triangulate_2::CONSTRAINED_DELAUNAY_2::Edge e(f_cur, i);
				if (inout.is_constrained(e)) //constrained edge detected, do not go over it
					continue;

				//this neighbor is a face in the same domain
				auto f_nb = f_cur->neighbor(i);
				if (!f_nb->info().visited) //not visited yet
					candfaces.push(f_nb);
			}
		}
	}

	void mark_domain_alternating(Triangulate_2::CONSTRAINED_DELAUNAY_2& inout)
	{
		typedef Triangulate_2::CONSTRAINED_DELAUNAY_2::Face_handle FH;

		for (auto it = inout.all_faces_begin(); it != inout.all_faces_end(); ++it)
		{
			it->info().visited = false;
			it->info().nesting_level = 0;
			it->set_in_domain(true);
		}

		std::queue<FH> next_candfaces;
		next_candfaces.push(inout.infinite_face());
		int n_level = 0;
		std::queue<FH> candfaces;
		while (!next_candfaces.empty())
		{
			candfaces = next_candfaces;
			next_candfaces = std::queue<FH>();
			while (!candfaces.empty())
			{
				auto f_cur = candfaces.front();
				candfaces.pop();

				//if this face is already visited, ignore it
				if (f_cur->info().visited)
					continue;

				f_cur->info().visited = true;
				f_cur->info().nesting_level = n_level;
				//std::cout << "is finite=" << !inout.is_infinite(f_cur) << ", level=" << n_level << std::endl;
				//f_cur->set_in_domain(cur_domain);
				//std::cout << cur_domain << std::endl;

				//visit its neighbors, do not go over the constrained edges
				for (int i = 0; i < 3; i++)
				{
					//check the edge going from this face to its neighbor
					Triangulate_2::CONSTRAINED_DELAUNAY_2::Edge e(f_cur, i);

					auto f_nb = f_cur->neighbor(i);
					if (!f_nb->info().visited) //not visited yet
					{
						if (inout.is_constrained(e)) //crossing constrained edge, push to next level
							next_candfaces.push(f_nb);
						else candfaces.push(f_nb); //in this sub domain
					}
				}
			}
			n_level++;
		}

		//std::cout << "setting domain flag" << std::endl;
		for (auto it = inout.all_faces_begin(); it != inout.all_faces_end(); ++it)
		{
			if (it->info().nesting_level % 2 == 0)
			{
				//std::cout << "OUT:is finite=" << !inout.is_infinite(it) << ", level=" << it->info().nesting_level << std::endl;
				it->set_in_domain(false);
			}
			else
			{
				//std::cout << "IN:is finite=" << !inout.is_infinite(it) << ", level=" << it->info().nesting_level << std::endl;
				it->set_in_domain(true);
			}
		}

		//reset vertex index
		re_label_all_vertices(inout);

		//int n_face_in_domain = 0;
		//for (auto it = inout.finite_faces_begin(); it != inout.finite_faces_end(); ++it)
		//{
		//	if (it->is_in_domain())
		//		n_face_in_domain++;
		//}

		//int n_vertices = 0;
		//for (auto it = inout.finite_vertices_begin(); it != inout.finite_vertices_end(); ++it)
		//	n_vertices++;
		//std::cout << "we have " << n_face_in_domain << " faces in domain" << std::endl;
		//std::cout << "vertices = " << n_vertices << std::endl;
	}

	void compute_vertex_normals(const TRIMESH_3& trimesh, std::vector<VEC_3>& output)
	{
		namespace pmp = CGAL::Polygon_mesh_processing;
		using VD = TRIMESH_3::Vertex_index;

		auto n_vert = trimesh.number_of_vertices();

		std::vector<VEC_3> normals(n_vert);
		auto vd2idx = boost::make_function_property_map<VD>([](VD x) {return int(x); });
		auto vertex_normal_map = boost::make_iterator_property_map(normals.begin(), vd2idx);
		pmp::compute_vertex_normals(trimesh, vertex_normal_map,
			pmp::parameters::vertex_point_map(trimesh.points()).geom_traits(KERNEL()));

		output = std::move(normals);
	}

	void compute_face_normals(const TRIMESH_3& trimesh, std::vector<VEC_3>& output)
	{
		namespace pmp = CGAL::Polygon_mesh_processing;
		using FD = TRIMESH_3::Face_index;

		auto n_face = trimesh.number_of_faces();

		std::vector<VEC_3> normals(n_face);
		auto fd2idx = boost::make_function_property_map<FD>([](FD x) {return int(x); });
		auto face_normal_map = boost::make_iterator_property_map(normals.begin(), fd2idx);
		pmp::compute_face_normals(trimesh, face_normal_map,
			pmp::parameters::vertex_point_map(trimesh.points()).geom_traits(KERNEL()));

		output = std::move(normals);
	}

};
