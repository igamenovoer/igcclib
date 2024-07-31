//implementations
#include <memory>
#include <vector>
#include <igcclib/geometry/MeshSearcher.hpp>
#include <igcclib/core/igcclib_eigen.hpp>
#include <igcclib/geometry/igcclib_cgal_eigen.hpp>
#include <CGAL/Barycentric_coordinates_2/triangle_coordinates_2.h>

namespace _NS_UTILITY
{
	inline void MeshSearcher::update_query_structure()
	{
		//update trilist
		const auto& v = m_mesh->get_vertices();
		const auto& f = m_mesh->get_faces();

		auto pts = to_point_list_3(v);
		m_trilist->resize(f.rows());
		for (Eigen::Index i = 0; i < f.rows(); i++)
		{
			int k1 = f(i, 0);
			int k2 = f(i, 1);
			int k3 = f(i, 2);
			TRI_3 tri(pts[k1], pts[k2], pts[k3]);
			m_trilist->at(i) = tri;
		}

		//create aabb tree
		m_aabb_tree = std::make_shared<FACETREE_3>(m_trilist->begin(), m_trilist->end());
		m_aabb_tree->accelerate_distance_queries();
	}

	inline void MeshSearcher::set_mesh(const TriangularMesh& mesh, bool update /*= true*/, bool make_copy/*= false*/)
	{
		if (make_copy)
		{
			TriangularMesh _mesh = mesh;
			m_mesh = std::make_shared<TriangularMesh>(_mesh);
		}
		else
		{
			m_mesh.reset(const_cast<TriangularMesh*>(&mesh), [](const void* p) {});
		}

		
		if (update)
			update_query_structure();
	}

	inline void MeshSearcher::find_closest_point(const fMATRIX& pts, fMATRIX* out_nnpts, iVECTOR* out_idxtri /*= 0*/, fMATRIX* out_bc_pts /*=0*/)
	{
		assert_throw(m_aabb_tree != nullptr, "cannot query before aabb tree is built");
		auto ptslist = to_point_list_3(pts);

		//resulting nearest points
		std::vector<POINT_3> nnpts(ptslist.size());

		//result: index of triangle where nearest points lie
		std::vector<int_type> idxtri(ptslist.size());

		for (int i = 0; i < ptslist.size(); i++)
		{
			auto p = ptslist[i];
			auto res = m_aabb_tree->closest_point_and_primitive(p);
			nnpts[i] = res.first;
			idxtri[i] = res.second - m_trilist->begin();
		}

		//compute barycentric coordinate
		// typedef CGAL::Barycentric_coordinates::Triangle_coordinates_2<KERNEL> BC_COMPUTE;
		std::vector<POINT_3> bcpts(ptslist.size());
		for (int i = 0; i < ptslist.size(); i++)
		{
			auto& tri = m_trilist->at(idxtri[i]);
			auto& p = nnpts[i];
			auto bc = compute_barycentric_coordinate(tri, p);
			bcpts[i] = bc;
		}

		//invalidate the hit if barycentric coordinate is invalid
		if (out_bc_pts) {
			for (size_t i = 0; i < ptslist.size(); i++) {
				auto bc = bcpts[i];
				if (std::isnan(bc.x()) || std::isnan(bc.y()) || std::isnan(bc.z())) {
					//invalidate the hit
					idxtri[i] = -1;
				}
			}
		}

		if (out_nnpts)
			*out_nnpts = to_matrix(nnpts);
		if (out_idxtri)
			*out_idxtri = to_vector(idxtri);
		if (out_bc_pts)
			*out_bc_pts = to_matrix(bcpts);
	}

	inline void MeshSearcher::find_closest_point(
		const POINT_3& p, POINT_3* out_closest_point, 
		int_type* out_idxtri, POINT_3* out_barycentric /*= nullptr*/)
	{
		assert_throw(m_aabb_tree != nullptr, "cannot query before aabb tree is built");
		auto res = m_aabb_tree->closest_point_and_primitive(p);
		POINT_3 p_close = res.first;
		int_type idxtri = res.second - m_trilist->begin();

		if (out_closest_point) *out_closest_point = p_close;
		if (out_idxtri) *out_idxtri = idxtri;

		if (out_barycentric) {
			*out_barycentric = compute_barycentric_coordinate(m_trilist->at(idxtri), p_close);
			if (std::isnan(out_barycentric->x()) || std::isnan(out_barycentric->y()) || std::isnan(out_barycentric->z())) {
				//invalidate the hit
				if (out_idxtri)
					*out_idxtri = -1;
			}
		}
	}

	inline void MeshSearcher::intersect_with_ray_first(const fMATRIX& p0, const fMATRIX& dirs,
		fMATRIX* out_hitpts, iVECTOR* out_idxtri, fMATRIX* out_bcpts)
	{
		assert_throw(p0.rows() == dirs.rows(), "number of p0 does not match number of dirs");
		fMATRIX hitpts(p0.rows(),3);
		iVECTOR idxtri(p0.rows());
		fMATRIX bcpts(p0.rows(), 3);
		for (int i = 0; i < p0.rows(); i++)
		{
			POINT_3 p(p0(i, 0), p0(i, 1), p0(i, 2));
			VEC_3 d(dirs(i, 0), dirs(i, 1), dirs(i, 2));
			RAY_3 ray(p, p + d);
			POINT_3 res_point(0, 0, 0);
			int res_idxtri;
			intersect_with_ray_first(ray, &res_point, &res_idxtri);
			hitpts(i, 0) = res_point.x();
			hitpts(i, 1) = res_point.y();
			hitpts(i, 2) = res_point.z();
			idxtri(i) = res_idxtri;

			//compute barycentric coordinate
			POINT_3 res_bc(0, 0, 0);
			if (res_idxtri >= 0)
				res_bc = compute_barycentric_coordinate(m_trilist->at(res_idxtri), res_point);
			bcpts(i, 0) = res_bc.x();
			bcpts(i, 1) = res_bc.y();
			bcpts(i, 2) = res_bc.z();
		}

		//invalidate the hit if barycentric coordinate cannot be determined
		if (out_bcpts) {
			for (Eigen::Index i = 0; i < bcpts.rows(); i++) {
				auto r = bcpts.row(i);
				if (r.array().isNaN().any())
					idxtri[i] = -1; //invalidate the hit
			}
		}

		if (out_hitpts)
			*out_hitpts = hitpts;
		if (out_idxtri)
			*out_idxtri = idxtri;
		if (out_bcpts)
			*out_bcpts = bcpts;
	}

	inline void MeshSearcher::intersect_with_ray_first(const RAY_3& ray, 
		POINT_3* out_hitpoint, int_type* out_idxtri, POINT_3* out_barycentric)
	{
		auto res = m_aabb_tree->first_intersection(ray);

		POINT_3 res_point(0, 0, 0);
		SEGMENT_3 res_seg;
		int_type res_idxtri = -1;
		if (res)
		{
			//std::cout << "found intersection" << std::endl;
			if (!CGAL::assign(res_point, res->first))
			{
				CGAL::assign(res_seg, res->first);
				res_point = res_seg.source();
			}
			res_idxtri = res->second - m_trilist->begin();
		}
		if (out_hitpoint)
			*out_hitpoint = res_point;
		if (out_idxtri)
			*out_idxtri = res_idxtri;

		if (out_barycentric && res)
		{
			*out_barycentric = compute_barycentric_coordinate(m_trilist->at(res_idxtri), res_point);

			//check validity
			if (std::isnan(out_barycentric->x()) || std::isnan(out_barycentric->y()) || std::isnan(out_barycentric->z()))
			{
				//invalidate the hit
				if (out_idxtri)
					*out_idxtri = -1;
			}
		}
	}

	inline void MeshSearcher::init_with_triangular_mesh(MeshSearcher& output, const TriangularMesh& mesh)
	{
		MeshSearcher x;
		x.set_mesh(mesh, true);
		output = x;
	}

	//inline MeshSearcher MeshSearcher::init_with_triangular_mesh(const TriangularMesh& mesh)
	//{
	//	MeshSearcher output;
	//	init_with_triangular_mesh(output, mesh);
	//	return output;
	//}

	inline void MeshSearcher::init_with_vertex_face(MeshSearcher& output,
		const fMATRIX& vertices, const iMATRIX& faces)
	{
		MeshSearcher _output;
		auto trimesh = std::make_shared<TriangularMesh>();
		TriangularMesh::init_with_vertex_face(*trimesh, vertices, faces);
		_output.m_mesh = trimesh;
		_output.update_query_structure();
		output = _output;
	}

	inline void MeshSearcher::find_closest_point(const fVECTOR_3& p, fVECTOR_3* out_closest_point,
		int_type* out_idxtri, fVECTOR_3* out_barycentric)
	{
		POINT_3 _p(p[0], p[1], p[2]);
		POINT_3 _q, _bc;
		find_closest_point(_p, &_q, out_idxtri, &_bc);

		if (out_closest_point)
			*out_closest_point = fVECTOR_3(_q.x(), _q.y(), _q.z());
		
		if (out_barycentric)
			*out_barycentric = fVECTOR_3(_bc.x(), _bc.y(), _bc.z());
	}
};