#pragma once
//CGAL and Eigen interop

#include <igcclib/core/igcclib_eigen_def.hpp>
#include <igcclib/geometry/igcclib_cgal_def.hpp>

namespace _NS_UTILITY
{
	//convert nx2 matrix to n point list
	template<typename T>
	POINTLIST_2 to_point_list_2(const MATRIX_t<T>& pts)
	{
		assert(pts.cols() >= 2);
		POINTLIST_2 ptslist;
		for (size_t i = 0; i < pts.rows(); i++)
		{
			POINT_2 p(pts(i, 0), pts(i, 1));
			ptslist.push_back(p);
		}
		return ptslist;
	}

	//convert nx3 matrix to n point list
	template<typename T>
	POINTLIST_3 to_point_list_3(const MATRIX_t<T>& pts)
	{
		assert(pts.cols() >= 3);
		POINTLIST_3 ptslist;
		for (Eigen::Index i = 0; i < pts.rows(); i++)
		{
			POINT_3 p(pts(i, 0), pts(i, 1), pts(i, 2));
			ptslist.push_back(p);
		}
		return ptslist;
	}

	//convert pointlist to matrix
	inline fMATRIX to_matrix(const POINTLIST_3& pts)
	{
		fMATRIX mat(pts.size(), 3);
		for (size_t i = 0; i < pts.size(); i++)
		{
			mat(i, 0) = pts[i].x();
			mat(i, 1) = pts[i].y();
			mat(i, 2) = pts[i].z();
		}
		return mat;
	}

	inline fMATRIX to_matrix(const POINTLIST_2& pts)
	{
		fMATRIX mat(pts.size(), 2);
		for (size_t i = 0; i < pts.size(); i++)
		{
			mat(i, 0) = pts[i].x();
			mat(i, 1) = pts[i].y();
		}
		return mat;
	}

	//convert facelist to matrix
	inline iMATRIX to_matrix(const POLYFACELIST& faces)
	{
		auto ndim = faces[0].size();
		iMATRIX mat(faces.size(), ndim);
		for (int i = 0; i < faces.size(); i++)
		{
			for (int k = 0; k < faces[i].size(); k++)
			{
				mat(i, k) = (iMATRIX::value_type)faces[i][k];
			}
		}
		return mat;
	}

	inline fMATRIX to_matrix(const std::vector<VEC_3>& data) {
		fMATRIX output(data.size(), 3);
		for (size_t i = 0; i < data.size(); i++) {
			output(i, 0) = data[i].x();
			output(i, 1) = data[i].y();
			output(i, 2) = data[i].z();
		}
		return output;
	}


	//to face list
	template<typename T>
	POLYFACELIST to_face_list(const MATRIX_t<T>& faces)
	{
		using T = POLYFACE::value_type;
		POLYFACELIST facelist;
		for (int i = 0; i < faces.rows(); i++)
		{
			POLYFACE f(faces.cols());
			for (int j = 0; j < faces.cols(); j++)
			{
				f[j] = T(faces(i, j));
			}
			facelist.push_back(f);
		}
		return facelist;
	}

	//create mesh
	template<typename _POINT_2, typename T1, typename T2>
	void make_trimesh_2(
		const MATRIX_t<T1>& pts,
		const MATRIX_t<T2>& faces, 
		CGAL::Surface_mesh<_POINT_2>& outmesh)
	{
		typedef CGAL::Surface_mesh<_POINT_2> _TMESH;
		typedef typename _TMESH::vertex_index VI;
		typedef typename _TMESH::face_index FI;

		//create mesh
		outmesh.clear();

		assert(pts.cols() >= 2);
		for (int i = 0; i < pts.rows(); i++)
		{
			_POINT_2 p(pts(i, 0), pts(i, 1));
			outmesh.add_vertex(p);
		}
		for (int i = 0; i < faces.rows(); i++)
		{
			//POLYFACE f(faces.cols());
			std::vector<VI> f(faces.cols());
			for (int j = 0; j < faces.cols(); j++)
				f[j] = VI(faces(i, j));
			outmesh.add_face(f);
		}
	}

	//create mesh
	template<typename _POINT_3, typename T1, typename T2>
	void make_trimesh_3(
		const MATRIX_t<T1>& pts,
		const MATRIX_t<T2>& faces,
		CGAL::Surface_mesh<_POINT_3>& outmesh)
	{
		typedef CGAL::Surface_mesh<_POINT_3> _TMESH;
		typedef typename _TMESH::vertex_index VI;
		typedef typename _TMESH::face_index FI;

		//create mesh
		outmesh.clear();

		assert(pts.cols() >= 3);
		for (int i = 0; i < pts.rows(); i++)
		{
			_POINT_3 p(pts(i, 0), pts(i, 1), pts(i, 2));
			outmesh.add_vertex(p);
		}
		for (int i = 0; i < faces.rows(); i++)
		{
			std::vector<VI> f(faces.cols());
			//POLYFACE f(faces.cols());
			for (int j = 0; j < faces.cols(); j++)
				f[j] = VI(faces(i, j));
			outmesh.add_face(f);
		}
	}

	//convert triangulation to point and face
	template <typename FLOAT_T, typename INT_T>
	void read_vertex_face(const Triangulate_2::CONSTRAINED_DELAUNAY_2& cdt,
		MATRIX_t<FLOAT_T>* out_vertices, MATRIX_t<INT_T>* out_faces)
	{
		int n_finite_verts =0;
		int n_finite_faces =0;
		for (auto it = cdt.finite_vertices_begin(); it != cdt.finite_vertices_end(); ++it)
			n_finite_verts++;

		for (auto it = cdt.finite_faces_begin(); it != cdt.finite_faces_end(); ++it)
			if(it->is_in_domain())
				n_finite_faces++;

		MATRIX_t<FLOAT_T> vertices(n_finite_verts, 2);
		MATRIX_t<INT_T> faces(n_finite_faces, 3);
		//std::cout << "reading " << n_finite_faces << " faces" << std::endl;

		int k = 0;
		for (auto it = cdt.finite_vertices_begin(); it != cdt.finite_vertices_end(); ++it)
		{
			auto p = it->point();
			vertices(k, 0) = p.x();
			vertices(k, 1) = p.y();
			k++;
		}

		k = 0;
		for (auto it = cdt.finite_faces_begin(); it != cdt.finite_faces_end(); ++it)
		{
			//faces(k, 0) = it->vertex(0)->info();
			//faces(k, 1) = it->vertex(1)->info();
			//faces(k, 2) = it->vertex(2)->info();
			//k++;
			if (it->is_in_domain())
			{
				faces(k, 0) = it->vertex(0)->info().index;
				faces(k, 1) = it->vertex(1)->info().index;
				faces(k, 2) = it->vertex(2)->info().index;
				k++;
			}
		}

		if (out_vertices)
			*out_vertices = vertices;
		if (out_faces)
			*out_faces = faces;
	}

	template<typename FLOAT_T, typename INT_T>
	void read_vertex_face(const TRIMESH_3& trimesh, 
		MATRIX_t<FLOAT_T>* out_vertices, MATRIX_t<INT_T>* out_faces)
	{
		POINTLIST_3 pts;
		POLYFACELIST faces;
		read_vertex_face(trimesh, &pts, &faces);

		if (out_vertices)
			*out_vertices = to_matrix(pts);
		if (out_faces)
			*out_faces = to_matrix(faces);
	}

	//pts is the polygon, with or without end point duplication
	template<typename FLOAT_T>
	void triangulate_polygon_2(const MATRIX_t<FLOAT_T>& pts, 
		Triangulate_2::CONSTRAINED_DELAUNAY_2* out)
	{
		POINTLIST_2 ptslist = to_point_list_2(pts);
		triangulate_polygon_2(ptslist, out);
	}

	/// <summary>
	/// compute mean value coordinates in 2d
	/// </summary>
	/// <param name="polygon_pts">the polygon point sequence, the first and last point DO NOT have to be the same,
	/// as the polygon will be automatically closed</param>
	/// <param name="query_pts">points against which mean value coordinates are computed</param>
	/// <param name="output">output mean value coordinates for each point, each row is a point</param>
	template<typename FLOAT_T>
	inline void compute_mean_value_coordinate_2(
		const MATRIX_t<FLOAT_T>& polygon_pts, const MATRIX_t<FLOAT_T>& query_pts,
		MATRIX_t<FLOAT_T>& output)
	{
		assert_throw(polygon_pts.cols() == 2, "only accepts 2d points");
		assert_throw(query_pts.cols() == 2, "only accepts 2d points");

		//construct mean value coordinate object
		using MEAN_VALUE = CGAL::Barycentric_coordinates::Mean_value_2<KERNEL>;
		using MEAN_VALUE_COORD = CGAL::Barycentric_coordinates::Generalized_barycentric_coordinates_2<MEAN_VALUE, KERNEL>;

		auto _polygon_pts = to_point_list_2(polygon_pts);
		MEAN_VALUE_COORD mvc_computer(_polygon_pts.begin(), _polygon_pts.end());
		output.resize(query_pts.rows(), polygon_pts.rows());

		std::vector<FLOAT_T> _out;
		_out.reserve(polygon_pts.rows());

		for (int i = 0; i < query_pts.rows(); ++i)
		{
			_out.clear();
			POINT_2 p(query_pts(i, 0), query_pts(i, 1));
			mvc_computer(p, std::back_inserter(_out));
			for (int k = 0; k < _out.size(); k++)
				output(i, k) = _out[k];
		}
	}

	template<typename T>
	void compute_vertex_normals(const TRIMESH_3& trimesh, MATRIX_t<T>& output) {
		std::vector<VEC_3> normals;
		compute_vertex_normals(trimesh, normals);
		output = to_matrix(normals).template cast<T>();
	}

	template<typename T>
	void compute_face_normals(const TRIMESH_3& trimesh, MATRIX_t<T>& output) {
		std::vector<VEC_3> normals;
		compute_face_normals(trimesh, normals);
		output = to_matrix(normals).template cast<T>();
	}
};