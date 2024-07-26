#pragma once
#include "igcclib_magnum_def.hpp"
#include <igcclib/core/igcclib_eigen_def.hpp>

#include <vector>

namespace _NS_UTILITY
{
	/// <summary>
	/// convert vertices in Eigen format (one vertex per row) into Magnum vertex list
	/// </summary>
	/// <param name="vertices">vertices in matrix, one vertex per row</param>
	/// <param name="output">a list of Magnum vertices</param>
	inline void to_vertex_data(const fMATRIX& vertices, std::vector<MagnumProc::VertexData_P_3>& output)
	{
		output.resize(vertices.size());
		for (int i = 0; i < vertices.rows(); i++)
		{
			output[i].position = { float(vertices(i,0)), float(vertices(i,1)), float(vertices(i,2)) };
		}
	}

	/// <summary>
	/// convert a mesh with texture coordinates into a list of split vertices in Magnum format.
	/// The input mesh will be split such that triangles are disconnected from each other
	/// </summary>
	/// <param name="vertices">vertex matrix, each row is a vertex</param>
	/// <param name="faces">face array, each row is a face</param>
	/// <param name="uv">uv vertices, each row is a vertex</param>
	/// <param name="uv_faces">uv faces, each row is a face</param>
	/// <param name="output">Magnum vertices, each vertex has a unique texture coordinate</param>
	inline void to_vertex_data(
		const fMATRIX& vertices, const iMATRIX& faces, 
		const fMATRIX& uv, const iMATRIX& uv_faces,
		std::vector<MagnumProc::VertexData_PT_3>& output)
	{
		assert_throw(vertices.cols() == 3, "input vertices must be 3D");

		int n_face = faces.rows();
		output.resize(3 * n_face);
		for (int i = 0; i < n_face; i++)
		{
			auto f_geom = faces.row(i);
			auto f_tex = uv_faces.row(i);
			for (int k = 0; k < 3; k++)
			{
				auto v_geom = vertices.row(f_geom(k));
				auto v_tex = uv.row(f_tex(k));
				auto& x = output[i * 3 + k];
				x.position = { float(v_geom(0)), float(v_geom(1)), float(v_geom(2)) };
				x.texcoord = { float(v_tex(0)), float(v_tex(1)) };
			}
		}
	}

	inline void to_vertex_data(
		const fMATRIX& vertices, const iMATRIX& faces,
		const fMATRIX& uv, const iMATRIX& uv_faces,
		const fMATRIX& normals, const iMATRIX& normal_faces,
		std::vector<MagnumProc::VertexData_PTN_3>& output) {

		assert_throw(vertices.cols() == 3, "input vertices must be 3D");

		int n_face = faces.rows();
		output.resize(3 * n_face);
		for (int i = 0; i < n_face; i++)
		{
			auto f_geom = faces.row(i);
			auto f_tex = uv_faces.row(i);
			auto f_normal = normal_faces.row(i);
			for (int k = 0; k < 3; k++)
			{
				auto v_geom = vertices.row(f_geom(k));
				auto v_tex = uv.row(f_tex(k));
				auto v_normal = normals.row(f_normal(k));

				auto& x = output[i * 3 + k];
				x.position = { float(v_geom(0)), float(v_geom(1)), float(v_geom(2)) };
				x.texcoord = { float(v_tex(0)), float(v_tex(1)) };
				x.normal = { float(v_normal(0)), float(v_normal(1)), float(v_normal(2)) };
			}
		}
	}

	inline void to_vertex_data(
		const fMATRIX& vertices, const iMATRIX& faces,
		const fMATRIX& normals, const iMATRIX& normal_faces,
		std::vector<MagnumProc::VertexData_PN_3>& output) {

		assert_throw(vertices.cols() == 3, "input vertices must be 3D");

		int n_face = faces.rows();
		output.resize(3 * n_face);
		for (int i = 0; i < n_face; i++)
		{
			auto f_geom = faces.row(i);
			auto f_normal = normal_faces.row(i);
			for (int k = 0; k < 3; k++)
			{
				auto v_geom = vertices.row(f_geom(k));
				auto v_normal = normals.row(f_normal(k));

				auto& x = output[i * 3 + k];
				x.position = { float(v_geom(0)), float(v_geom(1)), float(v_geom(2)) };
				x.normal = { float(v_normal(0)), float(v_normal(1)), float(v_normal(2)) };
			}
		}
	}

	/// <summary>
	/// convert vertices with texture coordinates in Eigen format to Magnum format
	/// </summary>
	/// <param name="vertices">geometric vertices, one vertex per row</param>
	/// <param name="uv">texture coordinate for each vertex, one uv per row</param>
	/// <param name="output">the Magnum vertices</param>
	inline void to_vertex_data(const MATRIX_d& vertices, const MATRIX_d& uv, std::vector<MagnumProc::VertexData_PT_3>& output)
	{
		assert_throw(vertices.rows() == uv.rows(), "number of vertices does not match with that of uv");
		int n_vert = vertices.rows();
		output.resize(n_vert);
		for (int i = 0; i < n_vert; i++)
		{
			auto& x = output[i];
			x.position = { float(vertices(i,0)), float(vertices(i,1)), float(vertices(i,2)) };
			x.texcoord = { float(uv(i,0)), float(uv(i,1)) };
		}
	}
};