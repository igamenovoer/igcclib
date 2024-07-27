#pragma once
#include <igcclib/visualization/magnum/igcclib_magnum_def.hpp>
#include <igcclib/core/igcclib_eigen_def.hpp>

#include <vector>
#include <iostream>
#include <fstream>

namespace _NS_UTILITY
{
	namespace MagnumProc {
		/// <summary>
		/// convert vertices in Eigen format (one vertex per row) into Magnum vertex list
		/// </summary>
		/// <param name="vertices">vertices in matrix, one vertex per row</param>
		/// <param name="output">a list of Magnum vertices</param>
		inline void to_vertex_data(const fMATRIX& vertices, std::vector<VertexData_P_3>& output);

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
			std::vector<VertexData_PT_3>& output);

		inline void to_vertex_data(
			const fMATRIX& vertices, const iMATRIX& faces,
			const fMATRIX& uv, const iMATRIX& uv_faces,
			const fMATRIX& normals, const iMATRIX& normal_faces,
			std::vector<VertexData_PTN_3>& output);

		inline void to_vertex_data(
			const fMATRIX& vertices, const iMATRIX& faces,
			const fMATRIX& normals, const iMATRIX& normal_faces,
			std::vector<VertexData_PN_3>& output);

		/// <summary>
		/// convert vertices with texture coordinates in Eigen format to Magnum format
		/// </summary>
		/// <param name="vertices">geometric vertices, one vertex per row</param>
		/// <param name="uv">texture coordinate for each vertex, one uv per row</param>
		/// <param name="output">the Magnum vertices</param>
		inline void to_vertex_data(const MATRIX_d& vertices, const MATRIX_d& uv, std::vector<VertexData_PT_3>& output);

		/** \brief convert Magnum matrix to eigen matrix wrapper */
		template<typename T, size_t DIM>
		Eigen::Map<Eigen::Matrix<T,DIM,DIM>> to_eigen_map_colmajor(const Magnum::Math::Matrix<DIM, T>& mat) {
			return Eigen::Map<Eigen::Matrix<T, DIM, DIM>>(const_cast<T*>(mat.data()));
		}

		/** \brief convert Magnum vector to eigen vector wrapper */
		template<typename T, size_t DIM>
		Eigen::Map<Eigen::Matrix<T, DIM, 1>> to_eigen_map_colmajor(const Magnum::Math::Vector<DIM, T>& vec) {
			return Eigen::Map<Eigen::Matrix<T, DIM, 1>>(const_cast<T*>(vec.data()));
		}

		template<typename T, size_t DIM>
		void save_magnum_matrix(const Magnum::Math::Matrix<DIM, T>& mat, const std::string& filename);

		template<typename T, size_t DIM>
		void save_magnum_vector(const Magnum::Math::Vector<DIM, T>& vec, const std::string& filename);
	}
};

/** \brief print magnum matrix */
template<typename T, size_t N>
std::ostream& operator << (std::ostream& os, const Magnum::Math::Matrix<N, T>& mat) {
	using _NS_UTILITY::MagnumProc::to_eigen_map_colmajor;
	using MatType = typename decltype(to_eigen_map_colmajor(mat))::PlainMatrix;
	MatType emat = to_eigen_map_colmajor(mat);
	os << emat;
	return os;
}

/** \brief print magnum vector */
template<typename T, size_t N>
std::ostream& operator << (std::ostream& os, const Magnum::Math::Vector<N, T>& mat) {
	using _NS_UTILITY::MagnumProc::to_eigen_map_colmajor;
	using MatType = typename decltype(to_eigen_map_colmajor(mat))::PlainMatrix;
	MatType emat = to_eigen_map_colmajor(mat);
	os << emat;
	return os;
}

// ================================ implementation =================================
namespace _NS_UTILITY
{
	namespace MagnumProc {

		template<typename T, size_t DIM>
		void save_magnum_matrix(const Magnum::Math::Matrix<DIM, T>& mat, const std::string& filename)
		{
			std::ofstream outfile(filename);
			assert_throw(outfile.is_open(), "cannot open file");
			outfile << mat;
			outfile.close();
		}

		template<typename T, size_t DIM>
		void save_magnum_vector(const Magnum::Math::Vector<DIM, T>& vec, const std::string& filename) {
			std::ofstream outfile(filename);
			assert_throw(outfile.is_open(), "cannot open file");
			outfile << vec;
			outfile.close();
		}

		/// <summary>
		/// convert vertices in Eigen format (one vertex per row) into Magnum vertex list
		/// </summary>
		/// <param name="vertices">vertices in matrix, one vertex per row</param>
		/// <param name="output">a list of Magnum vertices</param>
		void to_vertex_data(const fMATRIX& vertices, std::vector<VertexData_P_3>& output)
		{
			output.resize(vertices.size());
			for (int i = 0; i < vertices.rows(); i++)
			{
				output[i].position = { float(vertices(i,0)), float(vertices(i,1)), float(vertices(i,2)) };
			}
		}

		void to_vertex_data(
			const fMATRIX& vertices, const iMATRIX& faces,
			const fMATRIX& uv, const iMATRIX& uv_faces,
			std::vector<VertexData_PT_3>& output)
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

		void to_vertex_data(
			const fMATRIX& vertices, const iMATRIX& faces,
			const fMATRIX& uv, const iMATRIX& uv_faces,
			const fMATRIX& normals, const iMATRIX& normal_faces,
			std::vector<VertexData_PTN_3>& output) {

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

		void to_vertex_data(
			const fMATRIX& vertices, const iMATRIX& faces,
			const fMATRIX& normals, const iMATRIX& normal_faces,
			std::vector<VertexData_PN_3>& output) {

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

		void to_vertex_data(const MATRIX_d& vertices, const MATRIX_d& uv, std::vector<VertexData_PT_3>& output)
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
	}
};