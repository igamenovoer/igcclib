#pragma once

#include <igcclib/visualization/soft_renderer/igcclib_softlit.hpp>
#include <igcclib/geometry/TriangularMesh.hpp>

namespace _NS_UTILITY {
	
	/**
	* \brief convert TriangularMesh to softlit primitive
	*
	* \param output the output softlit primitive
	* \param create_texture whether to create the texture object if the triangular mesh has a texture.
	If false, then the texture is not created, you need to use output.addTexture() to load the texture.
	* \param tmesh the triangular mesh
	*/
	void to_softlit_primitive(softlit::Primitive& output, const TriangularMesh& tmesh, bool create_texture = true);

	template<typename glm_mat_t, glm::length_t glm_row, glm::length_t glm_col, typename eigen_mat_t>
	void to_matrix(const glm::mat<glm_col, glm_row, glm_mat_t>& src, MATRIX_t<eigen_mat_t>& dst) {
		dst.resize(glm_row, glm_col);

		//src is column major
		auto src_data = &src[0].x;
		dst = Eigen::Map<const MATRIX_t<glm_mat_t>>(src_data, glm_col, glm_row)
			.transpose().template cast<eigen_mat_t>();
	}

	template<typename glm_mat_t, glm::length_t glm_row, glm::length_t glm_col, typename eigen_mat_t>
	void to_matrix(const glm::mat<glm_col, glm_row, glm_mat_t>& src, MATRIX_xt<eigen_mat_t, glm_row, glm_col>& dst) {
		//src is column major
		auto src_data = &src[0].x;
		dst = Eigen::Map<const MATRIX_t<glm_mat_t>>(src_data, glm_col, glm_row)
			.transpose().template cast<eigen_mat_t>();
	}

	template<typename glm_mat_t, glm::length_t glm_row, glm::length_t glm_col, typename eigen_mat_t>
	void to_matrix(const MATRIX_t<eigen_mat_t>& src, glm::mat<glm_col, glm_row, glm_mat_t>& dst) {
		assert_throw(src.rows() == glm_row && src.cols() == glm_col, "matrix size mismatch");

		//dst is column major
		auto dst_data = &dst[0].x;
		Eigen::Map<MATRIX_t<glm_mat_t>>(dst_data, glm_col, glm_row) = src.transpose().template cast<glm_mat_t>();
	}

	template<typename glm_mat_t, glm::length_t glm_row, glm::length_t glm_col, typename eigen_mat_t>
	void to_matrix(const MATRIX_xt<eigen_mat_t, glm_row, glm_col>& src, glm::mat<glm_col, glm_row, glm_mat_t>& dst) {
		//dst is column major
		auto dst_data = &dst[0].x;
		Eigen::Map<MATRIX_t<glm_mat_t>>(dst_data, glm_col, glm_row) = src.transpose().template cast<glm_mat_t>();
	}
}