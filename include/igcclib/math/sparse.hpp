#pragma once
#include <igcclib/igcclib_master.hpp>
#include <igcclib/core/igcclib_eigen.hpp>

namespace _NS_UTILITY {
	template<typename T>
	MATRIX_t<T> mldivide(const SP_MATRIX_col_t<T>& A, const MATRIX_t<T>& b) {
		Eigen::SparseLU<SP_MATRIX_col_t<T>> solver;
		solver.compute(A.transpose() * A);

		MATRIX_col_t<T> _b = b;
		MATRIX_col_t<T> sol = solver.solve(A.transpose() * _b);
		return sol;
	}

	template<typename T>
	MATRIX_t<T> mldivide_lu(const SP_MATRIX_t<T>& A, const MATRIX_t<T>& b) {
		Eigen::SparseLU<SP_MATRIX_col_t<T>> solver;
		SP_MATRIX_col_t<T> ata = A.transpose() * A;
		solver.compute(ata);

		MATRIX_t<T> sol = solver.solve(A.transpose() * b);
		return sol;
	}

	template<typename T>
	MATRIX_t<T> mldivide_llt(const SP_MATRIX_t<T>& A, const MATRIX_t<T>& b) {
		Eigen::SimplicialLLT<SP_MATRIX_col_t<T>> solver;
		SP_MATRIX_col_t<T> ata = A.transpose() * A;
		solver.compute(ata);

		MATRIX_t<T> sol = solver.solve(A.transpose() * b);
		return sol;
	}
};