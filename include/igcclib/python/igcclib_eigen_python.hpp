#pragma once
#include <iostream>
#include <string>

#include <igcclib/core/igcclib_eigen.hpp>
#include "igcclib_boost_python.hpp"
#include "util_python.h"

namespace _NS_UTILITY
{
	//convert to eigen matrix
	//if the ndarray is 1d, the resulting matrix is 1xn row vector
	template<typename T>
	void to_matrix(const NDARRAY& arr, MATRIX_t<T>& output)
	{
		NDARRAY arr_double = get_ndarray_as<T>(arr);
		T* npdata = get_ndarray_data<T>(arr_double);

		typedef decltype(arr.shape(0)) PINT;
		PINT nrow = 0;
		PINT ncol = 0;
		if (arr.get_nd() == 1)
		{
			//nrow = arr.shape(0);
			//ncol = 1;
			nrow = 1;
			ncol = arr.shape(0);
		}
		else
		{
			nrow = arr.shape(0);
			ncol = arr.shape(1);
		}
		if (output.IsRowMajor)
		{
			output.resize(nrow, ncol);
			memcpy(output.data(), npdata, sizeof(T)*nrow*ncol);
		}
		else
		{
			output.resize(ncol, nrow);
			memcpy(output.data(), npdata, sizeof(T)*nrow*ncol);
			output.transposeInPlace();
		}
	}

	//convert to eigen vector
	template<typename T>
	void to_vector(const NDARRAY& arr, VECTOR_t<T>& output)
	{
		auto arr_double = get_ndarray_as<T>(arr);
		double* npdata = get_ndarray_data<T>(arr_double);

		auto nelem = np_size(arr);
		output.resize(nelem);

		//copy
		memcpy(output.data(), npdata, sizeof(T)*nelem);
	}

	//conversion to ndarray
	template<typename T>
	NDARRAY to_ndarray(const MATRIX_t<T>& mat)
	{
		auto nrow = mat.rows();
		auto ncol = mat.cols();
		np::ndarray arr = np::zeros(bp::make_tuple(nrow, ncol), np::dtype::get_builtin<T>());
		T* arr_data = get_ndarray_data<T>(arr);
		unsigned int k = 0;
		for (unsigned int i = 0; i < nrow; i++)
			for (unsigned int j = 0; j < ncol; j++)
				arr_data[k++] = mat(i, j);
		return arr;
	}

	template<typename T>
	NDARRAY to_ndarray(const VECTOR_t<T>& mat)
	{
		auto nelem = mat.size();
		np::ndarray arr = np::zeros(bp::make_tuple(nelem), np::dtype::get_builtin<T>());
		T* arr_data = get_ndarray_data<T>(arr);
		unsigned int k = 0;
		for (unsigned int i = 0; i < nelem; i++)
			arr_data[k++] = mat(i);
		return arr;
	}
}