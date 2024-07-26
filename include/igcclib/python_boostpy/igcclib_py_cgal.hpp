#pragma once

#include "igcclib_py_boost_def.hpp"
#include <igcclib/geometry/igcclib_cgal.hpp>

namespace _NS_UTILITY
{
	inline PLANE np_to_plane(const boost::numpy::ndarray& p0, const boost::numpy::ndarray& normal)
	{
		np::ndarray arr_p0 = get_ndarray_as<double>(p0);
		np::ndarray arr_normal = get_ndarray_as<double>(normal);

		//validate the numpy array
		assert_python(np_size(arr_p0) == 3, "p0 must have exactly 3 elements");
		assert_python(np_size(arr_normal) == 3, "normal must have exactly 3 elements");

		// read data
		double* p0_data = reinterpret_cast<double*>(arr_p0.get_data());
		POINT_3 pivot(p0_data[0], p0_data[1], p0_data[2]);

		double* normal_data = reinterpret_cast<double*>(arr_normal.get_data());
		VEC_3 plnormal(normal_data[0], normal_data[1], normal_data[2]);

		PLANE plane(pivot, plnormal);
		return plane;
	}

	inline POINTLIST_3 to_point_list_3(np::ndarray inpts)
	{
		typedef decltype(inpts.shape(0)) PINT;
		PINT nrow, ncol, row_stride, col_stride;
		inpts = get_ndarray_as<double>(inpts);
		if (inpts.get_nd() == 1)
			inpts = inpts.reshape(bp::make_tuple(1, 3));
		nrow = inpts.shape(0);
		ncol = inpts.shape(1);
		assert_python(ncol == 3, "input points must be nx3 numpy array");

		POINTLIST_3 output(nrow);
		row_stride = inpts.strides(0) / sizeof(double);
		col_stride = inpts.strides(1) / sizeof(double);
		double* vertex_reader = reinterpret_cast<double*>(inpts.get_data());
		double val[3];
		for (int i = 0; i < nrow; i++, vertex_reader += row_stride)
		{
			//reading ith point
			double* colread = vertex_reader;
			for (int j = 0; j < 3; j++, colread += col_stride)
			{
				val[j] = *colread;
			}
			output[i] = POINT_3(val[0], val[1], val[2]);
		}
		return output;
	}

	inline boost::numpy::ndarray to_ndarray(const POINTLIST_3& pts)
	{
		auto dtype = np::dtype::get_builtin<double>();
		auto npts = pts.size();
		bp::tuple shape = bp::make_tuple(npts, 3);
		np::ndarray arr_pts = np::empty(shape, dtype);

		//fill in data
		double* data = get_ndarray_data<double>(arr_pts);
		for (size_t i = 0; i < pts.size(); i++)
		{
			const auto& p = pts[i];
			auto base = 3 * i;
			data[base] = p.x();
			data[base + 1] = p.y();
			data[base + 2] = p.z();
		}

		return arr_pts;
	}
};