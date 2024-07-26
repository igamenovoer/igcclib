#pragma once

#include "io_numpy.hpp"
#include <igcclib/vision/igcclib_opencv_def.hpp>

namespace _NS_UTILITY
{
	inline void save_np_array(const std::string& filename, const cv::Mat& mat)
	{
		std::vector<size_t> shape = { (size_t)mat.rows, (size_t)mat.cols, (size_t)mat.channels() };
		if (shape.back() == 1)
			shape.resize(2);

		size_t n_elem = 1;
		for (auto x : shape)
			n_elem *= x;

		if (mat.depth() == CV_8U)
			save_np_array(filename, shape, (uint8_t*)mat.data);
		else if (mat.depth() == CV_8S)
			save_np_array(filename, shape, (char*)mat.data);
		else if (mat.depth() == CV_16U)
			save_np_array(filename, shape, (uint16_t*)mat.data);
		else if (mat.depth() == CV_16S)
			save_np_array(filename, shape, (int16_t*)mat.data);
		else if (mat.depth() == CV_32S)
			save_np_array(filename, shape, (int32_t*)mat.data);
		else if (mat.depth() == CV_32F)
			save_np_array(filename, shape, (float*)mat.data);
		else if (mat.depth() == CV_64F)
			save_np_array(filename, shape, (double*)mat.data);
		else
			assert_throw(false, "unknown data type");
	}
}