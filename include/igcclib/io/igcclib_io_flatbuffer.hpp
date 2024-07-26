#pragma once
#include <igcclib/igcclib_master.hpp>
#include <igcclib/core/igcclib_eigen_def.hpp>

#include "generated/matrix_generated.h"
#include <vector>

namespace _NS_UTILITY
{
	template<typename T>
	struct FbsMatrixCreator {};

	template<>
	struct FbsMatrixCreator<int32_t> {
		using FbsReturn_t = flatbuffers::Offset < fbsdata::Matrix_i32>;
		static decltype(&fbsdata::CreateMatrix_i32) func() {
			return &fbsdata::CreateMatrix_i32;
		};
	};

	template<>
	struct FbsMatrixCreator<float> {
		using FbsReturn_t = flatbuffers::Offset < fbsdata::Matrix_f>;
		static decltype(&fbsdata::CreateMatrix_f) func() {
			return &fbsdata::CreateMatrix_f;
		}
	};

	template<>
	struct FbsMatrixCreator<double> {
		using FbsReturn_t = flatbuffers::Offset < fbsdata::Matrix_d>;
		static decltype(&fbsdata::CreateMatrix_d) func() {
			return &fbsdata::CreateMatrix_d;
		}
	};

	template<>
	struct FbsMatrixCreator<char> {
		using FbsReturn_t = flatbuffers::Offset < fbsdata::Matrix_u8>;
		static decltype(&fbsdata::CreateMatrix_u8) func() {
			return &fbsdata::CreateMatrix_u8;
		}
	};

	template<>
	struct FbsMatrixCreator<uint8_t> {
		using FbsReturn_t = flatbuffers::Offset<fbsdata::Matrix_u8>;
		static decltype(&fbsdata::CreateMatrix_u8) func() {
			return &fbsdata::CreateMatrix_u8;
		}
	};

	template<typename T, int NROW, int NCOL>
	typename FbsMatrixCreator<T>::FbsReturn_t 
		write_matrix_as_fbs(flatbuffers::FlatBufferBuilder& builder, const MATRIX_xt<T, NROW, NCOL>& _output)
	{
		auto f_content = builder.CreateVector(_output.data(), _output.size());

		std::vector<int32_t> shape{ (int32_t)_output.rows(),(int32_t)_output.cols() };
		auto f_shape = builder.CreateVector(shape);

		return FbsMatrixCreator<T>::func()(builder, f_content, f_shape);
	}



	template<typename MAT_TYPE, typename OUT_DATA_TYPE>
	void write_matrix_as_fbs(const MAT_TYPE& output, std::string filename)
	{
		using T = typename std::decay<OUT_DATA_TYPE>::type;
		namespace fbs = flatbuffers;
		fbs::FlatBufferBuilder builder(0);
		MATRIX_t<T> _output = output.template cast<T>();
		auto f_content = builder.CreateVector(_output.data(), _output.size());

		std::vector<int32_t> shape{ (int32_t)_output.rows(),(int32_t)_output.cols() };
		auto f_shape = builder.CreateVector(shape);

		auto mat = FbsMatrixCreator<T>::func()(builder, f_content, f_shape);
		builder.Finish(mat);

		auto buf = builder.GetBufferPointer();
		auto bufsize = builder.GetSize();

		std::ofstream outfile(filename, std::ios::binary | std::ios::trunc);
		outfile.write((char*)buf, bufsize);
		outfile.close();
	}

	template<typename MAT_TYPE>
	void write_matrix_as_fbs(const MAT_TYPE& output, std::string filename) {
		using T = typename MAT_TYPE::value_type;
		write_matrix_as_fbs<MAT_TYPE, T>(output, filename);
	}

	template<typename T, typename D>
	void read_matrix_from_fbs(const T* fb_mat, MATRIX_t<D>& output)
	{
		assert_throw(fb_mat->shape()->size() <= 2, "only supports 2d matrix");

		int nrow = fb_mat->shape()->Get(0);
		int ncol = 1;
		if (fb_mat->shape()->size() == 2)
			ncol = fb_mat->shape()->Get(1);

		output.resize(nrow, ncol);
		for (decltype(fb_mat->content()->size()) i = 0; i < fb_mat->content()->size(); i++)
			output(i) = (D)(fb_mat->content()->Get(i));
	}

	/// <summary>
	/// read matrix from flatbuffer into a linear buffer
	/// </summary>
	/// <param name="fb_mat">the flatbuffer matrix</param>
	/// <param name="output">the output buffer</param>
	template<typename T, typename D>
	void read_matrix_from_fbs(const T* fb_mat, std::vector<D>& output)
	{
		int n_element = 1;
		const auto fb_shape = fb_mat->shape();
		for (decltype(fb_shape->size()) i = 0; i < fb_shape->size(); i++)
			n_element *= fb_shape->Get(i);

		output.resize(n_element);
		const auto fb_content = fb_mat->content();
		for (decltype(fb_content->size()) i = 0; i < fb_content->size(); i++)
			output[i] = fb_content->Get(i);
	}

	template<typename T>
	typename FbsMatrixCreator<T>::FbsReturn_t 
		write_img_to_fbs(flatbuffers::FlatBufferBuilder& builder, std::vector<T> buffer,int width,int height,int n_channel)
	{
		auto f_content = builder.CreateVector(buffer.data(), buffer.size());

		std::vector<int32_t> shape{ (int32_t)height,(int32_t)width,(int32_t)n_channel };
		auto f_shape = builder.CreateVector(shape);

		return FbsMatrixCreator<T>::func()(builder, f_content, f_shape);
	}

	template<typename T, typename D>
	void read_image_from_fbs(const T* fb_mat, ImageRGBA_t<D>& output,
		double in_min, double in_max, double out_min, double out_max)
	{
		//number of channels
		int n_channel = 0;
		const auto fb_shape = fb_mat->shape();
		assert_throw(fb_shape->size() >= 2, "image must be at least 2 dimensional");
		assert_throw(fb_shape->size() < 4, "image can not have more than 3 dimensions");

		if (fb_shape->size() == 2)
			n_channel = 1;
		else if (fb_shape->size() == 3)
			n_channel = fb_shape->Get(2);

		//get image size
		int height = fb_shape->Get(0);
		int width = fb_shape->Get(1);

		//read the data as raw buffer
		std::vector<D> buffer;
		read_matrix_from_fbs(fb_mat, buffer);

		//construct the image from raw buffer
		output.template from_linear_buffer<D>(buffer, width, height, n_channel, in_min, in_max, out_min, out_max);
	
		//for debug
		//cv::Mat cv_img(height, width, CV_8UC4, buffer.data());
		//cv_img = cv_img.clone();
		//output.to_linear_buffer(buffer, 0, 255, 0, 255);
		//cv::Mat cv_new(height, width, CV_8UC4, buffer.data());
		//cv_new = cv_new.clone();
	}
}