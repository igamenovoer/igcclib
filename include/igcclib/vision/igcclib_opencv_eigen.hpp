#pragma once
#include <igcclib/igcclib_master.hpp>
#include <igcclib/core/igcclib_eigen_def.hpp>
#include "igcclib_opencv_def.hpp"

namespace _NS_UTILITY
{
	template<typename T, int NROW, int NCOL>
	IGCCLIB_API void to_matrix(const cv::Mat& src, MATRIX_xt<T, NROW, NCOL>& dst);

	template<typename T, int NROW, int NCOL>
	IGCCLIB_API void to_matrix(const MATRIX_xt<T, NROW, NCOL>& src, cv::Mat& dst);

	template<typename T, int NROW>
	IGCCLIB_API void to_vector(const cv::Mat& src, VECTOR_xt<T, NROW>& dst);

	/// <summary>
	/// wrap the eigen matrix into opencv matrix, sharing the internal data
	/// </summary>
	/// <param name="data">the eigen matrix</param>
	/// <returns>the wrapped opencv matrix</returns>
	template<typename T>
	IGCCLIB_API cv::Mat wrap_as_opencv_matrix(const MATRIX_t<T>& data);

	/** \brief convert a list of key points to a nx2 matrix */
	template<typename T>
	IGCCLIB_API void to_matrix(const std::vector<cv::KeyPoint>& kps, MATRIX_t<T>& dst);

	/** \brief convert eigen image to opencv image */
	template<typename T>
	IGCCLIB_API void to_opencv_image(const ImageRGBA_t<T>& img, cv::Mat& output, bool keep_float = false);

	/** \brief convert opencv image to eigen image */
	template <typename T>
	IGCCLIB_API void to_eigen_image(const cv::Mat& img, ImageRGBA_t<T>& output);

	/// <summary>
	/// extract elements in opencv matrix by mask, like data[mask] in numpy
	/// </summary>
	/// <param name="mat">the opencv single channel data</param>
	/// <param name="mask">the mask of the matrix entries to be extracted</param>
	/// <param name="output">output vector</param>
	template<typename T>
	IGCCLIB_API void submat_by_mask(const cv::Mat& mat, const cv::Mat& mask, VECTOR_t<T>& output);

	/**
	* \brief convert extrinsic returned by cv::solvepnp to a transmat that converts
	* object points in world to camera coordinate
	*
	* \param rvec rotation returned by cv::solvepnp
	* \param tvec translation returned by cv::solvepnp
	* \return fMATRIX_4 the right-mul transformation matrix
	*/
	inline fMATRIX_4 convert_opencv_extrinsic_obj2camera(const cv::Mat& rvec, const cv::Mat& tvec);
};

namespace _NS_UTILITY
{
	fMATRIX_4 convert_opencv_extrinsic_obj2camera(const cv::Mat& rvec, const cv::Mat& tvec)
	{
		using FT = fMATRIX_4::value_type;

		cv::Mat rotmat, d;
		cv::Rodrigues(rvec, rotmat);
		rotmat.convertTo(rotmat, cv::DataType<FT>::type);
		tvec.convertTo(d, cv::DataType<FT>::type);
		fMATRIX_4 output = fMATRIX_4::Identity();
		output.block(0, 0, 3, 3) = Eigen::Map<fMATRIX>((FT*)rotmat.data, 3, 3);
		output.topRows(3).rightCols(1) = Eigen::Map<fVECTOR_3>((FT*)d.data);
		output.transposeInPlace();
		return output;
	}

	template<typename T, int NROW>
	void to_vector(const cv::Mat& src, VECTOR_xt<T, NROW>& dst)
	{
		auto cvtype = cv::DataType<T>::type;
		int nrow = src.rows;
		int ncol = src.cols;
		if (cvtype == src.type())
		{
			dst = Eigen::Map<VECTOR_t<T>>((T*)src.data, nrow * ncol * src.channels());
		}
		else
		{
			cv::Mat srcnew;
			src.convertTo(srcnew, cvtype);
			dst = Eigen::Map<VECTOR_t<T>>((T*)srcnew.data, nrow * ncol * src.channels());
		}
	}

	template<typename T, int NROW, int NCOL>
	void to_matrix(const cv::Mat& src, MATRIX_xt<T, NROW, NCOL>& dst)
	{
		auto cvtype = cv::DataType<T>::type;
		int nrow = src.rows;
		int ncol = src.cols;
		if (cvtype == src.type())
		{
			dst = Eigen::Map<MATRIX_t<T>>((T*)src.data, nrow, ncol);
		}
		else
		{
			cv::Mat srcnew;
			src.convertTo(srcnew, cvtype);
			dst = Eigen::Map<MATRIX_t<T>>((T*)srcnew.data, nrow, ncol);
		}
	}

	template<typename T, int NROW, int NCOL>
	void to_matrix(const MATRIX_xt<T, NROW, NCOL>& src, cv::Mat& dst)
	{
		auto cvtype = cv::DataType<T>::type;
		int nrow = (int)src.rows();
		int ncol = (int)src.cols();
		cv::Mat x(nrow, ncol, cvtype, (void*)src.data());
		x.copyTo(dst);
	}

	/// <summary>
	/// wrap the eigen matrix into opencv matrix, sharing the internal data
	/// </summary>
	/// <param name="data">the eigen matrix</param>
	/// <returns>the wrapped opencv matrix</returns>
	template<typename T>
	cv::Mat wrap_as_opencv_matrix(const MATRIX_t<T>& data) {
		int cvtype = cv::DataType<T>::type;
		cv::Mat output((int)data.rows(), (int)data.cols(), 
			cvtype, const_cast<T*>(data.data()));
		return output;
	}

	//convert a list of key points to a nx2 matrix
	template<typename T>
	void to_matrix(const std::vector<cv::KeyPoint>& kps, MATRIX_t<T>& dst)
	{
		dst.setZero(kps.size(), 2);
		for (int i = 0; i < kps.size(); i++)
		{
			dst(i, 0) = kps[i].pt.x;
			dst(i, 1) = kps[i].pt.y;
		}
	}

	//convert eigen image to opencv image
	template<typename T>
	void to_opencv_image(const ImageRGBA_t<T>& img, cv::Mat& output, bool keep_float)
	{
		bool is_float_image = std::is_same<T, float>::value || std::is_same<T, double>::value;
		int height = img.get_height();
		int width = img.get_width();

		if (img.is_gray())
		{
			to_matrix(img.r, output);
			if (is_float_image && !keep_float)
			{
				cv::Mat tmp;
				output.convertTo(tmp, CV_8UC1, 255);
				output = tmp;
			}
		}
		else //RGB or RGBA image
		{
			std::vector<cv::Mat> chlist(3);
			to_matrix(img.r, chlist[0]);
			to_matrix(img.g, chlist[1]);
			to_matrix(img.b, chlist[2]);
			if (img.has_alpha())
			{
				cv::Mat x;
				to_matrix(img.a, x);
				chlist.push_back(x);
			}
			cv::merge(chlist, output);

			if (is_float_image && !keep_float)
			{
				cv::Mat tmp;
				int cvtype = chlist.size() == 3 ? CV_8UC3 : CV_8UC4;
				output.convertTo(tmp, cvtype, 255);
				output = tmp;
			}
		}
	}

	//convert opencv image to eigen image
	template <typename T>
	void to_eigen_image(const cv::Mat& img, ImageRGBA_t<T>& output)
	{
		int cvtype = img.type();
		bool is_integer_image = cvtype == CV_8UC1 || cvtype == CV_8UC3 || cvtype == CV_8UC4;
		bool is_float_image = cvtype == CV_32FC1 || cvtype == CV_32FC3 || cvtype == CV_32FC4
			|| cvtype == CV_64FC1 || cvtype == CV_64FC3 || cvtype == CV_64FC4;
		bool is_output_float = std::is_same<T, float>::value || std::is_same<T, double>::value;

		assert_throw(is_integer_image || is_float_image, "only supports images containing 1,3,4 channels");

		//single channel image?
		if (img.channels() == 1)
		{
			MATRIX_t<T> mat;
			to_matrix(img, mat);

			if(is_integer_image && is_output_float)
				mat /= 255;
			
			if (is_float_image && !is_output_float)
				mat *= 255;

			output.set_gray(mat);
		}
		else //RGB or RGBA
		{
			std::vector<cv::Mat> chlist;
			cv::split(img, chlist);
			
			std::vector<MATRIX_t<T>> matlist(chlist.size());
			for (int i = 0; i < chlist.size(); i++)
			{
				auto& mat = matlist[i];

				to_matrix(chlist[i], mat);
				if (is_integer_image && is_output_float)
					mat /= 255;

				if (is_float_image && !is_output_float)
					mat *= 255;
			}

			if (chlist.size() == 3)
				output.set_RGB(matlist[0], matlist[1], matlist[2]);
			if (chlist.size() == 4)
				output.set_RGBA(matlist[0], matlist[1], matlist[2], matlist[3]);
		}
	}

	/// <summary>
	/// extract elements in opencv matrix by mask, like data[mask] in numpy
	/// </summary>
	/// <param name="mat">the opencv single channel data</param>
	/// <param name="mask">the mask of the matrix entries to be extracted</param>
	/// <param name="output">output vector</param>
	template<typename T>
	void submat_by_mask(const cv::Mat& mat, const cv::Mat& mask, VECTOR_t<T>& output) {
		assert_throw(mat.channels() == 1, "input matrix must be single channel");
		assert_throw(mat.size == mask.size, "matrix and mask must have the same size");

		//count the number of elements
		auto n =cv::countNonZero(mask);
		output.resize(n);

		//unify type
		cv::Mat _mat, _mask;
		{
			int cvtype = cv::DataType<T>::type;
			if (mat.type() != cvtype)
				mat.convertTo(_mat, cvtype);
			else
				_mat = mat;

			if (mask.type() != CV_8UC1)
				_mask = mask > 0;
			else
				_mask = mask;
		}

		//read data
		int k = 0;
		for(int i=0; i<mask.rows; i++)
			for (int j = 0; j < mask.cols; j++)
			{
				if (_mask.at<uint8_t>(i, j))
					output(k++) = _mat.at<T>(i, j);
			}

		assert_throw(k == n, "some element is not read for unknown reason");
	}
};