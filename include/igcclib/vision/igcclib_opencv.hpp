#pragma once

#include <cmath>
#include <type_traits>

#include <igcclib/vision/igcclib_opencv_def.hpp>
#include <igcclib/core/igcclib_eigen_def.hpp>

namespace _NS_UTILITY {

	/**
	* \brief find camera extrinsic such that pts_3d are projected to pts_2d using the
	given projection matrix
	*
	* \param pts_3d Nx3 object points
	* \param pts_2d Nx2 image points, in xy coordinate
	* \param projection_matrix right-mul projection matrix. For orthographic projection, the last column must be 0.
	* \return right-mul world-to-camera transformation matrix. Note that for orthographic projection,
	the extrinsic matrix will include a uniform scaling factor.
	*/
	IGCCLIB_API fMATRIX_4 find_camera_extrinsic(const fMATRIX& pts_3d, const fMATRIX& pts_2d, const fMATRIX_3& projection_matrix);

	/** \brief draw markers in an image,
	The marker type, size and line width follows cv::drawMarkers convention.*/
	IGCCLIB_API void draw_markers(
		cv::Mat& canvas, const fMATRIX& pts_xy,
		const fVECTOR_3& color3f, 
		int marker_type = cv::MARKER_CROSS, int marker_size = 20, int line_width = 1);

	/// <summary>
	/// resize the image so that its longest dimension is equal to a specified value
	/// </summary>
	/// <param name="input">the input image</param>
	/// <param name="maxlen">the length of the longest dimension</param>
	/// <param name="output">the output image</param>
	/// <param name="interp_method">opencv interpolation method, parameter of cv::resize()</param>
	IGCCLIB_API void imresize_by_maxlen(
		cv::Mat& output, 
		const cv::Mat& input, 
		int maxlen,
		int interp_method = cv::INTER_LINEAR);

	/// <summary>
	/// convert the image to 3 channel image. If the input is a grayscale image,
	/// it will be repeated 3 times. If the input contains alpha channel,
	/// it will be removed
	/// </summary>
	/// <param name="input">the input image</param>
	/// <param name="output">the output 3 channel image</param>
	IGCCLIB_API void to_3_channel(const cv::Mat& input, cv::Mat& output);

	IGCCLIB_API cv::Scalar make_scalar(double value, int n_channel);

	/// <summary>
	/// diagonal length of an image
	/// </summary>
	/// <param name="sz">the size of the image</param>
	/// <returns>the diagonal length</returns>
	IGCCLIB_API double diagonal_length(const cv::Size sz);

	inline bool is_same_size(const cv::Mat& x, const cv::Mat& y) {
		return x.rows == y.rows && x.cols == y.cols;
	}

	inline bool is_same_format(const cv::Mat& x, const cv::Mat& y) {
		return x.type() == y.type();
	}

	/// <summary>
	/// check the data type of cvmat, without considering channels
	/// </summary>
	/// <param name="mat">the cvmat</param>
	/// <returns>whether the data type equals to a primitive type T, without considering channels</returns>
	template<typename T>
	IGCCLIB_API bool is_type(const cv::Mat& mat) {
		return mat.depth() == cv::DataType<T>::type;
	}

	/// <summary>
	/// check if the data type is floating point (float or double)
	/// </summary>
	/// <param name="mat">the input matrix</param>
	/// <returns></returns>
	inline bool is_type_floating_point(const cv::Mat& mat) {
		return is_type<float>(mat) || is_type<double>(mat);
	}

	/// <summary>
	/// check if the data type is integer type
	/// </summary>
	/// <param name="mat">the input matrix</param>
	/// <returns></returns>
	inline bool is_type_integer(const cv::Mat& mat) {
		return is_type<uint8_t>(mat) || is_type<char>(mat)
			|| is_type<short>(mat) || is_type<unsigned short>(mat)
			|| is_type<int>(mat);
	}

	IGCCLIB_API void clamp_values(cv::Mat& inout, double minval, double maxval);

	/// <summary>
	/// convert the cvmat into floating point format, the type parameter can be float or double.
	/// The floating point image has value range in (0,1), so if the input is uint8, it will be
	/// divided by 255.
	/// </summary>
	/// <param name="inout">the cvmat to be converted</param>
	template<typename T, class = typename std::enable_if<std::is_floating_point<T>::value>::type>
	IGCCLIB_API void to_floating_point(cv::Mat& inout) {
		int cvtype = cv::DataType<T>::type;
		if (is_type_integer(inout))
			inout.convertTo(inout, cvtype, 1.0 / 255);
		else
			inout.convertTo(inout, cvtype);
	}

	/// <summary>
	/// convert the cvmat into uint8 format. If the input is float, we assume its range is in 0 to 1,
	/// and it will be scaled by 255
	/// </summary>
	/// <param name="inout"></param>
	IGCCLIB_API void to_uint8(cv::Mat& inout);

	/// <summary>
	/// a class to deal with format conversion of images.
	/// The value range of any integer image is within (0,255),
	/// the value rante of any floating point image is within (0,1)
	/// </summary>
	class IGCCLIB_API CvMatHelper
	{
	private:
		cv::Mat m_data; //the true data

	public:
		CvMatHelper() {}
		CvMatHelper(const cv::Mat& data, bool copy_data = false){
			set_data(data, copy_data);
		}

		void set_data(const cv::Mat& data, bool copy_data = false) { 
			if (copy_data)
				data.copyTo(m_data);
			else
				m_data = data;
		}

		//replace a channel, and return a copy of the data, the format follows the input data format
		cv::Mat get_with_channel_replaced(const cv::Mat& data, int idx_channel);

		const cv::Mat& get_data_ref() const { return m_data; }

		//get a copy of the internal data
		void get_data_copy(cv::Mat& output) const {
			m_data.copyTo(output);
		}

		cv::Mat get_data_copy() const {
			cv::Mat output;
			m_data.copyTo(output);
			return output;
		}

		/// <summary>
		/// get the image as another type such as CV_32FC3, with optionally automatic conversion
		/// </summary>
		/// <param name="output">the output image</param>
		/// <param name="allow_change_channel">if True, the conversion will also consider change of number of channels, otherwise only convert data type and leave the channels unchanged.</param>
		/// <param name="allow_scale_value">if True, when converting from integer to float, the value
		/// will be scaled by 255, and in the reverse direction, it will be divided by 255. If false,
		/// the values will be change</param>
		void get_as_type(cv::Mat& output, int cv_type, 
			bool allow_change_channel = true,
			bool allow_scale_value = true ) const;

		/// <summary>
		/// get the image as another type such as CV_32FC3, with optionally automatic conversion
		/// </summary>
		/// <param name="allow_change_channel">if True, the conversion will also consider change of number of channels, otherwise only convert data type and leave the channels unchanged.</param>
		/// <param name="allow_scale_value">if True, when converting from integer to float, the value
		/// will be scaled by 255, and in the reverse direction, it will be divided by 255. If false,
		/// the values will be change</param>
		cv::Mat get_as_type(int cv_type, bool allow_change_channel = true, bool allow_scale_value = true) const;

		/// <summary>
		/// modify the operating image with new data, handling type conversion,
		/// channel difference etc. For example, if the internal image has 4 channels
		/// and the new data has 3 channels, it will only modify the first 3 channels
		/// of the internal image. If they have type difference, 
		/// the new data will be automatically converted before modifying the internal image.
		/// </summary>
		/// <param name="data">the data to be set into the internal data.</param>
		/// <param name="allow_scale_value">if True, when converting from integer to float, the value
		/// will be scaled by 255, and in the reverse direction, it will be divided by 255. If false,
		/// the values will not be change</param>
		void update_by(const cv::Mat& data, bool allow_scale_value = true);
	};

}