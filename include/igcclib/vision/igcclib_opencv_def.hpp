#pragma once

#include <igcclib/igcclib_master.hpp>
#include <opencv2/opencv.hpp>

namespace _NS_UTILITY {

	//const int CV_FLOAT_TYPE = cv::DataType<float_type>::type;
	const int CV_FLOAT_TYPE = CV_32F;

	/// <summary>
	/// convert opencv type to image format. For multi channel image, prefer order in RGBA.
	/// </summary>
	/// <param name="cvtype">the opencv type</param>
	/// <returns>the image format</returns>
	inline ImageFormat image_format_from_cvtype_rgba(int cvtype) {
		switch (cvtype) {
		case CV_8UC1:
			return ImageFormat::GRAY;
		case CV_8UC3:
			return ImageFormat::RGB;
		case CV_8UC4:
			return ImageFormat::RGBA;
		default:
			return ImageFormat::NONE;
		}
	}

	/// <summary>
	/// convert image format to opencv data type
	/// </summary>
	/// <param name="format">the image format</param>
	/// <returns>the opencv datatype</returns>
	inline int cvtype_from_image_format(ImageFormat format)
	{
		switch (format) {
		case ImageFormat::BGR:
		case ImageFormat::RGB:
			return CV_8UC3;
		case ImageFormat::BGRA:
		case ImageFormat::RGBA:
			return CV_8UC4;
		case ImageFormat::GRAY:
			return CV_8UC1;
		default:
			assert_throw(false, "unsupported image format");
			return 0;
		}
	}

	/// <summary>
	/// wrap a linear image buffer into opencv image, the image buffer must be 
	/// RGBRGB or RGBARGBA or grayscale format.
	/// </summary>
	/// <param name="output">the output cv image</param>
	/// <param name="data">the image data</param>
	/// <param name="width">the width of the image</param>
	/// <param name="height">the height of the image</param>
	/// <param name="format">the image format</param>
	/// <param name="copy">copy the data into the cv image?</param>
	inline void wrap_as_opencv_image(cv::Mat& output, const uint8_t* data,
		size_t width, size_t height, ImageFormat format,
		bool copy = false)
	{
		int nch = get_num_channel(format);
		int cvtype = 0;
		switch (nch)
		{
		case 1:
			cvtype = CV_8UC1;
			break;
		case 3:
			cvtype = CV_8UC3;
			break;
		case 4:
			cvtype = CV_8UC4;
			break;
		}
		cv::Mat img((int)height, (int)width, cvtype, const_cast<uint8_t*>(data));

		if (copy)
		{
			img.copyTo(output);
		}
		else {
			output = img;
		}
	}
}

// =============== cereal support ==============
namespace cereal {
	template<typename T>
	void save(T& ar, const cv::Mat& img) {
		//save shape
		int nrow = img.rows;
		int ncol = img.cols;
		ar(nrow, ncol);

		//save type
		int cvtype = img.type();
		ar(cvtype);

		//save data
		std::vector<uint8_t> rawdata;
		if (!img.empty())
		{
			rawdata.resize(img.total() * img.elemSize());
			cv::Mat tmp(img.rows, img.cols, cvtype, rawdata.data());
			img.copyTo(tmp);
		}
		ar(rawdata);
	}

	template<typename T>
	void load(T& ar, cv::Mat& img) {
		//load shape
		int nrow, ncol;
		ar(nrow, ncol);

		//load type
		int cvtype;
		ar(cvtype);

		//load data
		std::vector<uint8_t> rawdata;
		ar(rawdata);

		cv::Mat(nrow, ncol, cvtype, rawdata.data()).copyTo(img);
	}
}