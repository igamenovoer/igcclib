#pragma once
#include <boost/optional.hpp>

#include <igcclib/igcclib_master.hpp>
#include <igcclib/core/igcclib_eigen.hpp>
#include <igcclib/vision/igcclib_opencv_eigen.hpp>
#include <igcclib/vision/igcclib_opencv.hpp>

namespace _NS_UTILITY
{
	/// <summary>
	/// resize an image
	/// </summary>
	/// <param name="input">the input image</param>
	/// <param name="output">the output image</param>
	/// <param name="width">the target image width</param>
	/// <param name="height">the target image height</param>
	template<typename T>
	void IGCCLIB_API resize_image(const ImageRGBA_t<T>& input, ImageRGBA_t<T>& output, int width, int height)
	{
		using namespace _NS_UTILITY;
		int n_ch = input.get_number_of_channels();
		bool is_inout = &input == &output;

		if (is_inout)
		{
			//input and output uses the same object, we need temporary object
			ImageRGBA_t<T> tmp;
			tmp.allocate(width, height, n_ch);
			for (int i = 0; i < n_ch; i++)
			{
				cv::Mat cv_img(input.get_height(), input.get_width(), cv::DataType<T>::type, const_cast<T*>(input.get_channel(i).data()));
				cv::Mat _output(height, width, cv::DataType<T>::type, tmp.get_channel(i).data());
				cv::resize(cv_img, _output, cv::Size(width, height));
			}
			output = tmp;
		}
		else
		{
			//directly write to output
			output.allocate(width, height, n_ch);
			for (int i = 0; i < n_ch; i++)
			{
				cv::Mat cv_img(input.get_height(), input.get_width(), cv::DataType<T>::type, const_cast<T*>(input.get_channel(i).data()));
				cv::Mat _output(height, width, cv::DataType<T>::type, output.get_channel(i).data());
				cv::resize(cv_img, _output, cv::Size(width, height));
			}
		}
	}

	/// <summary>
	/// scale a rectangle defined in an image in pixel unit, relative to its own center.
	/// </summary>
	/// <param name="inout_xywh">the rectangle in pixel unit, (x,y,width,height), which will be overwritten as output</param>
	/// <param name="width">the image width</param>
	/// <param name="height">the image height</param>
	/// <param name="scale">scale factor of the pixel rect</param>
	/// <param name="force_square">force the rect to be square? if true, longer edge will be cut to match the shorter edge</param>
	IGCCLIB_API void scale_pixel_rect(iVECTOR_4& inout_xywh, int width, int height, double scale, bool force_square = false);

	/// <summary>
	/// convert image between formats
	/// </summary>
	/// <param name="img">the image to be converted</param>
	/// <param name="output">the output image</param>
	/// <param name="fmt_from">the format of img. Note that the format will be adjusted according
	/// to the image channels, for example, RGBA will be adjusted to RGB when the image has only 3 channels.</param>
	/// <param name="fmt_to">the desitination format</param>
	/// <param name="auto_adjust_format">if true, the format will be adjusted 
	/// (e.g., by adding or removing alpha channel) if needed</param>
	IGCCLIB_API void convert_image(const cv::Mat& img, cv::Mat& output,
		ImageFormat fmt_from, ImageFormat fmt_to, bool auto_adjust_format = false);

	/// <summary>
	/// read image. By default, the output image is either gray, rgb or rgba.
	/// </summary>
	/// <param name="filename">the image filename</param>
	/// <param name="output">the output image</param>
	/// <param name="output_format">the output image format, if set to None, then the output format
	/// will be gray, rgb or rgba, depends on the image content</param>
	IGCCLIB_API void imread(const std::string filename, cv::Mat& output, 
		ImageFormat output_format = ImageFormat::NONE);

	/// <summary>
	/// write an image to file
	/// </summary>
	/// <param name="filename">the output filename</param>
	/// <param name="image">the input image</param>
	/// <param name="format">the input image format, if set to NONE, 
	/// the image will be written as-is without conversion.</param>
	IGCCLIB_API void imwrite(const std::string filename, const cv::Mat& image,
		ImageFormat format = ImageFormat::NONE);

	/// <summary>
	/// perform distance transform. By default, 0 is obstacle and 1 is empty space.	
	/// </summary>
	/// <param name="mask">the binary mask</param>
	/// <param name="output">output distance matrix</param>
	/// <param name="zero_as_empty_space">if true, 0 is empty space and 1 is obstacle.</param>
	template<typename T, typename D>
	void IGCCLIB_API distance_transform(const MATRIX_t<T>& mask, MATRIX_t<D>& output,
		bool zero_as_empty_space = false) {

		MATRIX_u8 empty_space_mask; //in this mask, 0 is always obstacle, 1 is always empty space
		if (zero_as_empty_space)
			empty_space_mask = (mask.array() == 0).template cast<uint8_t>();
		else
			empty_space_mask = (mask.array() != 0).template cast<uint8_t>();

		//wrap as opencv mat
		cv::Mat _mask(empty_space_mask.rows(), empty_space_mask.cols(), 
			CV_8UC1, empty_space_mask.data());

		//compute distance transform
		cv::Mat distmap;
		cv::distanceTransform(_mask, distmap, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

		//output
		output = Eigen::Map<MATRIX_f>((float*)distmap.data, distmap.rows, distmap.cols).template cast<D>();
	}

	/**
	* \brief perform distance transform, and compute labels as well. By default, 0 is obstacle and 1 is empty space.	
	*
	* \param mask the binary mask
	* \param output_distance output distance matrix, output_distance(i,j) is the shortest distance from (i,j) to one of the foreground pixels in mask.
	* \param output_labels	output_labels(i,j) is the index of the nearest foreground pixel
	* \param zero_as_empty_space  if true, 0 is empty space and 1 is obstacle
	*/
	template<typename T, typename D>
	void IGCCLIB_API distance_transform(const MATRIX_t<T>& mask, MATRIX_t<D>& output_distance, iMATRIX& output_labels,
		bool zero_as_empty_space = false) {

		MATRIX_u8 empty_space_mask; //in this mask, 0 is always obstacle, 1 is always empty space
		if (zero_as_empty_space)
			empty_space_mask = (mask.array() == 0).template cast<uint8_t>();
		else
			empty_space_mask = (mask.array() != 0).template cast<uint8_t>();

		//wrap as opencv mat
		cv::Mat _mask(empty_space_mask.rows(), empty_space_mask.cols(),
			CV_8UC1, empty_space_mask.data());

		//compute distance transform
		cv::Mat distmap, lbmap;
		cv::distanceTransform(_mask, distmap, lbmap, cv::DIST_L2, cv::DIST_MASK_PRECISE, cv::DIST_LABEL_PIXEL);

		//find all obstacle pixels and assign a linear index to each
		std::vector<int> pixel_index;
		for (int i = 0; i < _mask.total(); i++)
			if (!_mask.data[i])
				pixel_index.push_back(i);

		lbmap.forEach<int32_t>(
			[&](int32_t& pixel, const int* position) {pixel = pixel_index[pixel-1]; }
		);

		//output
		output_distance = Eigen::Map<MATRIX_f>((float*)distmap.data, distmap.rows, distmap.cols).template cast<D>();
		output_labels = Eigen::Map<iMATRIX>((int32_t*)lbmap.data, lbmap.rows, lbmap.cols).template cast<int_type>();
	}

	/// <summary>
	/// perform distance transform. By default, 0 is obstacle and 1 is empty space.	
	/// </summary>
	/// <param name="mask">the binary mask</param>
	/// <param name="output">output distance matrix</param>
	/// <param name="zero_as_empty_space">if true, 0 is empty space and 1 is obstacle.</param>
	template<typename T>
	void IGCCLIB_API distance_transform(const cv::Mat& mask, MATRIX_t<T>& output, bool zero_as_empty_space = false) {
		cv::Mat _mask = mask != 0;
		MATRIX_u8 emask = Eigen::Map<MATRIX_u8>(_mask.data, _mask.rows, _mask.cols);
		distance_transform(emask, output, zero_as_empty_space);
	}

	template<typename T>
	void IGCCLIB_API distance_transform(const cv::Mat& mask, MATRIX_t<T>& output_distance, iMATRIX& output_labels, bool zero_as_empty_space = false) {
		cv::Mat _mask = mask != 0;
		MATRIX_u8 emask = Eigen::Map<MATRIX_u8>(_mask.data, _mask.rows, _mask.cols);
		distance_transform(emask, output_distance, output_labels, zero_as_empty_space);
	}

	/// <summary>
	/// find the boundary pixels of a binary mask, which is inside the foreground region.
	/// In the mask, non zeros are foreground pixels.
	/// </summary>
	/// <param name="mask">the mask</param>
	/// <param name="output">the boundary mask</param>
	IGCCLIB_API void find_inner_boundary(const cv::Mat& mask, cv::Mat& output);

	/// <summary>
	/// find the boundary pixels of a binary mask, which is inside the foreground region.
	/// In the mask, non zeros are foreground pixels.
	/// </summary>
	/// <param name="mask">the mask</param>
	/// <param name="output">the boundary mask</param>
	IGCCLIB_API void find_inner_boundary(const MATRIX_u8& mask, MATRIX_u8& output);

	/// <summary>
	/// fill all the holes (zero values) in 
	/// the binary image with 255.
	/// </summary>
	/// <param name="input">the input image, where non zeros are foreground, zeros are background</param>
	/// <param name="output">binary image with all the holes filled</param>
	IGCCLIB_API void binary_fill_holes(const cv::Mat& input, cv::Mat& output);

	/**
	* \brief find the largest connected component in input, and write it to output.
	*
	* \param input	the input single-channel image, where nonzero pixels are foreground
	* \param output	output CV_8UC1 mask of the largest connected component
	* \param connectivity	8 or 4 connnected
	*/
	IGCCLIB_API void get_largest_connected_component(const cv::Mat& input, cv::Mat* output, int connectivity = 8);

	/// <summary>
	/// upscale an image and then smooth it
	/// </summary>
	/// <param name="output">the output image</param>
	/// <param name="img">the input image, can be single or multi channel</param>
	/// <param name="upsize">size of the upscaled image</param>
	/// <param name="sigma">sigma of smoothing, in pixel unit</param>
	IGCCLIB_API void pyramid_up(cv::Mat& output,
		const cv::Mat& img, cv::Size upsize,
		boost::optional<double> sigma = boost::none);

	/// <summary>
	/// smooth an image and then downscale it
	/// </summary>
	/// <param name="output">the output image</param>
	/// <param name="img">the input image</param>
	/// <param name="downsize">the size of the downscaled image</param>
	/// <param name="sigma">sigma of smoothing before downscale</param>
	IGCCLIB_API void pyramid_down(cv::Mat& output,
		const cv::Mat& img, cv::Size downsize,
		boost::optional<double> sigma = boost::none);

	/// <summary>
	/// paste img_src[weight_src>0] into img_dst using multiband blending.
	/// The resulting image is img_src * weight_src + img_dst * (1-weight_src),
	/// but with smoothing across different frequency bands. Floating point images
	/// are recommended. If the image is uint8, it will be converted to 
	/// floating point before processing.
	/// Only accept uint8, float or double images.
	/// </summary>
	/// <param name="output">the output image</param>
	/// <param name="img_src">the source image, single or multi channel. If it is floating point, the color range is assumed to be in unit range. </param>
	/// <param name="img_dst">the destination image onto which the source image is pasted, should have the same format as source image.</param>
	/// <param name="weight_src">a weight map of the same size and source and destination, where 1.0 means only the source image is visible. </param>
	/// <param name="n_band">number of bands to use, if not set, use as many as possible.</param>
	/// <param name="sigma">the std variance of band filters.</param>
	IGCCLIB_API void multi_band_blending(cv::Mat& output,
		const cv::Mat& img_src, const cv::Mat& img_dst,
		const cv::Mat& weight_src,
		boost::optional<int> n_band = boost::none,
		boost::optional<double> sigma = boost::none);

	/**
	* \brief rotate the input image about its center, by a specified angle in counter-clockwise direction
	*
	* \param output the output image
	* \param out_transmat the right-multiply transformation matrix that is used to transform the input image, 
	treating every pixel as (x,y) where x+ right, y+ down. Can be nullptr to ignore this output.
	* \param input_image the input image
	* \param angle_rad the angle in radian by which the image is rotated
	* \param preserve_image_size whether the output image size is the same as input. If false, the output image will be sized just enough to contain the rotated image.
	*/
	IGCCLIB_API void rotate_image(cv::Mat& output, fMATRIX_3* out_transmat, const cv::Mat& input_image, double angle_rad,
		bool preserve_image_size /*= false*/);

};