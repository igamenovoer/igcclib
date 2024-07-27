#pragma once
#include <boost/optional.hpp>
#include <string>
#include <algorithm>

#include <igcclib/vision/igcclib_image_processing.hpp>
#include <igcclib/vision/igcclib_opencv_eigen.hpp>

namespace _NS_UTILITY
{
	inline void imshow(const cv::Mat& img, ImageFormat format = ImageFormat::NONE, bool normalize = false,
		std::string name = "",  int win_flag = cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO)
	{
		std::string _name;

		if (name.empty())
		{
			std::string namechars = "qwertyuiopasdfghjkl";
			std::vector<char> cands(namechars.begin(), namechars.end());
			std::random_shuffle(cands.begin(), cands.end());
			_name = std::string(cands.begin(), cands.end());
		}
		else
		{
			_name = name;
		}

		cv::Mat _img;
		img.copyTo(_img);

		if (normalize) {
			_img.convertTo(_img, CV_32F);
			std::vector<cv::Mat> chlist;
			cv::split(_img, chlist);
			for (int i = 0; i < chlist.size(); i++) {
				double v_min, v_max;
				cv::minMaxLoc(chlist[i], &v_min, &v_max);
				chlist[i] = (chlist[i] - v_min) / (v_max - v_min);
			}
			cv::merge(chlist, _img);
		}

		//convert floating point to uint8
		if (is_type_floating_point(_img))
			_img.convertTo(_img, CV_8U, 255.0);

		if (format != ImageFormat::NONE)
			convert_image(_img, _img, format, ImageFormat::BGRA, true);

		cv::namedWindow(_name, win_flag);
		cv::imshow(_name, _img);
		while (cv::getWindowProperty(_name, 0) >= 0)
		{
			int key = cv::waitKey(50);
			if (key > 0 && key < 255)
				break;
		}
		cv::destroyWindow(_name);
	}

	template<typename T>
	inline void imshow(const MATRIX_t<T>& img, bool auto_normalize = false,
		boost::optional<std::string> _name = boost::none,
		boost::optional<int> _winflag = boost::none) {

		std::string name = _name ? *_name : "";
		int winflag = _winflag ? *_winflag : (cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO);

		cv::Mat imgshow;

		if (std::is_same<T, float>::value || std::is_same<T, double>::value) {
			MATRIX_d tmp = img.template cast<double>();
			//grayscale image

			if (auto_normalize) {
				tmp = (tmp.array() - tmp.minCoeff()) / (tmp.maxCoeff() - tmp.minCoeff());
			}

			tmp = tmp.unaryExpr([](double x) {return std::min(1.0, std::max(0.0, x)); });
			MATRIX_u8 tmp_u8 = (tmp.array() * 255).template cast<uint8_t>();
			to_matrix(tmp_u8, imgshow);
		}
		else {
			MATRIX_u8 tmp = img.template cast<uint8_t>();
			to_matrix(tmp, imgshow);
		}

		imshow(imgshow, ImageFormat::GRAY, false, name, winflag);
	}
};