#pragma once
//#include "pch.h"
#include <igcclib/vision/igcclib_opencv_eigen.hpp>

namespace _NS_UTILITY
{
	//calibration output of opencv
	struct CalibrationInfo_OpenCV
	{
		cv::Mat object_points;
		cv::Mat image_points;
		cv::Mat camera_matrix;	//in opencv convention
		cv::Mat rvec, tvec; //extrinsic vectors
		cv::Mat distcoef;	//distortion

		template<typename T>
		void serialize(T& ar)
		{
			ar(object_points, image_points);
			ar(camera_matrix, rvec, tvec);
			ar(distcoef);
		}

		void set_image_points(const std::vector<cv::Point2f>& pts)
		{			
			image_points.create(pts.size(), 1, CV_32FC2);
			for (size_t i = 0; i < pts.size(); i++)
				image_points.at<cv::Vec2f>(i) = pts[i];
		}

		void set_object_points(const std::vector<cv::Point3f>& pts)
		{
			object_points.create(pts.size(), 1, CV_32FC3);
			for (size_t i = 0; i < pts.size(); i++)
				object_points.at<cv::Vec3f>(i) = pts[i];
		}

		/** \brief get extrinsic matrix in right-mul format */
		fMATRIX_4 get_extrinsic_matrix() const {
			fMATRIX_4 output = fMATRIX_4::Identity();

			if (!rvec.empty())
			{
				cv::Mat rotmat;
				cv::Rodrigues(rvec, rotmat);
				rotmat.convertTo(rotmat, cv::DataType<float_type>::type);
				output.block(0, 0, 3, 3) = Eigen::Map<fMATRIX>((float_type*)rotmat.data, 3, 3).transpose();
			}

			if (!tvec.empty())
			{
				fMATRIX _tvec;
				to_matrix(tvec, _tvec);
				for (int i = 0; i < 3; i++)
					output(3, i) = _tvec.data()[i];
			}

			return output;
		}
	};
}
