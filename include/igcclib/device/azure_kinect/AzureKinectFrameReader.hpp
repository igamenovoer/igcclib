#pragma once

#include "igcclib_device_azure_kinect.hpp"
#include <igcclib/device/DefaultFrameReader.hpp>

namespace _NS_UTILITY
{
	class AzureKinectFrameReader : public DefaultFrameReader {
	public:
		using Options = k4a_calibration_t;

		virtual bool get_color_coordinate_for_depth(fVECTOR_2* out_color_xy, const iVECTOR_2& depth_xy) const override;
		virtual bool get_color_coordinate_for_depth(cv::Mat* out_depth2color_xy, cv::Mat* out_mask) const override;
		virtual bool get_color_value_for_depth(iVECTOR_4* out_color_value, const iVECTOR_2& depth_xy, ImageFormat fmt = ImageFormat::NONE) const override;
		virtual bool get_color_value_for_depth(cv::Mat* out_depth2color, cv::Mat* out_mask, ImageFormat fmt = ImageFormat::NONE) const override;
		virtual bool get_depth_3d_point(fVECTOR_3* out_point, const iVECTOR_2& depth_xy) const override;

		/** \brief convert a depth pixel into 3d point, relative to specified device. */
		virtual bool get_depth_3d_point(fVECTOR_3* out_point, const iVECTOR_2& depth_xy, int relative_to_device_component) const;

		/** \brief convert all depth pixels into 3d points, relative to specified device. */
		virtual bool get_depth_3d_point(
			cv::Mat* out_depth2xyz, 
			int relative_to_device_component = DeviceComponentType::DEPTH_CAMERA) const;

		//virtual bool map_color_to_depth_coordinate(fVECTOR_2* out_depth_xy, const iVECTOR_2& color_xy) const;
		//virtual bool map_color_to_depth_coordinate(cv::Mat* out_color2depth_xy, cv::Mat* out_mask) const;

		/** \brief generate depth map for color image */
		virtual bool get_depth_value_for_color(cv::Mat* out_color2depth) const;		

		virtual bool get_color_3d_point(
			fVECTOR_3* out_point, const iVECTOR_2& color_xy, uint16_t depth_value, 
			int relative_to_device_component = DeviceComponentType::COLOR_CAMERA) const;

		/** \brief convert all color pixels into 3d points, relative to specified device. */
		virtual bool get_color_3d_point(
			cv::Mat* out_color2xyz, 
			const cv::Mat& color2depth, 
			int relative_to_device_component = DeviceComponentType::COLOR_CAMERA) const;

		/** \brief initialize with Options */
		virtual bool init(void* context) override;

	protected:
		std::shared_ptr<k4a_calibration_t> m_calib;
		AzureSharedPtr<k4a_transformation_t> m_transform;
	};
}