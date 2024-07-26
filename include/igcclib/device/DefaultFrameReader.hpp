#pragma once

#include "FrameReader.hpp"

namespace _NS_UTILITY
{
	/*!
	 * \class DefaultFrameReader
	 *
	 * \brief frame reader capable of reading standard frame data,
	 like those got from kinect, kinect v2 and kinect v3.
	 In standard frame data, the depth map is uint16 in mm unit.
	 Note that the data read from the frame may be reference, you should not modify them before making a copy.
	 * 
	 */
	class DefaultFrameReader :public FrameReader
	{
	public:
		virtual bool get_depth_image(cv::Mat* output) const override;

		virtual bool get_color_coordinate_for_depth(fVECTOR_2* out_color_xy, const iVECTOR_2& depth_xy) const override;
		virtual bool get_color_coordinate_for_depth(cv::Mat* out_depth2color_xy, cv::Mat* out_mask) const override;

		/** \brief image format is not used in default frame reader, we just copy the value as is */
		virtual bool get_color_value_for_depth(iVECTOR_4* out_color_value, const iVECTOR_2& depth_xy, ImageFormat fmt = ImageFormat::NONE) const override;

		/** \brief image format is not used in default frame reader, we just copy the value as is */
		virtual bool get_color_value_for_depth(cv::Mat* out_depth2color, cv::Mat* out_mask, ImageFormat fmt = ImageFormat::NONE) const override;
		virtual CameraModel get_depth_camera() const override;
		virtual bool get_depth_3d_point(fVECTOR_3* out_point, const iVECTOR_2& depth_xy) const override;
		virtual bool get_color_image(cv::Mat* output, ImageFormat fmt = ImageFormat::NONE) const override;
		virtual void set_framedata(FrameData::Ptr data) override;
		virtual CameraModel get_color_camera() const override;

		virtual bool init(void* context) override;		

	public:
		using DepthValue_t = uint16_t;

	protected:
		//accelerate access
		cv::Mat m_color_image;
		cv::Mat m_color_intrinsic, m_color_extrinsic, m_color_distortion;

		cv::Mat m_depth_image;
		cv::Mat m_depth2image_xy;
		cv::Mat m_depth2xyz;
		cv::Mat m_depth2color;
		cv::Mat m_depth_intrinsic, m_depth_extrinsic;
	};
}