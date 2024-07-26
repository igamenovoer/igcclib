#pragma once

#include "FrameData.hpp"

namespace _NS_UTILITY
{
	/*!
		* \class FrameReader
		*
		* \brief device-dependent frame content reader.
		*/
	class FrameReader {
	protected:
		FrameData::Ptr m_data;

	public:
		using Ptr = std::shared_ptr<FrameReader>;
		FrameReader() {}
		//FrameReader(FrameData::Ptr data) { init_with_framedata(data); }
		//FrameReader(const FrameData& data) { init_with_framedata(data); }

		virtual ~FrameReader() {}

		/** \brief initialize with context, return whether it is successful */
		virtual bool init(void* context) = 0;

		virtual void set_framedata(FrameData::Ptr data) {
			m_data = data;
		}

		virtual const FrameData::Ptr& get_frame_data() const { return m_data; }

		/** \brief depth image captured by depth camera. 0 value in the depth map represents invalid depth. */
		virtual bool get_depth_image(cv::Mat* output) const = 0;

		/** \brief get a uint8 mask designating which depth pixel is usable */
		virtual bool get_depth_mask(cv::Mat* output) const
		{
			if (!m_data) return false;
			cv::Mat dmap;
			bool has_depth = get_depth_image(&dmap);
			if (!has_depth)
				return false;

			if (output)
				*output = dmap != 0;
			return true;
		}

		/** \brief mapping depth pixel coordinate to color pixel coordinate, both in xy space.
		return whether the conversion is successful. */
		virtual bool get_color_coordinate_for_depth(fVECTOR_2* out_color_xy, const iVECTOR_2& depth_xy) const = 0;

		/**
		* \brief get a mapping from depth to color coordinates all together
		*
		* \param out_depth2color_xy CV_32FC2, mapping each depth pixel (i,j) to color pixel (x,y) in xy space
		* \param out_mask mask(i,j)>0 iff depth pixel (i,j) has a valid color pixel correspondence
		* \return bool whether there exists a map
		*/
		virtual bool get_color_coordinate_for_depth(cv::Mat* out_depth2color_xy, cv::Mat* out_mask) const = 0;

		/** \brief for a depth pixel, get the color value by mapping it to color image
		and retrieving the value. */
		virtual bool get_color_value_for_depth(
			iVECTOR_4* out_color_value, const iVECTOR_2& depth_xy,
			ImageFormat fmt = ImageFormat::NONE) const = 0;

		/**
		* \brief assigning a color to each depth pixel
		*
		* \param out_depth2color	 a color image of the same size as depth map,
		where x(i,j) is the color of depth pixel (i,j).
		* \param out_mask mask(i,j)>0 iff pixel (i,j) has a valid color
		* \param fmt the returned image format. If not specified, use the default format.
		* \return bool whether the operation is successful
		*/
		virtual bool get_color_value_for_depth(
			cv::Mat* out_depth2color, cv::Mat* out_mask,
			ImageFormat fmt = ImageFormat::NONE) const = 0;

		/** \brief get depth camera intrinsic and extrinsic */
		virtual CameraModel get_depth_camera() const = 0;

		/** \brief get the 3d point by back projecting a depth pixel, in depth camera space.
		return whether the conversion is successful. */
		virtual bool get_depth_3d_point(fVECTOR_3* out_point, const iVECTOR_2& depth_xy) const = 0;

		/** \brief color image captured by color camera with specified format.
		If not specified, use the format as the device does. */
		virtual bool get_color_image(cv::Mat* output, ImageFormat fmt = ImageFormat::NONE) const = 0;

		/** \brief get the color camera intrinsic, extrinsic and distortion info */
		virtual CameraModel get_color_camera() const = 0;
	};
}