#include "AzureKinectFrameReader.h"
#include "AzureKinectReader.h"

namespace _NS_UTILITY
{

	bool AzureKinectFrameReader::get_color_coordinate_for_depth(fVECTOR_2* out_color_xy, const iVECTOR_2& depth_xy) const
	{
		if (!m_data)
			return false;

		//do we have calibration?
		if (!m_calib)
			//no? use the default method
			return DefaultFrameReader::get_color_coordinate_for_depth(out_color_xy, depth_xy);

		//we require the depth map
		if (m_depth_image.empty())
			return false;

		//coordinate in range?
		const auto& dmap = m_depth_image;

		auto x = depth_xy[0];
		auto y = depth_xy[1];
		if (x < 0 || x >= dmap.cols || y < 0 || y >= dmap.rows)
			return false;

		auto depth_value = dmap.at<uint16_t>(y, x);
		if (depth_value == INVALID_DEPTH_VALUE)
			return false;

		if (!out_color_xy)
			return true;

		//we have calibration data, compute the coordinate now
		k4a_float2_t depth_coordinate;
		depth_coordinate.v[0] = x;
		depth_coordinate.v[1] = y;

		k4a_float2_t	 color_coordinate;
		int valid;
		k4a_calibration_2d_to_2d(m_calib.get(), &depth_coordinate, depth_value, 
			K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &color_coordinate, &valid);

		out_color_xy->x() = color_coordinate.v[0];
		out_color_xy->y() = color_coordinate.v[1];

		return true;
	}

	bool AzureKinectFrameReader::get_color_coordinate_for_depth(cv::Mat* out_depth2color_xy, cv::Mat* out_mask) const
	{
		if (!m_data)
			return false;

		//do we have calibration?
		if (!m_calib)
			//no? use the default method
			return DefaultFrameReader::get_color_coordinate_for_depth(out_depth2color_xy, out_mask);

		//we require the depth map
		if (m_depth_image.empty())
			return false;

		if (!out_depth2color_xy && !out_mask)
			return true;

		convert_k4a_depth_to_image_coordinate(out_depth2color_xy, out_mask, *m_calib, m_depth_image, 
			K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR);
		return true;
	}

	bool AzureKinectFrameReader::get_color_value_for_depth(iVECTOR_4* out_color_value, const iVECTOR_2& depth_xy, ImageFormat fmt /*= ImageFormat::NONE*/) const
	{
		if (!m_data)
			return false;

		iVECTOR_4 _out_color_value{ 0,0,0,0 };
		bool ok = true;

		//do we have calibration?
		if (!m_calib)
		{
			//no? use the default method
			ok = DefaultFrameReader::get_color_value_for_depth(&_out_color_value, depth_xy, fmt);
		}
		else {
			if (m_depth_image.empty() || m_color_image.empty())
				return false;

			//use k4a api
			fVECTOR_2 color_xy;
			ok = get_color_coordinate_for_depth(&color_xy, depth_xy);
			bool in_range = color_xy.x() >= 0 && color_xy.x() < m_color_image.cols && color_xy.y() >= 0 && color_xy.y() < m_color_image.rows;
			ok = ok && in_range;
			if (ok)
			{
				auto x = m_color_image.at<cv::Vec4b>(color_xy.y(), color_xy.x());
				for (int k = 0; k < 4; k++)
					_out_color_value[k] = x[k];
			}
		}

		//change format, we know the captured image is BGRA32
		switch (fmt)
		{
		case ImageFormat::RGB:
		case ImageFormat::RGBA:
			std::swap(_out_color_value[0], _out_color_value[2]);
			break;
		case ImageFormat::GRAY:
		{
			auto mean_value = (_out_color_value[0] + _out_color_value[1] + _out_color_value[2]) / 3;
			for (int k = 0; k < 3; k++)
				_out_color_value[k] = mean_value;
		}break;
		}

		return ok;
	}

	bool AzureKinectFrameReader::get_color_value_for_depth(cv::Mat* out_depth2color, cv::Mat* out_mask, ImageFormat fmt /*= ImageFormat::NONE*/) const
	{
		if (!m_data)
			return false;

		if (!m_calib)
		{
			return DefaultFrameReader::get_color_value_for_depth(out_depth2color, out_mask, fmt);
		}
		else
		{
			if (m_depth_image.empty() || m_color_image.empty())
				return false;

			if (!out_depth2color && !out_mask)
				return true;
			
			//wrap images into k4a format
			auto _depth_image = wrap_cvmat_to_k4a_image(m_depth_image);
			auto _color_image = wrap_cvmat_to_k4a_image(m_color_image);

			cv::Mat tmp;
			cv::Mat* output = out_depth2color ? out_depth2color : &tmp;
			output->create(m_depth_image.rows, m_depth_image.cols, CV_8UC4);
			output->setTo(cv::Scalar::all(0));
			auto _output_image = wrap_cvmat_to_k4a_image(*output);

			//do the conversion
			auto res = k4a_transformation_color_image_to_depth_camera(
				m_transform.get(), _depth_image.get(), 
				_color_image.get(), _output_image.get());
			assert_throw(!K4A_FAILED(res), "failed to map depth to color");

			//get the mask from the alpha channel of output
			if (out_mask)
			{
				out_mask->create(output->rows, output->cols, CV_8UC1);
				out_mask->setTo(0);
				for (int i = 0; i < output->total(); i++)
					out_mask->data[i] = output->at<cv::Vec4b>(i)[3];
			}

			//convert format
			if (out_depth2color)
			{
				if (fmt == ImageFormat::BGR)
					cv::cvtColor(*output, *output, cv::COLOR_BGRA2BGR);
				else if (fmt == ImageFormat::GRAY)
					cv::cvtColor(*output, *output, cv::COLOR_BGRA2GRAY);
				else if (fmt == ImageFormat::RGB)
					cv::cvtColor(*output, *output, cv::COLOR_BGRA2RGB);
				else if (fmt == ImageFormat::RGBA)
					cv::cvtColor(*output, *output, cv::COLOR_BGRA2RGBA);
			}

			return true;
		}
	}

	bool AzureKinectFrameReader::get_depth_3d_point(fVECTOR_3* out_point, const iVECTOR_2& depth_xy) const
	{
		return get_depth_3d_point(out_point, depth_xy, DeviceComponentType::DEPTH_CAMERA);
	}

	bool AzureKinectFrameReader::get_depth_3d_point(
		fVECTOR_3* out_point, const iVECTOR_2& depth_xy, 
		int relative_to_device_component) const
	{
		if (!m_data) return false;

		if (!m_calib)
			return DefaultFrameReader::get_depth_3d_point(out_point, depth_xy);

		if (m_depth_image.empty())
			return false;

		//in range test
		auto x = depth_xy.x();
		auto y = depth_xy.y();
		bool in_range = x >= 0 && x < m_depth_image.cols && y >= 0 && y < m_depth_image.rows;
		if (!in_range)
			return false;

		//depth validity test
		bool valid_depth = m_depth_image.at<uint16_t>(y, x) != INVALID_DEPTH_VALUE;
		if (!valid_depth)
			return false;

		if (!out_point) return true;

		//map point
		k4a_float2_t p;
		p.v[0] = x;
		p.v[1] = y;
		auto depth_value = m_depth_image.at<uint16_t>(y, x);
		int is_valid;

		k4a_float3_t q;
		auto target_k4a_camera = get_component_type_to_k4a_type().at(relative_to_device_component);
		k4a_calibration_2d_to_3d(m_calib.get(), &p, depth_value,
			K4A_CALIBRATION_TYPE_DEPTH, target_k4a_camera,
			&q, &is_valid);

		out_point->x() = q.xyz.x;
		out_point->y() = q.xyz.y;
		out_point->z() = q.xyz.z;

		return is_valid;
	}

	bool AzureKinectFrameReader::get_depth_3d_point(
		cv::Mat* out_depth2xyz, 
		int relative_to_device_component) const
	{
		if (!m_data || !m_calib || m_depth_image.empty())
			return false;

		if (!out_depth2xyz)
			return true;

		auto target_k4a_type = get_component_type_to_k4a_type().at(relative_to_device_component);
		convert_k4a_depth_to_point_cloud(
			out_depth2xyz, nullptr, *m_calib, m_depth_image,
			K4A_CALIBRATION_TYPE_DEPTH,
			target_k4a_type);
		return true;
	}

	bool AzureKinectFrameReader::get_depth_value_for_color(cv::Mat* out_color2depth) const
	{
		if (!m_data || !m_calib || m_depth_image.empty() || m_color_image.empty())
			return false;

		if (!out_color2depth)
			return true;

		cv::Mat tmp;
		cv::Mat* output = out_color2depth ? out_color2depth : &tmp;
		output->create(m_color_image.rows, m_color_image.cols, CV_16UC1);
		output->setTo(cv::Scalar::all(0));

		auto _depth_image = wrap_cvmat_to_k4a_image(m_depth_image);
		auto _output = wrap_cvmat_to_k4a_image(*output);

		auto res = k4a_transformation_depth_image_to_color_camera(
			m_transform.get(), _depth_image.get(), _output.get()
		);
		assert_throw(!K4A_FAILED(res), "failed to generate color2depth");

		return true;
	}

	bool AzureKinectFrameReader::init(void* context)
	{
		DefaultFrameReader::init(context);
		m_calib.reset(new Options);
		auto _context = static_cast<Options*>(context);
		memcpy(m_calib.get(), _context, sizeof(Options));

		auto p_transform = k4a_transformation_create(m_calib.get());
		m_transform.reset(p_transform, [](k4a_transformation_t p) {if (p)  k4a_transformation_destroy(p); });

		return true;
	}

	bool AzureKinectFrameReader::get_color_3d_point(
		fVECTOR_3* out_point, const iVECTOR_2& color_xy, uint16_t depth_value, 
		int relative_to_device_component /*= DeviceComponentType::COLOR_CAMERA*/) const
	{
		if (!m_data || !m_calib || depth_value == INVALID_DEPTH_VALUE)
			return false;

		k4a_float2_t p;
		p.xy.x = color_xy.x();
		p.xy.y = color_xy.y();

		k4a_float3_t q;
		int valid = 0;
		auto target_k4a_camera = get_component_type_to_k4a_type().at(relative_to_device_component);
		k4a_calibration_2d_to_3d(m_calib.get(), &p, depth_value, 
			K4A_CALIBRATION_TYPE_COLOR, target_k4a_camera, &q, &valid);

		if (out_point)
		{
			for (int k = 0; k < 3; k++)
				(*out_point)[k] = q.v[k];
		}

		return valid;
	}

	bool AzureKinectFrameReader::get_color_3d_point(
		cv::Mat* out_color2xyz, const cv::Mat& color2depth, 
		int relative_to_device_component /*= DeviceComponentType::COLOR_CAMERA*/) const
	{
		if (!m_data || !m_calib)
			return false;

		if (!out_color2xyz)
			return true;

		auto target_k4a_camera = get_component_type_to_k4a_type().at(relative_to_device_component);
		convert_k4a_depth_to_point_cloud(out_color2xyz, nullptr, *m_calib, color2depth,
			K4A_CALIBRATION_TYPE_COLOR, target_k4a_camera);

		return true;
	}

}