#include "DefaultFrameReader.h"
#include "./../common/util_opencv_eigen.h"

namespace _NS_UTILITY
{

	bool DefaultFrameReader::get_depth_image(cv::Mat* output) const
	{
		if (!m_data)
			return false;

		auto dmap = m_depth_image;
		if (dmap.empty())
			return false;

		if (output)
			* output = dmap;
		return true;
	}

	bool DefaultFrameReader::get_color_coordinate_for_depth(fVECTOR_2* out_color_xy, const iVECTOR_2& depth_xy) const
	{
		if (!m_data)
			return false;

		if (m_depth2image_xy.empty())
			return false;
		auto x = depth_xy[0];
		auto y = depth_xy[1];

		auto dmap = m_depth_image;

		//coordinate in range?
		if (x < 0 || x >= dmap.cols || y < 0 || y >= dmap.rows)
			return false;

		if (!dmap.empty() && dmap.at<DepthValue_t>(y, x) == INVALID_DEPTH_VALUE)
			return false;
		else
		{
			auto xy = m_depth2image_xy.at<cv::Vec2f>(y, x);
			if (out_color_xy)
			{
				out_color_xy->x() = xy[0];
				out_color_xy->y() = xy[1];
			}
			return true;
		}
	}

	bool DefaultFrameReader::get_color_coordinate_for_depth(cv::Mat* out_depth2color_xy, cv::Mat* out_mask) const
	{
		if (!m_data)
			return false;

		if (out_depth2color_xy && m_depth2image_xy.empty())
			return false;
		if (out_mask && m_depth_image.empty())
			return false;

		if (out_depth2color_xy)
			m_depth2image_xy.copyTo(*out_depth2color_xy);
		if (out_mask)
			* out_mask = m_depth_image != 0;
		return true;
	}

	bool DefaultFrameReader::get_color_value_for_depth(iVECTOR_4* out_color_value, const iVECTOR_2& depth_xy, ImageFormat fmt /*= ImageFormat::NONE*/) const
	{
		if (!m_data)
			return false;

		if (m_color_image.empty())
			return false;

		if (!out_color_value)
			return true;

		//check depth validity if we have depth map
		if (!m_depth_image.empty())
		{
			auto x = depth_xy[0];
			auto y = depth_xy[1];
			if (x < 0 || x >= m_depth_image.cols || y < 0 || y >= m_depth_image.rows)
				return false;
			if (m_depth_image.at<DepthValue_t>(y, x) == INVALID_DEPTH_VALUE)
				return false;
		}

		iVECTOR_4 x{ 0,0,0,0 };

		//do we have the mapping?
		if (!m_depth2color.empty())
		{
			auto i = depth_xy[1];
			auto j = depth_xy[0];

			//check range
			if (i < 0 || i >= m_depth2color.rows || j < 0 || j >= m_depth2color.cols)
				return false;

			//in default reader, we just copy the color value
			auto p = m_depth2color.data + (i * m_depth2color.cols + j) * m_depth2color.channels();
			for (int k = 0; k < m_depth2color.channels(); k++)
				x[k] = p[k];
		}
		else
		{
			if (m_color_image.empty())
				return false;

			//read data from the coordinate map
			fVECTOR_2 color_xy;
			bool ok = get_color_coordinate_for_depth(&color_xy, depth_xy);
			if (!ok)
				return false;

			int i = (int)color_xy[1];
			int j = (int)color_xy[0];

			//check range
			if (i < 0 || i >= m_color_image.rows || j < 0 || j >= m_color_image.cols)
				return false;

			//read
			auto p = m_color_image.data + (i * m_color_image.cols + j) * m_color_image.channels();
			for (int k = 0; k < m_color_image.channels(); k++)
				x[k] = p[k];
		}

		*out_color_value = x;
		return true;
	}

	bool DefaultFrameReader::get_color_value_for_depth(cv::Mat* out_depth2color, cv::Mat* out_mask, ImageFormat fmt /*= ImageFormat::NONE*/) const
	{
		if (!m_data)
			return false;

		if (m_depth2color.empty() && m_depth2image_xy.empty())
			return false;

		if (m_depth_image.empty() || m_color_image.empty())
			return false;

		if (out_mask)
			* out_mask = m_depth_image != INVALID_DEPTH_VALUE;

		if (out_depth2color)
		{
			if (!m_depth2color.empty())
				m_depth2color.copyTo(*out_depth2color);
			else
			{
				out_depth2color->create(m_depth_image.rows, m_depth_image.cols, m_color_image.type());
				out_depth2color->setTo(cv::Scalar::all(0));

#pragma omp parallel for
				for(int ii=0; ii<m_depth_image.rows; ii++)
					for (int jj = 0; jj < m_depth_image.cols; jj++)
					{
						auto p = m_depth2image_xy.at<cv::Vec2f>(ii, jj);
						int x = (int)p[0];
						int y = (int)p[1];

						if(x == INVALID_FLOAT32_VALUE || y == INVALID_FLOAT32_VALUE)
							continue;

						auto write = out_depth2color->data + (ii * out_depth2color->cols + jj) * out_depth2color->channels();
						
						if (x >= 0 && x < m_color_image.cols && y >= 0 && y < m_color_image.rows)
						{
							auto read = m_color_image.data + (y * m_color_image.cols + x) * m_color_image.channels();
							memcpy(write, read, m_color_image.channels());

							if (out_mask)
								out_mask->at<uint8_t>(ii, jj) = 255;
						}
						else
						{
							if (out_mask)
								out_mask->at<uint8_t>(ii, jj) = 0;
						}
					}
			}
		}
		return true;
	}

	CameraModel DefaultFrameReader::get_depth_camera() const
	{
		CameraModel output;
		if (!m_data)
			return output;

		if (!m_depth_intrinsic.empty())
		{
			fMATRIX_3 intmat;
			to_matrix(m_depth_intrinsic, intmat);
			output.set_projection_matrix(intmat);
		}

		if (!m_depth_extrinsic.empty())
		{
			fMATRIX _extmat;
			to_matrix(m_depth_extrinsic, _extmat);
			fMATRIX_4 extmat = fMATRIX_4::Identity();
			extmat.block(0, 0, 4, 3) = _extmat;
			output.set_extrinsic_matrix(extmat);
		}

		return output;
	}

	bool DefaultFrameReader::get_depth_3d_point(fVECTOR_3* out_point, const iVECTOR_2& depth_xy) const
	{
		assert_throw(!m_depth2xyz.empty() && !m_depth_image.empty(), "depth2xyz is empty or depth map is empty");

		//check range
		auto x = depth_xy[0];
		auto y = depth_xy[1];

		bool in_range = x >= 0 && x < m_depth2xyz.cols && y >= 0 && y < m_depth2xyz.rows;
		assert_throw(in_range, "coordinate out of range");

		bool is_valid = m_depth_image.at<DepthValue_t>(y, x) != 0;
		if (!is_valid)
			return false;

		auto v = m_depth2xyz.at<cv::Vec3f>(y, x);
		if (out_point)
		{
			out_point->x() = v[0];
			out_point->y() = v[1];
			out_point->z() = v[2];
		}
		return true;
	}

	bool DefaultFrameReader::get_color_image(cv::Mat* output, ImageFormat fmt /*= ImageFormat::NONE*/) const
	{
		//assert_throw(!m_color_image.empty(), "color image is not available");
		if (m_color_image.empty())
			return false;

		if (!output)
			return true;

		if (fmt == ImageFormat::NONE)
		{
			*output = m_color_image;
			return true;
		}

		auto n_chn_target = get_num_channel(fmt);
		auto n_chn_source = m_color_image.channels();

		if (n_chn_target == n_chn_source)
			*output = m_color_image;
		else
		{
			cv::Mat img;
			if (n_chn_source == 1)
			{
				if (n_chn_target == 3)
					cv::cvtColor(m_color_image, img, cv::COLOR_GRAY2RGB);
				else if(n_chn_target == 4)
					cv::cvtColor(m_color_image, img, cv::COLOR_GRAY2RGBA);
			}
			else if (n_chn_source == 3)
			{
				if (n_chn_target == 1)
					cv::cvtColor(m_color_image, img, cv::COLOR_RGB2GRAY);
				else if(n_chn_target == 4)
					cv::cvtColor(m_color_image, img, cv::COLOR_RGB2RGBA);
			}
			else if (n_chn_source == 4)
			{
				if (n_chn_target == 1)
					cv::cvtColor(m_color_image, img, cv::COLOR_RGBA2GRAY);
				else if (n_chn_target == 3)
					cv::cvtColor(m_color_image, img, cv::COLOR_RGBA2RGB);
			}
			
			*output = img;
		}

		return true;
	}

	void DefaultFrameReader::set_framedata(FrameData::Ptr data)
	{
		FrameReader::set_framedata(data);
		auto read_segment = [&data](int segid) {
			cv::Mat output;
			auto x = data->get_segment(segid);
			if (x)
				output = *x;

			return output;
		};

		if (m_data)
		{
			m_color_image = read_segment(FrameSegmentID::COLOR_IMAGE);
			m_depth_image = read_segment((int)FrameSegmentID::DEPTH_IMAGE);
			m_depth2image_xy = read_segment((int)FrameSegmentID::DEPTH_TO_COLOR_XY);
			m_depth2xyz = read_segment((int)FrameSegmentID::DEPTH_TO_XYZ);
			m_depth2color = read_segment((int)FrameSegmentID::DEPTH_TO_COLOR_VALUE);
			m_color_intrinsic = read_segment((int)FrameSegmentID::COLOR_CAMERA_INTRINSIC);
			m_color_extrinsic = read_segment((int)FrameSegmentID::COLOR_CAMERA_EXTRINSIC);
			m_depth_intrinsic = read_segment((int)FrameSegmentID::DEPTH_CAMERA_INTRINSIC);
			m_depth_extrinsic = read_segment((int)FrameSegmentID::DEPTH_CAMERA_EXTRINSIC);
		}
		else
		{
			m_color_image = cv::Mat();
			m_depth_image = cv::Mat();
			m_depth2image_xy = cv::Mat();
			m_depth2xyz = cv::Mat();
			m_depth2color = cv::Mat();
			m_color_intrinsic = cv::Mat();
			m_color_extrinsic = cv::Mat();
			m_depth_intrinsic = cv::Mat();
			m_depth_extrinsic = cv::Mat();
		}
	}

	CameraModel DefaultFrameReader::get_color_camera() const
	{
		CameraModel output;
		if (!m_data)
			return output;

		if (!m_color_intrinsic.empty())
		{
			fMATRIX_3 intmat;
			to_matrix(m_color_intrinsic, intmat);
			output.set_projection_matrix(intmat);
		}

		if (!m_color_extrinsic.empty())
		{
			fMATRIX _extmat;
			to_matrix(m_color_extrinsic, _extmat);
			fMATRIX_4 extmat = fMATRIX_4::Identity();
			extmat.block(0, 0, 4, 3) = _extmat;
			output.set_extrinsic_matrix(extmat);
		}

		return output;
	}

	bool DefaultFrameReader::init(void* context)
	{
		return true;
	}

}