#include <igcclib/vision/VideoReader.hpp>

namespace _NS_UTILITY
{
	bool VideoReader::init_from_file(const std::string filename)
	{
		return m_video.open(filename);
	}


	size_t VideoReader::get_num_frames() const
	{
		if (m_video.isOpened())
			return m_video.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
		else
			return 0;
	}

	double VideoReader::get_frame_rate() const
	{
		if (m_video.isOpened())
			return 0;
		else
			return m_video.get(cv::VideoCaptureProperties::CAP_PROP_FPS);
	}

	void VideoReader::begin_sequential_read()
	{
		m_video.set(cv::VideoCaptureProperties::CAP_PROP_POS_FRAMES, 0);
	}

	bool VideoReader::read_next_frame(cv::Mat* output)
	{
		if (output && m_video.isOpened())
		{
			return m_video.read(*output);
		}
		else return false;
	}

	bool VideoReader::read_frame_by_index(size_t n, cv::Mat* output)
	{
		if (!output)
			return false;

		assert_throw(m_video.isOpened(), "no video is opened");
		assert_throw(m_video.set(cv::CAP_PROP_POS_FRAMES, n), "failed to seek the frame");
		return m_video.read(*output);
	}

	bool VideoReader::read_frame_by_time(double t_sec, cv::Mat* output)
	{
		if (!output)
			return false;

		assert_throw(m_video.isOpened(), "no video is opened");
		m_video.set(cv::CAP_PROP_POS_MSEC, t_sec * 1000);
		return m_video.read(*output);
	}

	int VideoReader::get_frame_width() const
	{
		return m_video.get(cv::CAP_PROP_FRAME_WIDTH);
	}

	int VideoReader::get_frame_height() const
	{
		return m_video.get(cv::CAP_PROP_FRAME_HEIGHT);
	}

	double VideoReader::get_current_time_ms() const
	{
		assert_throw(m_video.isOpened(), "no video is opened");
		return m_video.get(cv::CAP_PROP_POS_MSEC);
	}

	size_t VideoReader::get_current_frame_index() const
	{
		assert_throw(m_video.isOpened(), "no video is opened");
		return (size_t)m_video.get(cv::CAP_PROP_POS_FRAMES);
	}

}