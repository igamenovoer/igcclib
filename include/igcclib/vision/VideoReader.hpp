#pragma once
#include <igcclib/vision/igcclib_opencv_def.hpp>

namespace _NS_UTILITY
{
	class VideoReader
	{
	public:
		/** \brief initialize video reader from a file. The codec is recognized by file extension.
		Return whether the operation is successful. */
		bool init_from_file(const std::string filename);

		/** \brief initialize video reader from a file already read into the memory */
		// void init_from_data(const char* data);

		/** \brief get the number of frames */
		size_t get_num_frames() const;

		/** \brief get framerate as fps. For some video, it may be 0 */
		double get_frame_rate() const;

		/** \brief prepare the video reader for sequential reading */
		void begin_sequential_read();

		/** \brief in sequential reading, read the next frame. Return true if a frame is read successfully, otherwise return false. */
		bool read_next_frame(cv::Mat* output);

		/** \brief read a specific frame. */
		bool read_frame_by_index(size_t n, cv::Mat* output);

		/** \brief read frame by a timestamp in seconds */
		bool read_frame_by_time(double t_sec, cv::Mat* output);

		int get_frame_width() const;
		int get_frame_height() const;

		/** \brief get the current time in ms of the last read frame */
		double get_current_time_ms() const;

		/** \brief get the index of the frame last read */
		size_t get_current_frame_index() const;

		virtual ~VideoReader() {}

	protected:
		cv::VideoCapture m_video;
		size_t m_frame_index = 0;
	};
}