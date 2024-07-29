#include <igcclib/device/FrameRecorder.hpp>
#include <igcclib/io/igcclib_io_filesys.hpp>
#include <igcclib/io/igcclib_io_numpy.hpp>
#include <igcclib/io/igcclib_io_numpy_opencv.hpp>

#ifdef FRAME_RECORDER_WITH_COMPRESSION
	#include <igcclib/io/igcclib_io_compression.hpp>
#endif

#include <igcclib/core/igcclib_logging.hpp>
#include <cereal/cereal.hpp>
#include <cereal/archives/portable_binary.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace _NS_UTILITY
{
	static std::map<FrameRecorder::State, std::string> state2name{
		{FrameRecorder::State::CAPTURE_NEXT_FRAME, "screenshot captureing"},
		{FrameRecorder::State::QUIT, "quit"},
		{FrameRecorder::State::RECORD_ALL_STREAM, "recording"},
		{FrameRecorder::State::VIEWING, "idle"}
	};

	void FrameRecorder::init(const DirectoryOptions* opt_dir, const KeyboardOptions* opt_keyboard /*= nullptr*/)
	{
		if (opt_dir)
			m_opt_dirs = *opt_dir;

		if (opt_keyboard)
			m_opt_keys = *opt_keyboard;

		//make sure the directory exists
		if (!m_opt_dirs.outdir_frames.empty())
			make_dir(m_opt_dirs.outdir_frames);

		if (!m_opt_dirs.outdir_screenshot.empty())
			make_dir(m_opt_dirs.outdir_screenshot);
	}

	FrameRecorder::~FrameRecorder()
	{
		stop();
	}

	void FrameRecorder::set_output_filetype(OutputFiletype t)
	{
		m_out_filetype = (int)t;
		std::string name = get_output_filetype_name();
		Logging::get_default_logger()->info("output file type switched to {0}", name);
	}

	void FrameRecorder::set_output_filetype(int t)
	{
		m_out_filetype = t;
		std::string name = get_output_filetype_name();
		Logging::get_default_logger()->info("output file type switched to {0}", name);
	}

	void FrameRecorder::clear_frame_directory()
	{
		remove_all_in_dir(m_opt_dirs.outdir_frames);
	}

	void FrameRecorder::clear_screenshot_directory()
	{
		remove_all_in_dir(m_opt_dirs.outdir_screenshot);
	}

	void FrameRecorder::start()
	{
		auto thfunc = [=]() {this->handle_keyboard_input(); };
		m_key_thread.reset(new std::thread(thfunc));
	}

	void FrameRecorder::stop()
	{
		auto lg = Logging::get_default_logger();

		lg->info("waiting keyboard thread to exit, please press {0} and enter ...", m_opt_keys.quit);
		if (m_key_thread)
			m_key_thread->join();
		lg->info("keyboard thread exited");

		m_key_thread.reset();
	}

	void FrameRecorder::update(const FrameData& frame)
	{
		if (m_state == State::RECORD_ALL_STREAM)
		{
			//Logging::get_default_logger()->info("recording frame {0}", m_frame_count);

			std::ostringstream os;
			os << m_frame_basename << "-" << m_frame_count;
			m_frame_count++;

			save_frame(frame, m_opt_dirs.outdir_frames, os.str());
		}

		if (m_state == State::CAPTURE_NEXT_FRAME)
		{
			auto t = std::chrono::system_clock::now();
			auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
			std::ostringstream os;
			os << dt.count();

			//auto uuid = boost::uuids::random_generator()();
			//std::ostringstream os;
			//os << "["<<m_frame_captured++<<"]"<<uuid;
			//Logging::get_default_logger()->info("saving screenshot {0}", os.str());

			save_frame(frame, m_opt_dirs.outdir_screenshot, os.str());
			set_state(State::VIEWING);
		}
	}

	void FrameRecorder::set_state(State s)
	{
		if (s == State::RECORD_ALL_STREAM)
			m_frame_count = 0;

		Logging::get_default_logger()->info("switched to {0}", state2name[s]);
		m_state = s;
	}

	void FrameRecorder::save_frame(
		const FrameData& frame, const std::string& _output_dir, 
		const std::string& _output_name)
	{
		if ((int)m_out_filetype & (int)OutputFiletype::BINARY)
		{
			std::string output_filename = _output_dir + "/" + _output_name + ".dat";
			std::ofstream outfile(output_filename, std::ios::binary | std::ios::trunc);
			cereal::PortableBinaryOutputArchive oa(outfile);
			oa(frame);
		}

#ifdef FRAME_RECORDER_WITH_COMPRESSION
		if ((int)m_out_filetype & (int)OutputFiletype::COMPRESSED_BINARY)
		{
			std::string output_filename = _output_dir + "/" + _output_name + ".cdat";
			save_binary_by_cereal_compressed(output_filename, frame, 1);
		}
#endif

		//read frame data
		auto get_segment = [&frame](int segid) {
			//get a segment or return empty cvmat if it does not exist
			auto x = frame.get_segment(segid);
			if (x)
				return *x;
			else
				return cv::Mat();
		};

		auto color_image = get_segment(FrameSegmentID::COLOR_IMAGE);
		auto color2xyz = get_segment(FrameSegmentID::COLOR_TO_XYZ);
		auto color2depth = get_segment(FrameSegmentID::COLOR_TO_DEPTH_VALUE);

		auto depth_image = get_segment(FrameSegmentID::DEPTH_IMAGE);
		cv::Mat depth_mask;
		if(!depth_image.empty())
			depth_mask = depth_image > 0;
		auto depth2color = get_segment(FrameSegmentID::DEPTH_TO_COLOR_VALUE);
		auto depth2image_xy = get_segment(FrameSegmentID::DEPTH_TO_COLOR_XY);
		auto depth2xyz = get_segment(FrameSegmentID::DEPTH_TO_XYZ);

		//IMU data
		auto imu_accel_seq = get_segment(FrameSegmentID::IMU_ACCELEROMETER_SEQ);
		std::vector<uint64_t> imu_accel_time;
		{
			auto _imu_accel_time = get_segment(FrameSegmentID::IMU_ACCELEROMETER_TIMESTAMP);
			if (!_imu_accel_time.empty())
				imu_accel_time = FrameData::read_timestamp_u64(_imu_accel_time);
		}
		
		auto imu_gyro_seq = get_segment(FrameSegmentID::IMU_GYROSCOPE_SEQ);
		std::vector<uint64_t> imu_gyro_time;
		{
			auto _imu_gyro_time = get_segment(FrameSegmentID::IMU_GYROSCOPE_TIMESTAMP);
			if (!_imu_gyro_time.empty())
				imu_gyro_time = FrameData::read_timestamp_u64(_imu_gyro_time);
		}
		
		if ((int)m_out_filetype & (int)OutputFiletype::NUMPY)
		{
			std::string outdir = _output_dir + "/" + _output_name;
			make_dir(outdir);

			if (!color_image.empty())
				save_np_array(outdir + "/color.npy", color_image);
			if (!depth_image.empty())
				save_np_array(outdir + "/depth.npy", depth_image);
			if (!depth_mask.empty())
				save_np_array(outdir + "/mask_depth.npy", depth_mask);
			if (!depth2color.empty())
				save_np_array(outdir + "/depth2color.npy", depth2color);
			if (!depth2image_xy.empty())
				save_np_array(outdir + "/depth2image_xy.npy", depth2image_xy);
			if (!depth2xyz.empty())
				save_np_array(outdir + "/depth2xyz.npy", depth2xyz);
			if(!color2xyz.empty())
				save_np_array(outdir + "/color2xyz.npy", color2xyz);
			if(!color2depth.empty())
				save_np_array(outdir + "/color2depth.npy", color2depth);
			if (!imu_accel_seq.empty())
				save_np_array(outdir + "/imu_accel_seq.npy", imu_accel_seq);
			if (!imu_accel_time.empty())
				save_np_array(outdir + "/imu_accel_time.npy", imu_accel_time);
			if (!imu_gyro_seq.empty())
				save_np_array(outdir + "/imu_gyro_seq.npy", imu_gyro_seq);
			if (!imu_gyro_time.empty())
				save_np_array(outdir + "/imu_gyro_time.npy", imu_gyro_time);
		}

		if ((int)m_out_filetype & (int)OutputFiletype::IMAGE)
		{
			std::string outdir = _output_dir + "/" + _output_name;
			make_dir(outdir);

			auto func_img2bgr = [](cv::Mat img) {
				cv::Mat tmp;
				if (img.channels() == 3)
					cv::cvtColor(img, tmp, cv::COLOR_RGB2BGR);
				else if (img.channels() == 4)
					cv::cvtColor(img, tmp, cv::COLOR_RGBA2BGR);
				return tmp;
			};

			if (!color_image.empty())
			{
				cv::Mat tmp = func_img2bgr(color_image);
				cv::imwrite(outdir + "/color.png", tmp);
			}

			if (!color2depth.empty())
				cv::imwrite(outdir + "/color2depth.png", color2depth);

			if (!depth_image.empty())
				cv::imwrite(outdir + "/depth.png", depth_image);

			if (!depth_mask.empty())
				cv::imwrite(outdir + "/mask_depth.png", depth_mask);

			if (!depth2color.empty())
			{
				cv::Mat tmp = func_img2bgr(depth2color);
				cv::imwrite(outdir + "/depth2color.png", tmp);
			}
		}
	}

	void FrameRecorder::handle_keyboard_input()
	{
		std::string cmd;
		while (m_state != State::QUIT)
		{
			std::cin >> cmd;

			if (cmd == m_opt_keys.capture_a_frame)
				set_state(State::CAPTURE_NEXT_FRAME);
			else if (cmd == m_opt_keys.quit)
				set_state(State::QUIT);
			else if (cmd == m_opt_keys.record_all_stream)
				set_state(State::RECORD_ALL_STREAM);
			else if (cmd == m_opt_keys.standard_view)
				set_state(State::VIEWING);
			else if (cmd == m_opt_keys.stop_current_action)
				set_state(State::VIEWING);

			else if (cmd == m_opt_keys.output_binary)
				set_output_filetype(OutputFiletype::BINARY);
			else if (cmd == m_opt_keys.output_image)
				set_output_filetype(OutputFiletype::IMAGE);
			else if (cmd == m_opt_keys.output_numpy)
				set_output_filetype(OutputFiletype::NUMPY);
			else if (cmd == m_opt_keys.output_compress)
				set_output_filetype(OutputFiletype::COMPRESSED_BINARY);
		}
	}

	std::string FrameRecorder::get_output_filetype_name() const
	{
		std::vector<std::string> names;
		if (m_out_filetype & (int)OutputFiletype::BINARY)
			names.emplace_back("binary");
		if (m_out_filetype & (int)OutputFiletype::IMAGE)
			names.emplace_back("image");
		if (m_out_filetype & (int)OutputFiletype::NUMPY)
			names.emplace_back("numpy");
		if (m_out_filetype & (int)OutputFiletype::COMPRESSED_BINARY)
			names.emplace_back("compressed_binary");

		std::string output;
		for (size_t i = 0; i < names.size(); i++)
		{
			output += names[i];
			if (i + 1 < names.size())
				output += "+";
		}
		return output;
	}

}
