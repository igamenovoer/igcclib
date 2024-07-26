#include "FrameRecorder_v1.h"
#include "./../io/io_filesys.h"
#include "./../io/io_numpy.h"
#include "./../io/io_numpy_opencv.h"

#include "./../common/util_logging.h"
#include "cereal/cereal.hpp"
#include "cereal/archives/portable_binary.hpp"

#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_io.hpp"
#include "boost/uuid/uuid_generators.hpp"

namespace _NS_UTILITY
{
	namespace v1
	{
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
				auto uuid = boost::uuids::random_generator()();
				std::ostringstream os;
				os << uuid;
				//Logging::get_default_logger()->info("saving screenshot {0}", os.str());

				save_frame(frame, m_opt_dirs.outdir_screenshot, os.str());
				set_state(State::VIEWING);
			}
		}

		void FrameRecorder::set_state(State s)
		{
			if (s == State::RECORD_ALL_STREAM)
				m_frame_count = 0;

			Logging::get_default_logger()->info("switched to state {0}", (int)s);
			m_state = s;
		}

		void FrameRecorder::save_frame(const FrameData& frame, const std::string& _output_dir, const std::string& _output_name)
		{
			if ((int)m_out_filetype & (int)OutputFiletype::BINARY)
			{
				std::string output_filename = _output_dir + "/" + _output_name + ".dat";
				std::ofstream outfile(output_filename, std::ios::binary | std::ios::trunc);
				cereal::PortableBinaryOutputArchive oa(outfile);
				oa(frame);
			}

			if ((int)m_out_filetype & (int)OutputFiletype::NUMPY)
			{
				std::string outdir = _output_dir + "/" + _output_name;
				make_dir(outdir);

				if (!frame.color_1.empty())
					save_np_array(outdir + "/color.npy", frame.color_1);
				if (!frame.depth.empty())
					save_np_array(outdir + "/depth.npy", frame.depth);
				if (!frame.mask_depth.empty())
					save_np_array(outdir + "/mask_depth.npy", frame.mask_depth);
				if (!frame.depth2color.empty())
					save_np_array(outdir + "/depth2color.npy", frame.depth2color);
				if (!frame.depth2image_xy.empty())
					save_np_array(outdir + "/depth2image_xy.npy", frame.depth2image_xy);
				if (!frame.depth2xyz.empty())
					save_np_array(outdir + "/depth2xyz.npy", frame.depth2xyz);
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

				if (!frame.color_1.empty())
				{
					cv::Mat tmp = func_img2bgr(frame.color_1);
					cv::imwrite(outdir + "/color.png", tmp);
				}

				if (!frame.depth.empty())
					cv::imwrite(outdir + "/depth.png", frame.depth);

				if (!frame.mask_depth.empty())
					cv::imwrite(outdir + "/mask_depth.png", frame.mask_depth);

				if (!frame.depth2color.empty())
				{
					cv::Mat tmp = func_img2bgr(frame.depth2color);
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
}
