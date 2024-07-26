#pragma once
#include <thread>
#include <memory>

#include "igcclib_device_def.hpp"
#include "DeviceReader.hpp"

namespace _NS_UTILITY
{
	class FrameRecorder
	{
	public:
		enum class State
		{
			VIEWING,
			RECORD_ALL_STREAM,
			CAPTURE_NEXT_FRAME,
			QUIT
		};

		enum class OutputFiletype
		{
			BINARY = 1,	//save as general binary, which should be read using cereal
			NUMPY = 1 << 1,	//save as a list of numpy arrays, in this mode, a new folder will be created for each frame
			IMAGE = 1 << 2,	//save as a list of images, in this mode, a new folder will be created for each frame
			COMPRESSED_BINARY = 1<<3, //save as binary data with compression
		};

		struct KeyboardOptions
		{
			std::string record_all_stream = "record";	//in this mode, press enter will pause/resume recording
			std::string stop_current_action = "stop";
			std::string capture_a_frame = "capture";
			std::string quit = "quit";
			std::string standard_view = "view";

			std::string output_binary = "output:binary";
			std::string output_compress = "output:compress";		//compressed binary
			std::string output_numpy = "output:numpy";
			std::string output_image = "output:image";
		};

		struct DirectoryOptions
		{
			std::string outdir_frames = "frames";
			std::string outdir_screenshot = "screenshot";
		};

	public:
		virtual void init(const DirectoryOptions* opt_dir, const KeyboardOptions* opt_keyboard = nullptr);
		virtual KeyboardOptions& get_keyboard_options() { return m_opt_keys; }
		virtual DirectoryOptions& get_directory_options() { return m_opt_dirs; }

		virtual void set_output_filetype(OutputFiletype t);;
		virtual void set_output_filetype(int t);
		int get_output_filetype() const { return m_out_filetype; }

		//remove all files in output directory
		void clear_frame_directory();
		void clear_screenshot_directory();

		//start handling keyboard input
		virtual void start();

		//stop handling keyboard input
		virtual void stop();

		//notify the recorder with a new frame
		virtual void update(const FrameData& frame);

		//get current state
		virtual State get_state() { return m_state; }

		//set current state
		virtual void set_state(State s);

		//save a frame to the output directory, and the saved file names are dependent on the output_name
		//which should be unique for different frames
		virtual void save_frame(
			const FrameData& frame, const std::string& output_dir, 
			const std::string& output_name);

	public:
		virtual ~FrameRecorder();

	protected:
		KeyboardOptions m_opt_keys;
		DirectoryOptions m_opt_dirs;
		int m_out_filetype = (int)OutputFiletype::BINARY;

		//thread for handling keyboard input
		std::shared_ptr<std::thread> m_key_thread;
		std::atomic<State> m_state{State::VIEWING};
		const std::string m_frame_basename = "frame";
		int m_frame_count = 0;	//number of recorded frame in one session
		int m_frame_captured = 0;	//count the number of captured frames in this session

		void handle_keyboard_input();
		std::string get_output_filetype_name() const;
	};
}
