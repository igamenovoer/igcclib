#pragma once

#include <igcclib/device/igcclib_device_def.hpp>
#include <igcclib/device/DeviceReader.hpp>
#include "OpenNI.h"
#include <boost/optional/optional.hpp>
#include <string>

namespace _NS_UTILITY
{
	class OpenniManager
	{
	public:
		using Token = std::shared_ptr<int>;

		static Token& request_init();
		static void request_shutdown();
		static const std::string& LOG_KEY();
	};

	class OpenniStdDevice : public DeviceReader
	{
	public:

		struct Options {
			std::string device_uri;	//if not specified, use ANY_DEVICE

			boost::optional<openni::VideoMode> depth_config;
			bool use_depth = true;

			boost::optional<openni::VideoMode> color_config;
			bool use_color = true;

			boost::optional<openni::VideoMode> ir_config;
			bool use_ir = false;


		};

		virtual void open() override;
		virtual void destroy() override;
		virtual bool init(void* opt) override;
		virtual void close() override;
		virtual bool get_frame(FrameData* output, int frame_component = ~0) override;
		virtual fMATRIX_3 get_depth_camera_intrinsic() const override;
		virtual fMATRIX_3 get_color_camera_intrinsic() const override;

		std::vector<openni::VideoMode> get_color_video_modes() const;
		std::vector<openni::VideoMode> get_depth_video_modes() const;

	public:
		~OpenniStdDevice();

	private:
		Options m_opt;	//initialization options
		OpenniManager::Token m_openni_token;
		std::shared_ptr<openni::Device> m_device;
		std::shared_ptr<openni::VideoStream> m_stream_depth, m_stream_color, m_stream_ir;
		
		static void delete_stream(openni::VideoStream* stream);
	};
}