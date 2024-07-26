#include <OpenniStdDevice.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/null_sink.h>
#include <igcclib/core/igcclib_logging.hpp>

namespace ni = openni;

namespace _NS_UTILITY
{
	static auto lg = Logging::get_logger(OpenniManager::LOG_KEY(), Logging::LOGKEY_CONSOLE());

	static void check_and_throw(ni::Status status)
	{
		if (status != ni::Status::STATUS_OK)
			lg->error(ni::OpenNI::getExtendedError());
		assert_throw(status == ni::Status::STATUS_OK, ni::OpenNI::getExtendedError());
	}

	void OpenniStdDevice::open()
	{
		//open device
		m_device.reset(new ni::Device);
		{
			ni::Status rc;
			if (m_opt.device_uri.empty())
			{
				lg->info("opening any device");
				rc = m_device->open(ni::ANY_DEVICE);
			}
			else
			{
				lg->info("opening device {0}", m_opt.device_uri);
				rc = m_device->open(m_opt.device_uri.c_str());
			}
				
			check_and_throw(rc);

			lg->info("enabling color depth sync");
			//sync the device
			rc = m_device->setDepthColorSyncEnabled(true);
			check_and_throw(rc);
		}

		//open video streams
		if(m_opt.use_color)
			m_stream_color.reset(new ni::VideoStream, delete_stream);
		if(m_opt.use_depth)
			m_stream_depth.reset(new ni::VideoStream, delete_stream);
		if(m_opt.use_ir)
			m_stream_ir.reset(new ni::VideoStream, delete_stream);
		
		std::map<ni::SensorType, ni::VideoStream*> id2stream = {
			{ni::SENSOR_COLOR, m_stream_color.get()},
			{ni::SENSOR_DEPTH, m_stream_depth.get()},
			{ni::SENSOR_IR, m_stream_ir.get()}
		};

		for (auto x : id2stream)
		{
			auto stream_type = x.first;
			auto stream_obj = x.second;
			if(stream_obj == nullptr)
				continue;

			lg->info("opening stream {0}", stream_type);

			auto rc = stream_obj->create(*m_device, stream_type);
			check_and_throw(rc);

			//set video mode
			boost::optional<ni::VideoMode> vmode;
			switch (stream_type)
			{
			case ni::SENSOR_COLOR:
				vmode = m_opt.color_config;
				break;
			case ni::SENSOR_DEPTH:
				vmode = m_opt.depth_config;
				break;
			case ni::SENSOR_IR:
				vmode = m_opt.ir_config;
				break;
			}

			if (vmode)
			{
				auto rc = stream_obj->setVideoMode(*vmode);
				check_and_throw(rc);
			}

			rc = stream_obj->start();
			check_and_throw(rc);
		}

		lg->info("done");
	}

	void OpenniStdDevice::destroy()
	{
		lg->info("destroying device");
		m_openni_token.reset();
		lg->info("done");
	}

	bool OpenniStdDevice::init(void* opt)
	{
		lg->info("initializing OpenNI");
		if(opt != nullptr)
			m_opt = *(Options*)(opt);
		m_openni_token = OpenniManager::request_init();
		lg->info("done");
		return true;
	}

	void OpenniStdDevice::close()
	{
		lg->info("closing streams");
		m_stream_depth.reset();
		m_stream_color.reset();
		m_stream_ir.reset();

		lg->info("closing device");
		m_device.reset();

		lg->info("done");
	}

	bool OpenniStdDevice::get_frame(FrameData* output, int frame_component /*= ~0*/)
	{
		std::vector<ni::VideoStream*> streams;
		if (m_stream_color)
			streams.push_back(m_stream_color.get());
		if (m_stream_depth)
			streams.push_back(m_stream_depth.get());
		if (m_stream_ir)
			streams.push_back(m_stream_ir.get());

		//wait
		{
			int ready_stream = -1;
			auto rc = ni::OpenNI::waitForAnyStream(streams.data(), streams.size(), &ready_stream, 5000);
			check_and_throw(rc);
		}

		bool use_color = (frame_component & (int)FrameComponent::RGBA_IMAGE_1) && m_stream_color;
		bool use_depth = (frame_component & (int)FrameComponent::DEPTH_IMAGE) && m_stream_depth;
		bool use_depth2xyz = (frame_component &(int)FrameComponent::DEPTH_TO_XYZ) && use_depth;
		bool use_depth2color = (frame_component & (int)FrameComponent::DEPTH_TO_RGBA) && use_depth && use_color;
		bool use_depth2image_xy = (frame_component & (int)FrameComponent::DEPTH_TO_IMAGE_COORDINATE) && use_depth && use_color;
		bool use_ir = (frame_component & (int)FrameComponent::INFRARED_IMAGE_1) && m_stream_ir;

		assert_throw(!use_ir || m_stream_ir, "IR stream is not opened during init");
		assert_throw(!use_color || m_stream_color, "Color stream is not opened during init");
		assert_throw(!use_depth || m_stream_depth, "Depth stream is not opened during init");

		//read ir
		if (use_ir)
		{
			ni::VideoFrameRef frame;
			m_stream_ir->readFrame(&frame);

			auto pdata = (ni::Grayscale16Pixel*)frame.getData();
			auto cv_type = CV_16UC1;
			cv::Mat img(frame.getHeight(), frame.getWidth(), cv_type, pdata);
			img.copyTo(output->ir_1);
		}

		//read color
		if (use_color)
		{
			ni::VideoFrameRef frame;
			m_stream_color->readFrame(&frame);

			auto pdata = (ni::RGB888Pixel*)frame.getData();
			auto cv_type = CV_8UC3;
			cv::Mat img(frame.getHeight(), frame.getWidth(), cv_type, pdata);
			img.copyTo(output->color_1);
		}

		//read depth
		if (use_depth)
		{
			ni::VideoFrameRef frame;
			m_stream_depth->readFrame(&frame);

			auto pdata = (ni::DepthPixel*)frame.getData();
			auto cv_type = cv::DataType<ni::DepthPixel>::type;
			cv::Mat img(frame.getHeight(), frame.getWidth(), cv_type, pdata);
			img.copyTo(output->depth);

			//update mask
			output->mask_depth = output->depth > 0;
		}

		//convert depth to xyz
		if (use_depth2xyz)
		{
			auto& dmap = output->depth;
			output->depth2xyz.create(dmap.rows, dmap.cols, CV_32FC3);
			output->depth2xyz.setTo(0);
			
			for (int i = 0; i < dmap.rows; i++)
				for (int j = 0; j < dmap.cols; j++)
				{
					if(!output->mask_depth.at<char>(i,j)) continue;

					auto d = dmap.at<ni::DepthPixel>(i, j);
					float x=0, y=0, z=0;
					ni::CoordinateConverter::convertDepthToWorld(*m_stream_depth, j, i, d, &x, &y, &z);

					auto& pix = output->depth2xyz.at<cv::Vec3f>(i, j);
					pix[0] = x;
					pix[1] = y;
					pix[2] = z;
				}
		}

		//convert depth to image coordinate
		cv::Mat depth2image_xy;
		if (use_depth2image_xy || use_depth2color)
		{
			auto& dmap = output->depth;
			depth2image_xy.create(dmap.rows, dmap.cols, CV_32FC2);
			depth2image_xy.setTo(-1);

			for (int i = 0; i < dmap.rows; i++)
				for (int j = 0; j < dmap.cols; j++)
				{
					if (!output->mask_depth.at<char>(i, j)) continue;

					auto d = dmap.at<ni::DepthPixel>(i, j);
					int x = 0, y = 0;
					ni::CoordinateConverter::convertDepthToColor(*m_stream_depth, *m_stream_color, j, i, d, &x, &y);
					auto& pix = depth2image_xy.at<cv::Vec2f>(i, j);
					pix[0] = x;
					pix[1] = y;
				}
		}

		if (use_depth2image_xy)
		{
			output->depth2image_xy = depth2image_xy;
		}

		if (use_depth2color)
		{
			auto& dmap = output->depth;
			auto& img = output->color_1;
			output->depth2color.create(dmap.rows, dmap.cols, CV_8UC3);
			output->depth2color.setTo(0);

			for (int i = 0; i < dmap.total(); i++)
			{
				if (!output->mask_depth.at<char>(i)) continue;
				auto pix = depth2image_xy.at<cv::Vec2f>(i);
				int ii = (int)pix[1];
				int jj = (int)pix[0];

				//pixel out of bound
				if (ii < 0 || ii >= img.rows || jj < 0 || jj >= img.cols)
					continue;

				//set pixel
				output->depth2color.at<cv::Vec3b>(i) = img.at<cv::Vec3b>(ii, jj);
			}
		}

		return true;
	}

	_NS_UTILITY::fMATRIX_3 OpenniStdDevice::get_depth_camera_intrinsic() const
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	_NS_UTILITY::fMATRIX_3 OpenniStdDevice::get_color_camera_intrinsic() const
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	std::vector<openni::VideoMode> OpenniStdDevice::get_color_video_modes() const
	{
		assert_throw((bool)m_stream_color, "Color stream is not open during init");
		const auto& vmodes = m_stream_color->getSensorInfo().getSupportedVideoModes();
		std::vector<openni::VideoMode> output;
		for (int i = 0; i < vmodes.getSize(); i++)
			output.push_back(vmodes[i]);
		return output;
	}

	std::vector<openni::VideoMode> OpenniStdDevice::get_depth_video_modes() const
	{
		assert_throw((bool)m_stream_depth, "Depth stream is not open during init");
		const auto& vmodes = m_stream_depth->getSensorInfo().getSupportedVideoModes();
		std::vector<openni::VideoMode> output;
		for (int i = 0; i < vmodes.getSize(); i++)
			output.push_back(vmodes[i]);
		return output;
	}

	OpenniStdDevice::~OpenniStdDevice()
	{
		close();
		destroy();
	}

	void OpenniStdDevice::delete_stream(openni::VideoStream* stream)
	{
		if (!stream) return;

		if (stream->isValid())
		{
			stream->stop();
			stream->destroy();
		}
		delete stream;
	}

	const std::string& OpenniManager::LOG_KEY()
	{
		static std::string key = "openni_logger";
		return key;
	}

	OpenniManager::Token& OpenniManager::request_init()
	{
		static Token tk;
		static auto deleter = [](int* p) {
			if (p) delete p;
			openni::OpenNI::shutdown();
		};

		if (tk == nullptr)
		{
			auto rc = openni::OpenNI::initialize();
			assert_throw(rc == openni::STATUS_OK, openni::OpenNI::getExtendedError());
			tk.reset(new int, deleter);
		}
		return tk;
	}

	void OpenniManager::request_shutdown()
	{
		auto& tk = request_init();
		tk.reset();
	}

}