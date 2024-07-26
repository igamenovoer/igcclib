#include "AzureKinectReader.h"
#include "./../../common/util_opencv_eigen.h"

namespace _NS_UTILITY
{
	bool AzureKinectReader::init(void* opt)
	{
		m_opt = *static_cast<Options*>(opt);

		auto is_failed = K4A_FAILED(k4a_device_open(m_opt.device_index, &m_device));
		if (is_failed)
		{
			m_device = nullptr;
			return false;
		}

		//read serial
		{
			size_t len = 0;
			k4a_device_get_serialnum(m_device, nullptr, &len);

			std::vector<char> buf(len);
			k4a_device_get_serialnum(m_device, buf.data(), &len);

			m_serial = buf.data();
		}

		update_calibration();
		return true;
	}

	void AzureKinectReader::open()
	{
		//open cameras
		auto is_failed = K4A_FAILED(k4a_device_start_cameras(m_device, &m_opt.device_options));
		if (is_failed)
		{
			close();
			throw ErrFailedToOpenDevice("failed to start cameras");
		}

		//start imu
		is_failed = K4A_FAILED(k4a_device_start_imu(m_device));
		if (is_failed)
		{
			close();
			throw ErrFailedToOpenDevice("failed to start imu");
		}
	}

	void AzureKinectReader::close()
	{
		if (m_device)
		{
			k4a_device_stop_cameras(m_device);
			k4a_device_stop_imu(m_device);
		}
	}

	void AzureKinectReader::destroy()
	{
		close();
		if (m_transform)
		{
			k4a_transformation_destroy(m_transform);
			m_transform = nullptr;
		}

		if (m_device)
		{
			k4a_device_close(m_device);
			m_device = nullptr;
		}
		m_serial = "";
	}

	bool AzureKinectReader::get_frame(FrameData* output, const std::set<int>* components, int timeout_ms /*= -1*/)
	{
		//depth and color are read regardless of whether they are in components
		if (!output)
			return false;	

		//use default frame components if not specified
		bool has_depth_sensor = m_opt.device_options.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR;
		std::set<int> flags;
		if (components == nullptr)
			flags = get_default_components(has_depth_sensor);
		else
			flags = *components;

		auto has_key = [&flags](int key) {
			return flags.find(key) != flags.end();
		};

		AzureSharedPtr<k4a_capture_t> capture;
		{
			k4a_capture_t _capture = nullptr;
			if (timeout_ms < 0)
				timeout_ms = K4A_WAIT_INFINITE;
			auto res_capture = k4a_device_get_capture(m_device, &_capture, timeout_ms);
			if (res_capture != K4A_WAIT_RESULT_SUCCEEDED)
				return false;

			capture.reset(_capture, [](k4a_capture_t p) {if (p) k4a_capture_release(p); });
		}


		//once capture, read imu data
		{
			cv::Mat gyro, gyro_ts, accel, accel_ts;
			read_imu_data(&accel, &accel_ts, &gyro, &gyro_ts);

			if (has_key(FrameSegmentID::IMU_ACCELEROMETER_SEQ))
				output->set_segment(FrameSegmentID::IMU_ACCELEROMETER_SEQ, accel);

			if (has_key(FrameSegmentID::IMU_ACCELEROMETER_TIMESTAMP))
				output->set_segment(FrameSegmentID::IMU_ACCELEROMETER_TIMESTAMP, accel_ts);

			if (has_key(FrameSegmentID::IMU_GYROSCOPE_SEQ))
				output->set_segment(FrameSegmentID::IMU_GYROSCOPE_SEQ, gyro);

			if (has_key(FrameSegmentID::IMU_GYROSCOPE_TIMESTAMP))
				output->set_segment(FrameSegmentID::IMU_GYROSCOPE_TIMESTAMP, gyro_ts);
		}

		//get from framedata so that we may be able to reuse allocated memory
		cv::Mat color_image = *output->get_or_create_segment(FrameSegmentID::COLOR_IMAGE);
		{
			auto k_image = k4a_capture_get_color_image(capture.get());
			convert_k4a_image_to_cvmat(&color_image, k_image);

			if (has_key(FrameSegmentID::COLOR_IMAGE_TIMESTAMP_USEC_64))
			{
				auto t = output->get_or_create_segment(FrameSegmentID::COLOR_IMAGE_TIMESTAMP_USEC_64);
				t->create(1, 1, CV_32SC2);

				auto x = k4a_image_get_timestamp_usec(k_image);
				t->at<uint64_t>(0) = x;
			}

			k4a_image_release(k_image);
		}
		output->set_segment(FrameSegmentID::COLOR_IMAGE, color_image);

		cv::Mat depth_image = *output->get_or_create_segment(FrameSegmentID::DEPTH_IMAGE);
		{
			auto k_image = k4a_capture_get_depth_image(capture.get());
			convert_k4a_image_to_cvmat(&depth_image, k_image);

			if (has_key(FrameSegmentID::DEPTH_IMAGE_TIMESTAMP_USEC_64))
			{
				auto t = output->get_or_create_segment(FrameSegmentID::DEPTH_IMAGE_TIMESTAMP_USEC_64);
				t->create(1, 1, CV_32SC2);

				auto x = k4a_image_get_timestamp_usec(k_image);
				t->at<uint64_t>(0) = x;
			}

			k4a_image_release(k_image);
		}
		output->set_segment(FrameSegmentID::DEPTH_IMAGE, depth_image);

		//transform depth to color space
		cv::Mat color_to_depth_value = *output->get_or_create_segment(FrameSegmentID::COLOR_TO_DEPTH_VALUE);
		color_to_depth_value.create(color_image.rows, color_image.cols, depth_image.type());
		{
			bool use_this = has_key(FrameSegmentID::COLOR_TO_DEPTH_VALUE) || has_key(FrameSegmentID::COLOR_TO_XYZ);
			if (use_this)
			{
				auto img_output = wrap_cvmat_to_k4a_image(color_to_depth_value);
				auto depth = wrap_cvmat_to_k4a_image(depth_image);
				//int stride = k4a_image_get_stride_bytes(img_output.get());
				auto ok = !K4A_FAILED(k4a_transformation_depth_image_to_color_camera(m_transform, depth.get(), img_output.get()));
				assert_throw(ok, "failed to transform depth to color camera space");
			}
		}
		if (has_key(FrameSegmentID::COLOR_TO_DEPTH_VALUE))
			output->set_segment(FrameSegmentID::COLOR_TO_DEPTH_VALUE, color_to_depth_value);

		//get point cloud in color space
		cv::Mat color_to_xyz = *output->get_or_create_segment(FrameSegmentID::COLOR_TO_XYZ);
		if(has_key(FrameSegmentID::COLOR_TO_XYZ))
		{
			convert_k4a_depth_to_point_cloud(
				&color_to_xyz, nullptr, m_calib, color_to_depth_value,
				K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR
			);
			output->set_segment(FrameSegmentID::COLOR_TO_XYZ, color_to_xyz);
		}

		//depth to color coordinate map
		cv::Mat depth_to_color_xy = *output->get_or_create_segment(FrameSegmentID::DEPTH_TO_COLOR_XY);
		if (has_key(FrameSegmentID::DEPTH_TO_COLOR_XY))
		{
			convert_k4a_depth_to_image_coordinate(
				&depth_to_color_xy, nullptr, m_calib, depth_image,
				K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR);
			output->set_segment(FrameSegmentID::DEPTH_TO_COLOR_XY, depth_to_color_xy);
		}

		//depth to color value
		cv::Mat depth_to_color_value = *output->get_or_create_segment(FrameSegmentID::DEPTH_TO_COLOR_VALUE);
		if (has_key(FrameSegmentID::DEPTH_TO_COLOR_VALUE))
		{
			depth_to_color_value.create(depth_image.rows, depth_image.cols, color_image.type());
			depth_to_color_value.setTo(cv::Scalar::all(0));

			auto _color_image = wrap_cvmat_to_k4a_image(color_image);
			auto _depth_image = wrap_cvmat_to_k4a_image(depth_image);
			auto _output = wrap_cvmat_to_k4a_image(depth_to_color_value);
			k4a_transformation_color_image_to_depth_camera(
				m_transform, _depth_image.get(), _color_image.get(), _output.get()
			);

			output->set_segment(FrameSegmentID::DEPTH_TO_COLOR_VALUE, depth_to_color_value);
		}

		//depth to xyz
		cv::Mat depth_to_xyz = *output->get_or_create_segment(FrameSegmentID::DEPTH_TO_XYZ);
		if(has_key(FrameSegmentID::DEPTH_TO_XYZ))
		{
			convert_k4a_depth_to_point_cloud(
				&depth_to_xyz, nullptr, m_calib, depth_image,
				K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH);
			output->set_segment(FrameSegmentID::DEPTH_TO_XYZ, depth_to_xyz);
		}

		return true;
	}

	AzureKinectReader::~AzureKinectReader()
	{
		close();
		destroy();
	}

	std::shared_ptr<DeviceComponent> AzureKinectReader::get_device_component(int source_component_type, int relative_to) const
	{
		auto it = m_components.find(source_component_type);
		if (it == m_components.end())
			return nullptr;

		const auto& comp2calib = get_component_type_to_k4a_type();
		auto output = std::make_shared<AzureDeviceComponent>();
		*output = it->second;

		//get extrinsic matrix
		cv::Mat extr;
		get_k4a_camera_parameter(nullptr, &extr, nullptr, m_calib,
			output->get_calibration_type(),
			comp2calib.at(relative_to)
		);

		//set to the device component
		fMATRIX_4 extmat = fMATRIX_4::Identity();
		extmat.block(0, 0, 4, 3) = Eigen::Map<MATRIX_f>((float*)extr.data, 4, 3).cast<float_type>();
		output->set_extrinsic_matrix(extmat);

		return output;
	}

	bool AzureKinectReader::is_frame_component_available(int frame_component) const
	{
		//all components are available
		return true;
	}

	const std::string& AzureKinectReader::get_k4a_device_serial() const
	{
		return m_serial;
	}

	int AzureKinectReader::get_num_connected_device()
	{
		return k4a_device_get_installed_count();
	}

	std::set<int> AzureKinectReader::get_default_components(bool has_depth)
	{
		std::set<int> flags = {
			(int)FrameSegmentID::COLOR_IMAGE,
			//(int)FrameSegmentID::COLOR_CAMERA_INTRINSIC,
			//(int)FrameSegmentID::COLOR_CAMERA_EXTRINSIC,
			//(int)FrameSegmentID::COLOR_CAMERA_DISTORTION,
			//(int)FrameSegmentID::DEPTH_CAMERA_INTRINSIC,
			//(int)FrameSegmentID::DEPTH_CAMERA_EXTRINSIC,
		};

		if (has_depth)
		{
			//we have depth
			flags.insert({
				(int)FrameSegmentID::DEPTH_IMAGE,
				(int)FrameSegmentID::DEPTH_TO_COLOR_XY,
				(int)FrameSegmentID::DEPTH_TO_XYZ
				});
		}
		return flags;
	}

	void AzureKinectReader::update_calibration()
	{
		auto res = k4a_device_get_calibration(
			m_device, m_opt.device_options.depth_mode, 
			m_opt.device_options.color_resolution, &m_calib);

		assert_throw(!K4A_FAILED(res), "failed to get calibration");

		m_transform = k4a_transformation_create(&m_calib);
		const auto& complist = get_component_type_to_k4a_type();
		k4a_calibration_type_t ref_type = K4A_CALIBRATION_TYPE_DEPTH;

		//read intrinsic/extrinsic
		m_components.clear();
		for(auto& x: complist)
		{
			cv::Mat intr, extr, dist;
			auto comp_type = x.first;
			auto kinect_type = x.second;
			AzureDeviceComponent comp;
			get_k4a_device_component(&comp, m_calib, kinect_type, ref_type);
			m_components[comp_type] = comp;
		}
	}

	void AzureKinectReader::read_imu_data(
		cv::Mat* out_accel, cv::Mat* out_accel_timestamp, 
		cv::Mat* out_gyro, cv::Mat* out_gyro_timestamp)
	{
		std::vector<k4a_imu_sample_t> samples;
		k4a_imu_sample_t sp;
		while (!K4A_FAILED(k4a_device_get_imu_sample(m_device, &sp, 0)))
			samples.push_back(sp);

		//fill the data
		if (out_accel)
		{
			out_accel->create(1, samples.size(), CV_32FC3);
			for (int i = 0; i < samples.size(); i++) {
				auto& p = out_accel->at<cv::Vec3f>(i);
				auto& s = samples[i];
				for (int k = 0; k < 3; k++)
					p[k] = s.acc_sample.v[k];
			}
		}

		if (out_accel_timestamp)
		{
			std::vector<uint64_t> ts(samples.size());
			for (int i = 0; i < samples.size(); i++)
				ts[i] = samples[i].acc_timestamp_usec;
			FrameData::write_timestamp(out_accel_timestamp, ts.data(), ts.size());
		}

		if (out_gyro)
		{
			out_gyro->create(1, samples.size(), CV_32FC3);
			for (int i = 0; i < samples.size(); i++)
			{
				auto& p = out_gyro->at<cv::Vec3f>(i);
				auto& s = samples[i];
				for (int k = 0; k < 3; k++)
					p[k] = s.gyro_sample.v[k];
			}
		}

		if (out_gyro_timestamp)
		{
			std::vector<uint64_t> ts(samples.size());
			for (int i = 0; i < samples.size(); i++)
				ts[i] = samples[i].gyro_timestamp_usec;
			FrameData::write_timestamp(out_gyro_timestamp, ts.data(), ts.size());
		}
	}
}


