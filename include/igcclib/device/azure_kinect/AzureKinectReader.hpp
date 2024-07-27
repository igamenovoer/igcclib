#pragma once

#include <map>
#include <igcclib/device/DeviceReader.hpp>
#include <igcclib/device/azure_kinect/igcclib_device_azure_kinect.hpp>

namespace _NS_UTILITY
{
	namespace FrameSegmentID {
		enum {
			//color image timestamp in usec (1e-6 second), as 64 bit unsigned integer.
			//only available if COLOR_IMAGE component is captured
			COLOR_IMAGE_TIMESTAMP_USEC_64,

			//like the above, but for depth image
			DEPTH_IMAGE_TIMESTAMP_USEC_64
		};
	}

	/** \brief capable of reading data from Azure Kinect */
	class AzureKinectReader : public DeviceReader
	{
	public:
		struct Options
		{
			int device_index = K4A_DEVICE_DEFAULT;
			k4a_device_configuration_t device_options;

			Options() {
				device_options = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
			}

			//quick access to different presets
			static Options preset_depth_color_1080p() {
				Options opt;
				opt.device_options.camera_fps = K4A_FRAMES_PER_SECOND_15;
				opt.device_options.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
				opt.device_options.color_resolution = K4A_COLOR_RESOLUTION_1080P;
				opt.device_options.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
				opt.device_options.synchronized_images_only = true;

				return opt;
			}
		};

		virtual bool init(void* opt) override;
		virtual void open() override;
		virtual void close() override;
		virtual void destroy() override;

		/** \brief color image format is BGRA, depth is single channel uint16, positions are 32f */
		virtual bool get_frame(FrameData* output, const std::set<int>* components, int timeout_ms = -1) override;
		virtual std::shared_ptr<DeviceComponent> get_device_component(int source_component_type, int relative_to) const override;
		virtual bool is_frame_component_available(int frame_component) const override;
		~AzureKinectReader();

		const k4a_calibration_t& get_k4a_calibration() const { return m_calib; }
		k4a_device_t get_k4a_device() const { return m_device; }
		const Options& get_options() const { return m_opt; }
		Options& get_options() { return m_opt; }

		const std::string& get_k4a_device_serial() const;

	public:
		/** \brief get the number of devices connected to computer */
		static int get_num_connected_device();
		static std::set<int> get_default_components(bool has_depth);

	private:
		/** \brief read calibration info from device */
		void update_calibration();

		/** \brief read all the cached IMU samples from the device */
		void read_imu_data(
			cv::Mat* out_accel, cv::Mat* out_accel_timestamp, 
			cv::Mat* out_gyro, cv::Mat* out_gyro_timestamp);

	private:
		Options m_opt;
		k4a_device_t m_device = nullptr;
		k4a_calibration_t m_calib;
		k4a_transformation_t m_transform = nullptr;
		std::string m_serial;	//device id

		std::map<int, AzureDeviceComponent> m_components;
	};
}