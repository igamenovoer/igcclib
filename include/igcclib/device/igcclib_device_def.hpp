#pragma once

#include <igcclib/igcclib_master.hpp>
#include <igcclib/vision/igcclib_opencv_def.hpp>
#include <igcclib/vision/ImageData.hpp>
#include <igcclib/core/igcclib_eigen_def.hpp>
#include <igcclib/vision/CameraModel.hpp>

namespace _NS_UTILITY {
	namespace DeviceComponentType
	{
		/** \brief device type that you can extend in your subclass. */
		enum
		{
			NONE = -1, //representing unknown component
			COLOR_CAMERA = 10000,
			DEPTH_CAMERA,
			IMU_ACCELEROMETER,
			IMU_GYROSCOPE
		};
	}

	/** \brief representing a device component */
	class IGCCLIB_API DeviceComponent : public CameraModel
	{
	public:
		using CameraModel::CameraModel;
		using Ptr = std::shared_ptr<DeviceComponent>;

	protected:
		int m_type = DeviceComponentType::NONE;

	public:
		int get_type() const { return m_type; }
		void set_component_type(int t) { m_type = t; }
		virtual void copy_to(CameraModel* output) const override;
	};

	inline void DeviceComponent::copy_to(CameraModel* output) const
	{
		if (!output) return;
		CameraModel::copy_to(output);

		auto _output = dynamic_cast<DeviceComponent*>(output);
		if (_output)
		{
			_output->m_type = m_type;
		}
	}

	/** \brief convert the uint16 depth map to uint8 for visualization, using range-reflection */
	inline void convert_depth_uint16_to_uint8(cv::Mat* output, cv::Mat& depth_image)
	{
		if (!output) return;
		depth_image.copyTo(*output);
		output->forEach<uint16_t>([](uint16_t& p, const int*) {p %= 256; });
		output->convertTo(*output, CV_8UC1);
	}
}