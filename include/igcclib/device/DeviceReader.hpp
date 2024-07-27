#pragma once

#include <set>
#include <igcclib/device/igcclib_device_def.hpp>
#include <igcclib/device/FrameData.hpp>


namespace _NS_UTILITY
{
	class DeviceReader
	{
	public:
		/**
		* \brief initilaize the device
		*
		* \param opt options for initilaization
		* \return bool whether the device is initialized successfully
		*/
		virtual bool init(void* opt) = 0;

		/** \brief open the device for reading */
		virtual void open() = 0;

		virtual bool is_opened() const { return m_is_opened; }

		/** \brief close the device, after that, you can use open() to read data again. */
		virtual void close() = 0;

		/** \brief release all resources, after calling this method,
		this object should not be used again */
		virtual void destroy() = 0;

		/**
		* \brief get a frame from the device, return whether a frame is retrieved
		*
		* \param output the output frame
		* \param components	the components you want to retrieve from the device. Setting it to nullptr
		means to get the default components, depending on the device.
		* \param timeout_ms	ms to wait until time out for retrieving frame, -1 means infinite.
		* \return bool whether a frame is retrieved successfully
		*/
		virtual bool get_frame(FrameData* output, const std::set<int>* components, int timeout_ms = -1) = 0;

		/** \brief report what frame components can be read from the device */
		virtual bool is_frame_component_available(int frame_component) const = 0;

		/**
		* \brief get a device component
		*
		* \param source_component_type	the device component type of the retrieved component
		* \param relative_to		the source component's extrinsic matrix converts point from source to this component
		* \return const DeviceComponent* the device component whose type is source_component_type, whose extrinsic is
		relative to "relative_to" device. Return nullptr if the component type is not available.
		*/
		virtual std::shared_ptr<DeviceComponent> get_device_component(int source_component_type, int relative_to) const = 0;

		virtual ~DeviceReader() {}

		//IDENTITY management
		virtual void set_name(const std::string& name) { m_name = name; }
		virtual const std::string& get_name() const { return m_name; }
		virtual int get_id() const { return m_id; }
		virtual void set_id(int id) { m_id = id; }

	public:
		class ErrFailedToOpenDevice : public std::logic_error
		{
		public:
			using std::logic_error::logic_error;
		};

	protected:
		std::string m_name;
		int m_id = 0;
		bool m_is_opened = false;
	};
}