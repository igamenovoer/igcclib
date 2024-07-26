#pragma once

#include "igcclib_device_def.hpp"

namespace _NS_UTILITY
{
	/*!
	 * \class GyroAccelSensor
	 *
	 * \brief IMU composite sensor with gyroscope and accelerometer, designed using azure kinect IMU sensor as a reference.
	 This class tracks the position based on gyro and accelerometer, and the drift should be corrected externally.
	 */
	class GyroAccelSensor
	{
	public:
		GyroAccelSensor(int buffer_size = 1000);
		virtual ~GyroAccelSensor() {}

		/** \brief reset the position tracking state */
		virtual void reset_tracking_state();

		/** \brief the number of IMU samples to keep */
		int get_buffer_size() const;

		/** \brief set the number of IMU samples to keep */
		void set_buffer_size(int n);

		/** \brief input some accelerometer samples collected when the sensor is idle, 
		which helps to build a noise model for the accel readings */
		virtual void set_idle_accel_by_samples(const std::vector<fVECTOR_3>& samples);

		/** \brief input some gyroscope samples collected when the sensor is idle,
		which are used to build a noise model for gyro readings*/
		virtual void set_idle_gyro_by_samples(const std::vector<fVECTOR_3>& samples);

		/**
		* \brief input a new IMU sample
		*
		* \param gyro	the gyroscope reading
		* \param accel	the accelerometer reading
		* \param time_sec	the timestamp of this sample, in second
		* \return the index of this sample, note that the indices will be reused when the buffer is full, that is, it is not guaranteed to be increasing.
		*/
		virtual int push_sample(const fVECTOR_3& gyro, const fVECTOR_3& accel, int64_t time_us);

		/** \brief get the transformation matrix of a particular sample. The transformation matrix is in right-mul format, that is, the last row is position. */
		virtual const fMATRIX_4& get_transform(int index = -1) const;

		/** \brief manually set the transformation matrix of the latest sample */
		virtual void fix_latest_transform(const fMATRIX_4& tmat);

	protected:
		/** \brief compute the transformation for a particular sample */
		virtual void _compute_transform(int index) const;

	protected:
		/** \brief index of the latest sample in the buffer */
		int m_buf_index = -1;

		class Sample {
		public:
			virtual ~Sample() {}

			//sample data
			fVECTOR_3 gyro = fVECTOR_3::Zero();		//gyroscope sample
			fVECTOR_3 accel = fVECTOR_3::Zero();		//accelerometer sample
			int64_t time_us = -1;			//timestamp in us (1e-6 second)

			//derived data
			fVECTOR_3 velocity_world = fVECTOR_3::Zero();	//velocity in world space
			fMATRIX_4 transmat = fMATRIX_4::Identity();	//the transformation in world space
			bool is_transmat_manually_set = false;	//is this transformation manually set?

			bool is_valid() const { return time >= 0; }

			/** \brief get world position */
			fVECTOR_3 get_position() const {
				return { transmat(3,0), transmat(3,1), transmat(3,2) };
			}
		};

		/** \brief all samples */
		std::vector<std::shared_ptr<Sample>> m_samples;

		/** \brief idle acceleration */
		fVECTOR_3 m_idle_accel = fVECTOR_3::Zero();

		/** \brief idle gyroscope data */
		fVECTOR_3 m_idle_gyro = fVECTOR_3::Zero();
	};
}