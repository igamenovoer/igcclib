#include "GyroAccelSensor.h"
#include "igcpp/math/geometry.h"

namespace _NS_UTILITY
{
	GyroAccelSensor::GyroAccelSensor(int buffer_size /*= 1000*/)
	{
		set_buffer_size(buffer_size);
	}

	void GyroAccelSensor::reset_tracking_state()
	{
		m_buf_index = -1;
		for (auto& x : m_samples)
			x->time_us = -1;
	}

	int GyroAccelSensor::get_buffer_size() const
	{
		return m_samples.size();
	}

	void GyroAccelSensor::set_buffer_size(int n)
	{
		m_samples.resize(n);
		for (int i = 0; i < n; i++)
		{
			m_samples[i].reset(new Sample);
		}
	}


	void GyroAccelSensor::set_idle_accel_by_samples(const std::vector<fVECTOR_3>& samples)
	{
		//compute sample average
		m_idle_accel.setZero();
		auto n = samples.size();
		for (const auto& x : samples)
			m_idle_accel += x / n;
	}

	void GyroAccelSensor::set_idle_gyro_by_samples(const std::vector<fVECTOR_3>& samples)
	{
		m_idle_gyro.setZero();
		auto n = samples.size();
		for (const auto& x : samples)
			m_idle_gyro += x / n;
	}

	int GyroAccelSensor::push_sample(const fVECTOR_3& gyro, const fVECTOR_3& accel, int64_t time_us)
	{
		assert_throw(time_us >= 0, "time_us must be non-negative");
		int idx_write = (m_buf_index + 1) % m_samples.size();
		auto& sample = m_samples[idx_write];

		//write gyro and accel readings
		sample->accel = accel - m_idle_accel;
		sample->gyro = gyro - m_idle_gyro;
		sample->time_us = time_us;
		m_buf_index = idx_write;

		_compute_transform(idx_write);
		return idx_write;
	}

	const fMATRIX_4& GyroAccelSensor::get_transform(int index /*= -1*/) const
	{
		assert_throw(m_buf_index >= 0, "no computed transformation");
		if (index < 0)
			return m_samples[m_buf_index]->transmat;
		else
			return m_samples[index]->transmat;
	}

	void GyroAccelSensor::fix_latest_transform(const fMATRIX_4& tmat)
	{
		auto idx = m_buf_index;
		assert_throw(idx >= 0, "no transform to fix");
		Sample* cur_sample = m_samples[idx].get();

		//find the previous fix transform backwards
		auto buf_size = get_buffer_size();
		Sample* prev_fixed_sample = nullptr;
		for (int i = 1; i < buf_size; i++)
		{
			auto idx_prev = (idx - i) % buf_size;
			auto& sample = m_samples[idx_prev];
			if (sample->is_transmat_manually_set)
			{
				prev_fixed_sample = sample.get();
				break;
			}
		}

		//if previously fixed transformation is found, compute velocit based on that
		if (prev_fixed_sample)
		{
			fVECTOR_3 prev_pos = prev_fixed_sample->get_position();
			fVECTOR_3 cur_pos = tmat.leftCols(3).row(3);
			auto dt = cur_sample->time_us - prev_fixed_sample->time_us;
			assert_throw(dt > 0, "time from previous sample to current sample is negative");
			fVECTOR_3 v = (cur_pos - prev_pos) / dt * 1e6;
			cur_sample->velocity_world = v;
		}
		cur_sample->transmat = tmat;
	}

	void GyroAccelSensor::_compute_transform(int index) const
	{
		auto buf_size = get_buffer_size();
		auto idx_prev = (index - 1) % buf_size;
		auto prev_sample = m_samples[idx_prev];
		auto cur_sample = m_samples[index];

		if (!prev_sample->is_valid())
		{
			//this is the first sample
			cur_sample->transmat.setIdentity();
			cur_sample->velocity_world.setZero();
			cur_sample->is_transmat_manually_set = true;
			return;
		}

		//integrate imu readings to track position
		fVECTOR_3 accel_world = prev_sample->transmat.block(0, 0, 3, 3).transpose() * prev_sample->accel;
		fVECTOR_3 velocity_world = prev_sample->velocity_world;
		fVECTOR_3 prev_pos_world = prev_sample->get_position();
		double dt = (cur_sample->time_us - prev_sample->time_us) / 1e6;
		fVECTOR_3 cur_pos_world = prev_pos_world + 0.5 * accel_world * dt * dt + dt * velocity_world;

		//compute rotation
		auto angular_velocity = cur_sample->gyro.norm();
		auto rot_angle = angular_velocity * dt;
		auto rot_axis = cur_sample->gyro.normalized().eval();
		fMATRIX_3 rotmat = rotation_matrix(rot_angle, rot_axis).block(0,0,3,3);		//right-mul rotation matrix in local frame
		fMATRIX_3 R = rotmat * prev_sample->transmat.block(0, 0, 3, 3);	// R_prev * inv(R_prev) * R * R_prev

		cur_sample->transmat.setIdentity();
		cur_sample->transmat.block(0, 0, 3, 3) = R;
		cur_sample->transmat.leftCols(3).row(3) = cur_pos_world;
		cur_sample->velocity_world = velocity_world + accel_world * dt;
	}

}
