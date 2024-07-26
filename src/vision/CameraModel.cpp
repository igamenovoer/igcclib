#include <igcclib/vision/CameraModel.hpp>
#include <igcclib/vision/igcclib_opencv_def.hpp>
#include <igcclib/core/igcclib_logging.hpp>


namespace _NS_UTILITY {
	void CameraModel::ray_from_projected_points(const fVECTOR_2& p, fVECTOR_3* out_ray_p0, fVECTOR_3* out_ray_dir) const
	{
		fMATRIX_4 transmat = m_extrinsic_matrix.inverse();

		fVECTOR_3 p3(p[0], p[1], 1);
		fVECTOR_3 ray_dir = (p3.transpose() * m_projection_matrix.inverse() * transmat.block(0, 0, 3, 3)).transpose();
		ray_dir.normalize();

		fVECTOR_3 ray_p0 = transmat.leftCols(3).bottomRows(1).transpose();

		if (out_ray_p0)
			*out_ray_p0 = ray_p0;
		if (out_ray_dir)
			*out_ray_dir = ray_dir;
	}

	double CameraModel::get_fov_width_degree() const
	{
		if (is_orthographic())
			return 0;

		auto width = std::abs(m_projection_matrix(2, 0) * 2);
		auto focal = std::abs(m_projection_matrix(0, 0));
		auto fov = 2 * std::atan2(width, 2 * focal) * MathConstant::Rad2Deg;
		return fov;
	}

	double CameraModel::get_fov_height_degree() const
	{
		if (is_orthographic())
			return 0;

		auto height = std::abs(m_projection_matrix(2, 1) * 2);
		auto focal = std::abs(m_projection_matrix(0, 0));
		auto fov = 2 * std::atan2(height, 2 * focal) * MathConstant::Rad2Deg;
		return fov;
	}

	void CameraModel::set_distortion_coefficient(const fVECTOR& distcoef)
	{
		int n_coef = (int)distcoef.size();
		assert_throw(n_coef == 0 || n_coef == 4 || n_coef == 5 || n_coef == 8 || n_coef == 12 || n_coef == 14,
			"wrong number of distortion coefficients, it must contain 0,4,5,8,12 or 14 elements");
		m_distortion = distcoef;
	}

	void CameraModel::copy_to(CameraModel* output) const
	{
		output->m_distortion = m_distortion;
		output->m_extrinsic_matrix = m_extrinsic_matrix;
		output->m_height = m_height;
		output->m_width = m_width;
		output->m_projection_matrix = m_projection_matrix;
	}

	fMATRIX CameraModel::project_points(const fMATRIX& pts, bool with_distortion, bool preserve_z) const
	{
		auto n_points = pts.rows();

		fMATRIX _pts;
		transform_points(pts, m_extrinsic_matrix, _pts);
		fVECTOR zs = _pts.col(2);

		fMATRIX output;
		if (is_orthographic())
		{
			_pts *= m_projection_matrix;
			output = _pts.leftCols(2);
		}
		else
		{
			auto cv_float_type = cv::DataType<float_type>::type;
			cv::Mat zero_1x3(1, 3, cv_float_type, cv::Scalar::all(0));
			cv::Mat distcoef(1, m_distortion.size(), cv_float_type, (void*)m_distortion.data());
			cv::Mat camera_matrix(3, 3, cv_float_type);
			Eigen::Map<fMATRIX>((float_type*)camera_matrix.data, 3, 3) = m_projection_matrix.transpose();

			std::vector<cv::Point3d> points(n_points);
			for (size_t i = 0; i < points.size(); i++)
				points[i] = cv::Point3d(_pts(i, 0), _pts(i, 1), _pts(i, 2));

			cv::Mat distort_points;
			cv::projectPoints(points, zero_1x3, zero_1x3, camera_matrix, distcoef, distort_points);
			distort_points.convertTo(distort_points, cv_float_type);

			output = Eigen::Map<fMATRIX>((float_type*)distort_points.data, points.size(), 2);
		}

		if (preserve_z)
		{
			fMATRIX _output(output.rows(), 3);
			_output.leftCols(2) = output;
			_output.col(2) = zs;
			output = _output;
		}
			
		return output;
	}

	fVECTOR_2 CameraModel::project_points(const fVECTOR_3& p, bool with_distortion) const
	{
		fMATRIX q = p.transpose();
		fVECTOR_2 output = project_points(q, with_distortion).row(0);
		return output;
	}

	fMATRIX_3 CameraModel::projection_matrix_by_fov_width(double fov_deg, double width, double height)
	{
		double fov_rad = fov_deg * MathConstant::Deg2Rad;
		double f = width / (2 * std::tan(fov_rad / 2));
		fMATRIX_3 mat;
		mat << f, 0, 0,
			0, f, 0,
			width / 2, height / 2, 1;
		return mat;
	}

	fMATRIX_3 CameraModel::projection_matrix_by_fov_height(double fov_deg, double width, double height)
	{
		double fov_rad = fov_deg * MathConstant::Deg2Rad;
		double f = height / (2 * std::tan(fov_rad / 2));
		fMATRIX_3 mat;
		mat << f, 0, 0,
			0, f, 0,
			width / 2, height / 2, 1;
		return mat;
	}

	fMATRIX_4 CameraModel::extrinsic_matrix_by_look_at(const fVECTOR_3& camera_position, const fVECTOR_3& target, const fVECTOR_3& up_vector)
	{
		fMATRIX_4 camera_frame = fMATRIX_4::Identity();

		//the camera looks at z-, x+ right, y+ up
		fVECTOR_3 z = (camera_position - target).stableNormalized();
		fVECTOR_3 x = up_vector.cross(z).stableNormalized();
		fVECTOR_3 y = z.cross(x).stableNormalized();

		//now, invert the axis to be compatible with opencv, looks at z+, x+ right, y+ down
		camera_frame.block(0, 0, 3, 3).row(0) = x.transpose();
		camera_frame.block(0, 0, 3, 3).row(1) = -y.transpose();
		camera_frame.block(0, 0, 3, 3).row(2) = -z.transpose();
		camera_frame.bottomRows(1).leftCols(3) = camera_position.transpose();
		fMATRIX_4 extmat = camera_frame.inverse();
		return extmat;
	}

	fVECTOR_3 CameraModel::convert_world_to_camera(const fVECTOR_3& p) const
	{
		return transform_single_point(p, m_extrinsic_matrix);
	}

	fVECTOR_3 CameraModel::convert_camera_to_world(const fVECTOR_3& p) const
	{
		return transform_single_point(p, m_extrinsic_matrix.inverse().eval());
	}

}


