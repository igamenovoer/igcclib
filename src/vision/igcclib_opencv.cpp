#include <map>
#include <igcclib/vision/igcclib_opencv.hpp>
#include <igcclib/geometry/igcclib_geometry.hpp>

namespace _NS_UTILITY {
	void draw_markers(
		cv::Mat& img, const fMATRIX& pts_xy,
		const fVECTOR_3& color3f, int marker_type, int marker_size, int line_width) {

		cv::Mat& output = img;

		for (fMATRIX::Index i = 0; i < pts_xy.rows(); i++) {
			cv::Point p((int)pts_xy(i, 0), (int)pts_xy(i, 1));
			cv::Scalar color(color3f[0] * 255, color3f[1] * 255, color3f[2] * 255);
			cv::drawMarker(output, p, color, marker_type, marker_size, line_width);
		}
	}

	fMATRIX_4 find_camera_extrinsic(
		const fMATRIX& pts_3d, const fMATRIX& pts_2d, 
		const fMATRIX_3& projection_matrix)
	{
		bool is_orthographic = projection_matrix.col(2).squaredNorm() < 1e-8;

		fMATRIX_4 output;
		if (is_orthographic) {
			//orthographic calibration
			fMATRIX_3 pmat = projection_matrix;
			pmat.col(2) = fVECTOR_3(0, 0, 1);

			//transform the 2d points
			fMATRIX pts_target(pts_2d.rows(),3);
			pts_target.fill(0);
			{
				fMATRIX _pts;
				transform_points(pts_2d, pmat.inverse().eval(), _pts);
				pts_target.leftCols(2) = _pts;
			}

			//find similarity transform only considering the xy coordinate
			output = find_transform_similarity(pts_3d, pts_target);
			//output = find_transform_projective(pts_3d, pts_target);
		}
		else {
			//projective case
			int cvtype = cv::DataType<float_type>::type;
			fMATRIX_3 pmat = projection_matrix.transpose();
			cv::Mat cameraMatrix(3, 3, cvtype, pmat.data());

			cv::Mat objectPoints((int)pts_3d.rows(), (int)pts_3d.cols(), cvtype, (char*)pts_3d.data());
			cv::Mat imagePoints((int)pts_2d.rows(), (int)pts_2d.cols(), cvtype, (char*)pts_2d.data());

			cv::Mat rvec, tvec;
			cv::solvePnP(objectPoints, imagePoints, cameraMatrix, cv::Mat(), rvec, tvec);

			//convert to transmat
			cv::Mat rotmat;
			cv::Rodrigues(rvec, rotmat);
			rotmat.convertTo(rotmat, cvtype);
			tvec.convertTo(tvec, cvtype);

			output.setIdentity();
			output.block(0, 0, 3, 3) = Eigen::Map<fMATRIX_3>((float_type*)rotmat.data).transpose();
			output.leftCols(3).row(3) = Eigen::Map<fVECTOR_3>((float_type*)tvec.data);
		}
		return output;
	}

	void to_3_channel(const cv::Mat& input, cv::Mat& output)
	{
		if (input.channels() == 1)
			cv::cvtColor(input, output, cv::COLOR_GRAY2RGB);
		else if (input.channels() == 3)
			input.copyTo(output);
		else if (input.channels() == 4)
			cv::cvtColor(input, output, cv::COLOR_BGRA2BGR);
	}

	cv::Scalar make_scalar(double value, int n_channel)
	{
		switch (n_channel) {
		case 1:
			return cv::Scalar(value);
		case 2:
			return cv::Scalar(value, value);
		case 3:
			return cv::Scalar(value, value, value);
		case 4:
			return cv::Scalar(value, value, value, value);
		default: 
			assert_throw(false, "channel can only be 1,2,3,4");
			return cv::Scalar();
		}
	}

	double diagonal_length(const cv::Size sz)
	{
		return std::sqrt((double)sz.width * sz.width + (double)sz.height * sz.height);
	}

	void clamp_values(cv::Mat& inout, double minval, double maxval)
	{
		inout = cv::max(cv::min(inout, maxval), minval);
	}

	void to_uint8(cv::Mat& inout)
	{
		int cvtype = cv::DataType<uint8_t>::type;
		if (is_type_floating_point(inout))
			inout.convertTo(inout, cvtype, 255.0);
		else
			inout.convertTo(inout, cvtype);
	}

	void CvMatHelper::get_as_type(cv::Mat& output, int cv_type, 
		bool allow_change_channel /*= true*/,
		bool allow_scale_value) const
	{
		cv::Mat dummy(0, 0, cv_type);

		//make enough channels
		std::vector<cv::Mat> chlist;
		cv::split(m_data, chlist);

		std::vector<cv::Mat> outchlist;
		if (chlist.size() == 1) {
			//single channel input, multi channel output
			for (int i = 0; i < dummy.channels(); i++)
				outchlist.push_back(chlist[0]);
		}
		else {
			//multi channel input, multi channel output
			for (size_t i = 0; i < chlist.size() && i < dummy.channels(); i++)
				outchlist.push_back(chlist[i]);

			if (outchlist.size() < dummy.channels()) {
				cv::Mat ONE;
				chlist[0].copyTo(ONE);
				if (is_type_integer(ONE))
					ONE.setTo(255);
				else
					ONE.setTo(1.0);

				for (size_t i = outchlist.size(); i < dummy.channels(); i++)
					outchlist.push_back(ONE);
			}
		}
		cv::merge(outchlist, output);

		//matching datatype
		if (allow_scale_value)
		{
			if (is_type_integer(output) && is_type_floating_point(dummy))
			{
				//from integer to floating point
				output.convertTo(output, cv_type, 1.0 / 255);
			}
			else if (is_type_floating_point(output) && is_type_integer(dummy)) {
				//from floating point to integer
				output.convertTo(output, cv_type, 255.0);
			}
			else output.convertTo(output, cv_type);
		}
		else
			output.convertTo(output, cv_type);
	}

	cv::Mat CvMatHelper::get_with_channel_replaced(const cv::Mat& data, int idx_channel)
	{
		std::vector<cv::Mat> chlist;
		cv::split(m_data, chlist);
		chlist[idx_channel] = data;

		cv::Mat output;
		cv::merge(chlist, output);
		return output;
	}


	void CvMatHelper::update_by(const cv::Mat& data, bool allow_scale_value)
	{
		assert_throw(is_same_size(data, m_data), "the new data does not have the same size as internal data");

		std::vector<cv::Mat> chlist_old;
		cv::split(m_data, chlist_old);
		
		cv::Mat newdata;

		//unify data type
		if (allow_scale_value)
		{
			if (is_type_integer(m_data) && is_type_floating_point(data))
				data.convertTo(newdata, m_data.type(), 255.0);
			else if (is_type_floating_point(m_data) && is_type_integer(data))
				data.convertTo(newdata, m_data.type(), 1.0 / 255);
			else
				data.convertTo(newdata, m_data.type());
		}
		else data.convertTo(newdata, m_data.type());

		//copy channels
		std::vector<cv::Mat> chlist_new;
		cv::split(newdata, chlist_new);

		if (chlist_new.size() == 1) //single channel to multi channel
		{
			for (int i = 0; i < chlist_old.size() && i < 4; i++) //do not affect alpha channel
				chlist_old[i] = chlist_new[0];
		}
		else { //multi channel to multi channel
			for (int i = 0; i < chlist_old.size() && i < chlist_new.size(); i++) {
				chlist_old[i] = chlist_new[i];
			}
		}
		cv::merge(chlist_old, m_data);
	}

	cv::Mat CvMatHelper::get_as_type(int cv_type, bool allow_change_channel /*= true*/, bool allow_scale_value /*= true*/) const
	{
		cv::Mat output;
		get_as_type(output, cv_type, allow_change_channel, allow_scale_value);
		return output;
	}

	void imresize_by_maxlen(cv::Mat& output, const cv::Mat& input, int maxlen, int interp_method)
	{
		int height = input.rows;
		int width = input.cols;

		if (height > width)
		{
			double scale = (double)maxlen / height;
			height = maxlen;
			width = (int)(width * scale);
		}
		else
		{
			double scale = (double)maxlen / width;
			width = maxlen;
			height = (int)(height * scale);
		}

		cv::resize(input, output, cv::Size(width, height), 0, 0, interp_method);
	}

}

