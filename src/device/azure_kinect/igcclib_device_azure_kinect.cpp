#include <igcclib/device/azure_kinect/igcclib_device_azure_kinect.hpp>
#include <igcclib/device/FrameData.hpp>
#include <igcclib/vision/igcclib_opencv_eigen.hpp>

namespace _NS_UTILITY
{
	void convert_k4a_image_to_cvmat(cv::Mat* output, k4a_image_t img)
	{
		if (!output)
			return;

		auto h = k4a_image_get_height_pixels(img);
		auto w = k4a_image_get_width_pixels(img);
		auto s = k4a_image_get_stride_bytes(img);
		auto nbyte = k4a_image_get_size(img);
		auto buf = k4a_image_get_buffer(img);
		auto fmt = k4a_image_get_format(img);

		//spdlog::info("image size={0}x{1}, nbyte={2}, stride={3}", w, h, nbyte, s);

		int cvtype = 0;
		switch (fmt)
		{
		case K4A_IMAGE_FORMAT_COLOR_BGRA32:
			cvtype = CV_8UC4;
			break;
		case K4A_IMAGE_FORMAT_DEPTH16:
		case K4A_IMAGE_FORMAT_IR16:
			cvtype = CV_16UC1;
			break;
		}

		cv::Mat temp(h, w, cvtype, buf, s);
		temp.copyTo(*output);
	}

	AzureSharedPtr<k4a_image_t> wrap_cvmat_to_k4a_image(const cv::Mat& mat)
	{
		bool type_ok = mat.type() == CV_8UC4 || mat.type() == CV_16UC1;
		assert_throw(type_ok, "the data type is not acceptable");

		k4a_image_format_t fmt = K4A_IMAGE_FORMAT_CUSTOM;
		switch (mat.type()) {
		case CV_16UC1:
			fmt = K4A_IMAGE_FORMAT_DEPTH16;
			break;
		case CV_8UC4:
			fmt = K4A_IMAGE_FORMAT_COLOR_BGRA32;
			break;
		}

		auto width = mat.cols;
		auto height = mat.rows;
		int stride = mat.step;
		k4a_image_t img;

		auto ok = !K4A_FAILED(k4a_image_create_from_buffer(
			fmt, width, height, stride, mat.data, mat.total() * mat.elemSize(),
			nullptr, nullptr, &img
		));

		assert_throw(ok, "failed to wrap image");

		//wrap the image into shared pointer
		AzureSharedPtr<k4a_image_t> output(img, [](k4a_image_t p) {if (p) k4a_image_release(p); });
		return output;
	}

	void convert_k4a_depth_to_point_cloud(cv::Mat* out_xyz, cv::Mat* out_validmask, const k4a_calibration_t& calib, const cv::Mat& depth, k4a_calibration_type_t source_camera, k4a_calibration_type_t target_camera)
	{
		//get point cloud in color space
		if (out_xyz)
		{
			out_xyz->create(depth.rows, depth.cols, CV_32FC3);
			out_xyz->setTo(cv::Scalar::all(INVALID_FLOAT32_VALUE));
		}

		if (out_validmask)
		{
			out_validmask->create(depth.rows, depth.cols, CV_8UC1);
			out_validmask->setTo(0);
		}

		//transform each pixel with depth to 3d
#pragma omp parallel for
		for (int i = 0; i < depth.rows; i++)
			for (int j = 0; j < depth.cols; j++)
			{
				//check depth
				auto d = depth.at<uint16_t>(i, j);
				if (d == 0) continue;

				k4a_float2_t p;
				p.xy.x = (float)j;
				p.xy.y = (float)i;

				k4a_float3_t q;
				int valid = 0;

				k4a_calibration_2d_to_3d(&calib, &p, d, source_camera, target_camera, &q, &valid);

				if (out_validmask)
					out_validmask->at<uint8_t>(i, j) = valid ? 255 : 0;

				if (valid && out_xyz)
				{
					auto& p = out_xyz->at<cv::Vec3f>(i, j);
					for (int k = 0; k < 3; k++)
						p[k] = q.v[k];
				}
			}
	}

	void convert_k4a_depth_to_image_coordinate(
		cv::Mat* out_xy, cv::Mat* out_validmask,
		const k4a_calibration_t& calib, const cv::Mat& depth,
		k4a_calibration_type_t source_camera, k4a_calibration_type_t target_camera)
	{
		if (out_xy)
		{
			out_xy->create(depth.rows, depth.cols, CV_32FC2);
			out_xy->setTo(cv::Scalar::all(INVALID_FLOAT32_VALUE));
		}

		if (out_validmask)
		{
			out_validmask->create(depth.rows, depth.cols, CV_8UC1);
			out_validmask->setTo(cv::Scalar::all(0));
		}

#pragma omp parallel for
		for (int i = 0; i < depth.rows; i++)
			for (int j = 0; j < depth.cols; j++)
			{
				auto d = depth.at<uint16_t>(i, j);
				if (d == 0) continue;

				k4a_float2_t p;
				p.xy.x = (float)j;
				p.xy.y = (float)i;

				k4a_float2_t q;
				int valid = 0;
				k4a_calibration_2d_to_2d(&calib, &p, d, source_camera, target_camera, &q, &valid);

				if (out_validmask)
					out_validmask->at<uint8_t>(i, j) = (uint8_t)valid;

				if (valid && out_xy)
				{
					auto& x = out_xy->at<cv::Vec2f>(i, j);
					x[0] = q.v[0];
					x[1] = q.v[1];
				}
			}
	}

	const std::map<int, k4a_calibration_type_t>& get_component_type_to_k4a_type()
	{
		static std::map<int, k4a_calibration_type_t> comp2calib = {
			{DeviceComponentType::COLOR_CAMERA, K4A_CALIBRATION_TYPE_COLOR},
			{DeviceComponentType::DEPTH_CAMERA, K4A_CALIBRATION_TYPE_DEPTH},
			{DeviceComponentType::IMU_ACCELEROMETER, K4A_CALIBRATION_TYPE_ACCEL},
			{DeviceComponentType::IMU_GYROSCOPE, K4A_CALIBRATION_TYPE_GYRO}
		};

		return comp2calib;
	}

	const std::map<k4a_calibration_type_t, int>& get_k4a_type_to_component_type()
	{
		static std::map<k4a_calibration_type_t, int> calib2comp;
		if (calib2comp.empty())
		{
			const auto& comp2calib = get_component_type_to_k4a_type();
			for (const auto& it : comp2calib)
				calib2comp[it.second] = it.first;
		}
		return calib2comp;
	}

	void get_k4a_camera_parameter(
		cv::Mat* out_intrinsic, cv::Mat* out_extrinsic, cv::Mat* out_distortion,
		const k4a_calibration_t& calib,
		k4a_calibration_type_t source_camera, k4a_calibration_type_t target_camera)
	{
		const k4a_calibration_camera_t* camera_param = nullptr;
		switch (source_camera)
		{
		case K4A_CALIBRATION_TYPE_COLOR:
			camera_param = &calib.color_camera_calibration;
			break;
		case K4A_CALIBRATION_TYPE_DEPTH:
			camera_param = &calib.depth_camera_calibration;
			break;
		}

		//read intrinsic
		if (out_intrinsic && camera_param)
		{
			const auto& p = camera_param->intrinsics.parameters.param;
			*out_intrinsic = cv::Mat::eye(3, 3, CV_32FC1);
			out_intrinsic->at<float>(0, 0) = p.fx;
			out_intrinsic->at<float>(1, 1) = p.fy;
			out_intrinsic->at<float>(2, 0) = p.cx;
			out_intrinsic->at<float>(2, 1) = p.cy;
		}

		//read extrinsic
		if (out_extrinsic)
		{
			out_extrinsic->create(4, 3, CV_32FC1);
			const auto& p = calib.extrinsics[source_camera][target_camera];
			using FT = std::remove_cv<std::remove_reference<decltype(p.rotation[0])>::type>::type;
			int cvtype = cv::DataType<FT>::type;
			cv::Mat rotmat(3, 3, cvtype, (void*)p.rotation);
			cv::transpose(rotmat, rotmat);	//convert to right-mul format

			cv::Mat pos(1, 3, cvtype, (void*)p.translation);

			rotmat.copyTo((*out_extrinsic)(cv::Rect(0, 0, 3, 3)));
			pos.copyTo((*out_extrinsic)(cv::Rect(0, 3, 3, 1)));
		}

		//read distortion
		if (out_distortion && camera_param)
		{
			const auto& p = camera_param->intrinsics.parameters.param;
			float d[] = { p.k1, p.k2, p.p1, p.p2, p.k3, p.k4, p.k5, p.k6 };

			out_distortion->create(1, sizeof(d) / sizeof(float), CV_32FC1);
			out_distortion->setTo(0);

			float* x = &out_distortion->at<float>(0, 0);
			std::copy(d, d + sizeof(d) / sizeof(float), x);
		}
	}

	void get_k4a_device_component(
		AzureDeviceComponent* output, 
		const k4a_calibration_t& calib, 
		k4a_calibration_type_t source_device, 
		k4a_calibration_type_t target_device)
	{
		if (!output) return;

		cv::Mat intr, extr, dist;
		get_k4a_camera_parameter(&intr, &extr, &dist, calib, source_device, target_device);

		AzureDeviceComponent& comp = *output;
		if (!intr.empty())
		{
			fMATRIX_3 projmat;
			to_matrix(intr, projmat);
			comp.set_projection_matrix(projmat);
		}

		if (!extr.empty())
		{
			fMATRIX_4 extmat = fMATRIX_4::Identity();
			extmat.block(0, 0, 4, 3) = Eigen::Map<MATRIX_f>((float*)extr.data, 4, 3).cast<float_type>();
			comp.set_extrinsic_matrix(extmat);
		}

		if (!dist.empty())
		{
			fVECTOR d;
			to_vector(dist, d);
			comp.set_distortion_coefficient(d);
		}

		if (source_device == K4A_CALIBRATION_TYPE_COLOR)
		{
			comp.set_image_size(calib.color_camera_calibration.resolution_width, calib.color_camera_calibration.resolution_height);
		}
		else if (source_device == K4A_CALIBRATION_TYPE_DEPTH)
		{
			comp.set_image_size(calib.depth_camera_calibration.resolution_width, calib.depth_camera_calibration.resolution_height);
		}

		auto comp_type = get_k4a_type_to_component_type().at(source_device);
		comp.set_component_type(comp_type);
		comp.set_calibration_type(source_device);
	}

	cv::Size get_k4a_color_camera_image_size(k4a_color_resolution_t res)
	{
		cv::Size output;
		switch (res)
		{
		case K4A_COLOR_RESOLUTION_1536P:
			output.width = 2048;
			output.height = 1536;
			break;
		case K4A_COLOR_RESOLUTION_3072P:
			output.width = 4096;
			output.height = 3072;
			break;
		case K4A_COLOR_RESOLUTION_720P:
			output.width = 1280;
			output.height = 720;
			break;
		case K4A_COLOR_RESOLUTION_2160P:
			output.width = 3840;
			output.height = 2160;
			break;
		case K4A_COLOR_RESOLUTION_1440P:
			output.width = 2560;
			output.height = 1440;
			break;
		case K4A_COLOR_RESOLUTION_1080P:
			output.width = 1920;
			output.height = 1080;
			break;
		}
		return output;
	}

}