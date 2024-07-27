#pragma once

#include <vector>
#include <k4a/k4a.h>
#include <igcclib/device/igcclib_device_def.hpp>

namespace _NS_UTILITY
{
	template<typename HandleType>
	using AzureSharedPtr = std::shared_ptr<typename std::remove_pointer<HandleType>::type>;

	class AzureDeviceComponent : public DeviceComponent
	{
	public:
		using DeviceComponent::DeviceComponent;

	protected:
		k4a_calibration_type_t m_k4a_calibration_type;

	public:
		k4a_calibration_type_t get_calibration_type() const { return m_k4a_calibration_type; }
		void set_calibration_type(k4a_calibration_type_t x) { m_k4a_calibration_type = x; }
	};

	/** \brief mapping from component type to k4a calibration type */
	const std::map<int, k4a_calibration_type_t>& get_component_type_to_k4a_type();

	/** \brief mapping from k4a calibration type to component type. This function is NOT thread-safe */
	const std::map<k4a_calibration_type_t, int>& get_k4a_type_to_component_type();

	void convert_k4a_image_to_cvmat(cv::Mat* output, k4a_image_t img);
	AzureSharedPtr<k4a_image_t> wrap_cvmat_to_k4a_image(const cv::Mat& mat);

	/**
	* \brief given a depth map, convert it to point cloud
	*
	* \param out_xyz	 CV_32FC3, each pixel is (x,y,z) for the corresponding pixel in depth
	* \param out_validmask	which point in out_xyz is valid. This is flagged by the API.
	* \param calib	the k4a calibration obejct
	* \param depth	the depth map in CV_16UC1, unit is mm, as returned by the API
	* \param source_camera	the depth map is captured in which camera
	* \param target_camera	the output xyz points are in which camera's coordinate
	*/
	void convert_k4a_depth_to_point_cloud(
		cv::Mat* out_xyz, cv::Mat* out_validmask,
		const k4a_calibration_t& calib,
		const cv::Mat& depth,
		k4a_calibration_type_t source_camera,
		k4a_calibration_type_t target_camera);

	/**
	* \brief convert depth in source camera to 2d xy coordinate
	in the image of the target camera
	*
	* \param out_xy	CV_32FC2, out_xy(i,j)=(x,y) means the pixel (i,j) in the depth maps to pixel (x,y) in the target image.
	* \param out_validmask	which point in out_xy is valid. This is flagged by the API.
	* \param calib	the k4a calibration obejct
	* \param depth	the depth map in CV_16UC1, unit is mm, as returned by the API
	* \param source_camera	the depth map is captured in which camera
	* \param target_camera	the output xyz points are in which camera's coordinate
	*/
	void convert_k4a_depth_to_image_coordinate(
		cv::Mat* out_xy, cv::Mat* out_validmask,
		const k4a_calibration_t& calib,
		const cv::Mat& depth,
		k4a_calibration_type_t source_camera,
		k4a_calibration_type_t target_camera
	);

	/**
	* \brief	 get intrinsic and extrinsic camera parameter from azure kinect.
	* Note that the output camera matrices are in right-mul format, that is,
	* Q=point.dot(camera_matrix), which is different from the opencv convention.
	*
	* \param out_intrinsic	3x3 CV_32FC1, right-mul intrinsic camera parameters
	* \param out_extrinsic	4x3 CV_32FC1, right-mul extrinsic camera parameters
	* \param out_distortion 1xn CV_32FC1, the distortion coefficients in opencv convention
	* \param calib	the k4a camera calibration info
	* \param source_camera	the camera whose intrinsic and extrinsic are being retrieved
	* \param target_camera	the extrinsic converts coordinate of source camera to target camera
	*/
	void get_k4a_camera_parameter(
		cv::Mat* out_intrinsic, cv::Mat* out_extrinsic, cv::Mat* out_distortion,
		const k4a_calibration_t& calib,
		k4a_calibration_type_t source_camera,
		k4a_calibration_type_t target_camera
	);

	/**
	* \brief	 get a device component from the calibration data
	*
	* \param output	the output device component
	* \param calib	the calibration info
	* \param source_device	the device to get
	* \param target_device	the extrinsic converts coordinates of source device to target device
	*/
	void get_k4a_device_component(AzureDeviceComponent* output,
		const k4a_calibration_t& calib,
		k4a_calibration_type_t source_device,
		k4a_calibration_type_t target_device);

	/** \brief convert color resolution flag into color image size */
	cv::Size get_k4a_color_camera_image_size(k4a_color_resolution_t res);
}

namespace cereal {
	//k4a_calibration_t
	template<typename Arch_t>
	void save(Arch_t& ar, const k4a_calibration_t& data) {
		ar(data.depth_camera_calibration, data.color_camera_calibration);
		
		//save extrinsic data
		for (int i = 0; i < K4A_CALIBRATION_TYPE_NUM; i++)
			for (int j = 0; j < K4A_CALIBRATION_TYPE_NUM; j++)
				ar(data.extrinsics[i][j]);

		//save others
		ar((int)data.depth_mode);
		ar((int)data.color_resolution);
	}

	template<typename Arch_t>
	void load(Arch_t& ar, k4a_calibration_t& data) {
		ar(data.depth_camera_calibration, data.color_camera_calibration);

		for(int i=0; i<K4A_CALIBRATION_TYPE_NUM; i++)
			for (int j = 0; j < K4A_CALIBRATION_TYPE_NUM; j++)
				ar(data.extrinsics[i][j]);

		int depth_mode, color_resolution;
		ar(depth_mode);
		ar(color_resolution);

		data.depth_mode = (k4a_depth_mode_t)depth_mode;
		data.color_resolution = (k4a_color_resolution_t)color_resolution;
	}

	//k4a_calibration_camera_t
	template<typename Arch_t>
	void save(Arch_t& ar, const k4a_calibration_camera_t& data) {
		ar(data.extrinsics);
		ar(data.intrinsics);
		ar(data.resolution_width, data.resolution_height);
		ar(data.metric_radius);
	}

	template<typename Arch_t>
	void load(Arch_t& ar, k4a_calibration_camera_t& output) {
		ar(output.extrinsics);
		ar(output.intrinsics);
		ar(output.resolution_width, output.resolution_height);
		ar(output.metric_radius);
	}

	//k4a_calibration_intrinsics_t
	template<typename Arch_t>
	void save(Arch_t& ar, const k4a_calibration_intrinsics_t& data) {
		std::vector<float> params(data.parameters.v, data.parameters.v + sizeof(data.parameters.v) / sizeof(float));

		ar((int)data.type);
		ar(data.parameter_count);
		ar(params);
	}

	template<typename Arch_t>
	void load(Arch_t& ar, k4a_calibration_intrinsics_t& data) {
		int dtype;
		unsigned int count;
		std::vector<float> params;

		ar(dtype);
		ar(count);
		ar(params);

		data.type = (k4a_calibration_model_type_t)dtype;
		data.parameter_count = count;
		std::copy(params.begin(), params.end(), data.parameters.v);
	}

	// k4a_calibration_extrinsics_t
	template<typename Arch_t>
	void save(Arch_t& ar, const k4a_calibration_extrinsics_t& data)
	{
		std::vector<float> rot(data.rotation, data.rotation + 9);
		std::vector<float> t(data.translation, data.translation + 3);
		ar(rot, t);
	}

	template<typename Arch_t>
	void load(Arch_t& ar, k4a_calibration_extrinsics_t& data) {
		std::vector<float> rot, t;
		ar(rot, t);

		std::copy(rot.begin(), rot.end(), data.rotation);
		std::copy(t.begin(), t.end(), data.translation);
	}
}