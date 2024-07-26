#pragma once
#include "igcclib_device_def.hpp"

namespace _NS_UTILITY
{
	/** \brief frame data id. We use namespace so that you can extend it for different devices.  */
	namespace FrameSegmentID
	{
		enum {
			COLOR_IMAGE = 10000, //the color image captured by color sensor
			COLOR_TO_DEPTH_VALUE, //the depth value for each color pixel
			COLOR_TO_XYZ,	//3d point for each color pixel, in color camera space
			COLOR_CAMERA_INTRINSIC,	//right-mul intrinsic matrix of the color camera
			COLOR_CAMERA_EXTRINSIC,	//4x3 right-mul rotation matrix of the color camera
			COLOR_CAMERA_DISTORTION,		//opencv-compatible distortion factors
			COLOR_RAW_IMAGE,		//device-dependent raw image

			DEPTH_IMAGE,  //the depth image captured by depth sensor
			DEPTH_TO_COLOR_XY, //mapping each depth pixel to a coordinate in color image
			DEPTH_TO_COLOR_VALUE, //color for each depth pixel
			DEPTH_TO_XYZ, //the 3d point in depth camera space for each depth pixel
			DEPTH_CAMERA_INTRINSIC,	//right-mul intrinsic matrix
			DEPTH_CAMERA_EXTRINSIC,	//4x3 right-mul rotation matrix of the depth camera

			IMU_ACCELEROMETER_SEQ,	//CV_32FC3, imu data sequence since last frame with N samples

			//1xN unsigned integer vector, each is the time stamp for a sample
			//IMPORTANT - as opencv does not support 32 or 64 bit unsigned integer,
			//we use CV_32SC1 and CV_32SC2 for that purpose, but the content MUST be
			//interpreted as unsigned integer when reading
			IMU_ACCELEROMETER_TIMESTAMP,	

			IMU_ACCELEROMETER_EXTRINSIC,		//the extrinsic matrix of the imu sensor
			IMU_GYROSCOPE_SEQ,	//CV_32FC3, imu data sequence since last frame with N samples
			IMU_GYROSCOPE_TIMESTAMP,		//like the accel timestamp, these are all unsigned integers
			IMU_GYROSCOPE_EXTRINSIC,
		};
	};

	inline const std::map<int, std::string>& get_frame_segment_id2name()
	{
		static std::map<int, std::string> id2name = {
			{(int)FrameSegmentID::COLOR_IMAGE, "fdk.color_image"},
			{(int)FrameSegmentID::COLOR_TO_DEPTH_VALUE, "fdk.color2depth_value"},
			{(int)FrameSegmentID::COLOR_TO_XYZ, "fdk.color2xyz"},
			{(int)FrameSegmentID::COLOR_CAMERA_INTRINSIC, "fdk.color_intrinsic"},
			{(int)FrameSegmentID::COLOR_CAMERA_EXTRINSIC, "fdk.color_extrinsic"},

			{(int)FrameSegmentID::DEPTH_IMAGE, "fdk.depth_image"},
			{(int)FrameSegmentID::DEPTH_TO_COLOR_XY, "fdk.depth2color_xy"},
			{(int)FrameSegmentID::DEPTH_TO_COLOR_VALUE, "fdk.depth2color_value"},
			{(int)FrameSegmentID::DEPTH_TO_XYZ, "fdk.depth2xyz"},
			{(int)FrameSegmentID::DEPTH_CAMERA_INTRINSIC, "fdk.depth_intrinsic"},
			{(int)FrameSegmentID::DEPTH_CAMERA_EXTRINSIC, "fdk.depth_extrinsic"},

			{(int)FrameSegmentID::IMU_ACCELEROMETER_SEQ, "fdk.accel_xyz"},
			{(int)FrameSegmentID::IMU_ACCELEROMETER_EXTRINSIC, "fdk.accel_extrinsic"},
			{(int)FrameSegmentID::IMU_GYROSCOPE_SEQ, "fdk.gyro_xyz"},
			{(int)FrameSegmentID::IMU_GYROSCOPE_EXTRINSIC, "fdk.gyro_extrinsic"},
		};
		return id2name;
	}

	/** \brief 0-depth means the depth value is not available */
	const int INVALID_DEPTH_VALUE = 0;
	const float INVALID_FLOAT32_VALUE = std::numeric_limits<float>::infinity();
	const double INVALID_DOUBLE_VALUE = std::numeric_limits<double>::infinity();

	/** \brief raw data got from the device. You can read the data directly if you know the device-specific details.
	Otherwise, use a device-specific FrameReader to read. */
	class FrameData {
	protected:
		std::map<int, std::shared_ptr<cv::Mat>> id2segment;	//raw data segment, keyed by an artificial id

	public:
		int id = -1;
		std::map<int, std::string> id2name;	//optional, you can assign a name to each id used in id2data, just for clarity

	public:
		template<typename Arch_t>
		void serialize(Arch_t& ar) {
			ar(id);
			ar(id2segment);
			ar(id2name);
		}

	public:
		using Ptr = std::shared_ptr<FrameData>;

		/** \brief get a frame segment by its id, return nullptr if the segment is not found */
		const cv::Mat* get_segment(int segid) const
		{
			auto it = id2segment.find(segid);
			if (it != id2segment.end())
				return it->second.get();
			else
				return nullptr;
		}

		/** \brief get a frame segment by its id, return nullptr if the segment is not found */
		cv::Mat* get_segment(int segid) {
			auto it = id2segment.find(segid);
			if (it != id2segment.end())
				return it->second.get();
			else
				return nullptr;
		}

		/** \brief get a segment, if not exist, create it */
		cv::Mat* get_or_create_segment(int segid) {
			auto it = id2segment.find(segid);
			if (it != id2segment.end())
				return it->second.get();
			else
			{
				id2segment[segid] = std::make_shared<cv::Mat>();
				return id2segment[segid].get();
			}
		}

		/** \brief put a frame segment into this frame, the content in the data will NOT be copied */
		void set_segment(int segid, const cv::Mat& data) {
			auto _data = get_or_create_segment(segid);
			*_data = data;
		}

		/** \brief put a frame segment into this frame, the input data will be kept as a reference */
		void set_segment(int segid, const std::shared_ptr<cv::Mat>& data)
		{
			id2segment[segid] = data;
		}

		std::map<int, const cv::Mat*> get_all_segments() const {
			std::map<int, const cv::Mat*> output;
			for (const auto& it : id2segment)
				output[it.first] = it.second.get();
			return output;
		}

		std::map<int, cv::Mat*> get_all_segments() {
			std::map<int, cv::Mat*> output;
			for (auto& it : id2segment)
				output[it.first] = it.second.get();
			return output;
		}

		/** \brief remove a segment, return whether it actually exists in the frame and got removed. Removing non-existing
		segment will return false. */
		bool remove_segment(int segid) {
			if (id2segment.find(segid) != id2segment.end())
			{
				id2segment.erase(segid);
				return true;
			}
			else
				return false;
		}

		void remove_all_segments() {
			id2segment.clear();
			id2name.clear();
		}

		void copy_to(FrameData* dst) const
		{
			if (!dst) return;

			dst->id = id;
			dst->id2name = id2name;
			for (auto it : id2segment)
			{
				auto x = dst->get_or_create_segment(it.first);
				it.second->copyTo(*x);
			}
		}

		virtual ~FrameData() {}

	private:
		template<typename T>
		static void _write_timestamp(cv::Mat* output, const T* data, int n)
		{
			if (!output) return;

			int cvtype = 0;
			int nbyte_each_element = sizeof(T);
			if (nbyte_each_element == 4)
				cvtype = CV_32SC1;
			else if (nbyte_each_element == 8)
				cvtype = CV_32SC2;

			output->create(1, n, cvtype);
			memcpy(output->data, data, n * nbyte_each_element);
		}

		//the length of the output buffer must be data.total()*sizeof(T)
		template<typename T>
		static void _read_timestamp(T* output, const cv::Mat& data)
		{
			int nbyte_per_element = sizeof(T);

			//need conversion?
			if (nbyte_per_element == 4 && data.elemSize() == 8)
			{
				//down cast from 64bit to 32bit
				for(int i=0; i<data.total(); i++)
					output[i] = (T)data.at<uint64_t>(i);
			}
			else if (nbyte_per_element == 8 && data.elemSize() == 4)
			{
				//up cast from 32bit to 64bit
				for (int i = 0; i < data.total(); i++)
					output[i] = (T)data.at<uint32_t>(i);
			}
			else if (nbyte_per_element == data.elemSize())
			{
				memcpy(output, data.data, data.elemSize() * data.total());
			}
			else
			{
				assert_throw(false, "unsupported opencv type being used as timestamp");
			}
		}

	public:
		/** \brief write timestamp array into a cvmat */
		static void write_timestamp(cv::Mat* output, const uint32_t* data, int n) {
			_write_timestamp<uint32_t>(output, data, n);
		}

		/** \brief write timestamp array into a cvmat */
		static void write_timestamp(cv::Mat* output, const uint64_t* data, int n) {
			_write_timestamp<uint64_t>(output, data, n);
		}

		/** \brief read timestamp array from a cvmat, the output buffer must be no smaller than data.total().
		If data is 32 bit, conversion will be applied. */
		static void read_timestamp(uint64_t* output, const cv::Mat& data) {
			_read_timestamp<uint64_t>(output, data);
		}

		/** \brief read timestamp array from a cvmat, the output buffer must be no smaller than data.total().
		If data is 64 bit, conversion will be applied. */
		static void read_timestamp(uint32_t* output, const cv::Mat& data) {
			_read_timestamp<uint32_t>(output, data);
		}

		/** \brief read timestamp array from a cvmat into 64 bit ints. If data is 32 bit, conversion will be applied. */
		static std::vector<uint64_t> read_timestamp_u64(const cv::Mat& data) {
			int n = data.total();
			std::vector<uint64_t> output(n);
			read_timestamp(output.data(), data);
			return output;
		}

		/** \brief read timestamp array from a cvmat into 32 bit ints. If data is 64 bit, conversion will be applied. */
		static std::vector<uint32_t> read_timestamp_u32(const cv::Mat& data) {
			int n = data.total();
			std::vector<uint32_t> output(n);
			read_timestamp(output.data(), data);
			return output;
		}
	};
}

