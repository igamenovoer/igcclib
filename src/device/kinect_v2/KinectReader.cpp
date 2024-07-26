#ifndef NOMINMAX
	#define NOMINMAX
#endif

#include <Windows.h>
#include <Ole2.h>
#include <Kinect.h>

#include "KinectReader.h"
#include "spdlog/spdlog.h"

namespace _NS_UTILITY {

	template<typename T>
	class ResourceHolder {
	private:
		std::unique_ptr<T, void(*)(T*)> m_ptr{ nullptr, nullptr };
	public:
		ResourceHolder(T* x) {			
			m_ptr = std::unique_ptr<T, void(*)(T*)>(x, [](T* obj) {if (obj) obj->Release(); });
		}
	};

	bool KinectReader::init(void* opt)
	{
		//create sensor
		{
			IKinectSensor* sensor = nullptr;
			auto res = GetDefaultKinectSensor(&sensor);
			if (FAILED(res))
				return false;
			m_sensor.reset(sensor, safe_release<IKinectSensor>);
			sensor->Open();
		}

		//get coordinate mapper
		{
			ICoordinateMapper* mapper = nullptr;
			auto res = m_sensor->get_CoordinateMapper(&mapper);
			if (FAILED(res))
				return false;
			m_mapper.reset(mapper, safe_release<ICoordinateMapper>);
		}
		return true;
	}

	void KinectReader::open()
	{
		//get frame reader
		{
			IMultiSourceFrameReader* reader = nullptr;
			auto res = m_sensor->OpenMultiSourceFrameReader(
				FrameSourceTypes_Color | FrameSourceTypes_Depth | FrameSourceTypes_Infrared,
				&reader);

			if (FAILED(res))
				assert_throw(false, "failed to open frame reader");
			m_reader_multi_src.reset(reader, safe_release<IMultiSourceFrameReader>);
		}
	}

	void KinectReader::destroy()
	{
		m_mapper.reset();
		if (m_sensor)
			m_sensor->Close();
		m_sensor.reset();
	}

	fMATRIX_3 KinectReader::get_depth_camera_intrinsic() const
	{
		return get_depth_camera_intrinsic_matrix();
	}

	fMATRIX_3 KinectReader::get_color_camera_intrinsic() const
	{
		return get_color_camera_intrinsic_matrix();
	}

	void KinectReader::close()
	{
		m_reader_multi_src.reset();
	}

	bool KinectReader::get_frame(FrameData* output, int frame_component)
	{
		IMultiSourceFrame* frame = nullptr;
		m_reader_multi_src->AcquireLatestFrame(&frame);
		ResourceHolder<IMultiSourceFrame> _frame(frame);	//hold it until exit
		if (!frame) return false;

		bool has_color = frame_component & (int)FrameComponent::RGBA_IMAGE_1;
		bool has_depth = frame_component & (int)FrameComponent::DEPTH_IMAGE;
		bool has_ir = frame_component & (int)FrameComponent::INFRARED_IMAGE_1;
		bool has_depth2image_xy = (frame_component & (int)FrameComponent::DEPTH_TO_IMAGE_COORDINATE) && has_depth;
		bool has_depth2xyz = (frame_component & (int)FrameComponent::DEPTH_TO_XYZ) && has_depth;
		bool has_depth2rgba = (frame_component & (int)FrameComponent::DEPTH_TO_RGBA) && has_depth && has_color;

		//get color
		if(has_color)
		{
			IColorFrameReference* ref_frame = nullptr;
			frame->get_ColorFrameReference(&ref_frame);
			ResourceHolder<IColorFrameReference> _ref_frame(ref_frame);
			if (!ref_frame) return false;

			IColorFrame* content = nullptr;
			ref_frame->AcquireFrame(&content);
			ResourceHolder<IColorFrame> _content(content);
			if (!content) return false;

			IFrameDescription* desc = nullptr;
			content->get_FrameDescription(&desc);
			ResourceHolder<IFrameDescription> _desc(desc);
			float fov_x, fov_y;
			desc->get_HorizontalFieldOfView(&fov_x);
			desc->get_HorizontalFieldOfView(&fov_y);
			m_color_fov.horz_deg = fov_x;
			m_color_fov.vert_deg = fov_y;

			//copy data
			output->color_1.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC4);
			content->CopyConvertedFrameDataToArray(
				(UINT)(output->color_1.total() * output->color_1.channels()), output->color_1.data, 
				ColorImageFormat::ColorImageFormat_Rgba);
		}

		//get IR image
		if (has_ir) {
			IInfraredFrameReference* ref_frame = 0;
			frame->get_InfraredFrameReference(&ref_frame);
			ResourceHolder<IInfraredFrameReference> _ref_frame(ref_frame);
			if (!ref_frame) return false;

			IInfraredFrame* content = 0;
			ref_frame->AcquireFrame(&content);
			ResourceHolder<IInfraredFrame> _content(content);
			if (!content) return false;

			//get description
			//IFrameDescription* desc = 0;
			//content->get_FrameDescription(&desc);
			//ResourceHolder<IFrameDescription> _desc(desc);
			//int width, height;
			//unsigned int nb;
			//desc->get_Width(&width);
			//desc->get_Height(&height);
			//desc->get_BytesPerPixel(&nb);
			//spdlog::info("IR frame = ({0}, {1}, {2})", width, height, nb);

			//copy data
			output->ir_1.create(DEPTH_HEIGHT, DEPTH_WIDTH, CV_16UC1);
			content->CopyFrameDataToArray((UINT)output->ir_1.total(), (UINT16*)output->ir_1.data);
		}

		//get depth frame
		if(has_depth)
		{
			IDepthFrameReference* ref_frame = nullptr;
			frame->get_DepthFrameReference(&ref_frame);
			ResourceHolder<IDepthFrameReference> _ref_frame(ref_frame);
			if (!ref_frame) return false;

			IDepthFrame* content = nullptr;
			ref_frame->AcquireFrame(&content);
			ResourceHolder<IDepthFrame> _content(content);
			if (!content) return false;

			IFrameDescription* desc = nullptr;
			content->get_FrameDescription(&desc);
			ResourceHolder<IFrameDescription> _desc(desc);
			float fov_x, fov_y;
			desc->get_HorizontalFieldOfView(&fov_x);
			desc->get_HorizontalFieldOfView(&fov_y);
			m_depth_fov.horz_deg = fov_x;
			m_depth_fov.vert_deg = fov_y;

			output->depth.create(DEPTH_HEIGHT, DEPTH_WIDTH, CV_16UC1);
			output->depth.setTo(0);
			UINT sz;
			UINT16* buf;
			content->AccessUnderlyingBuffer(&sz, &buf);
			content->CopyFrameDataToArray((UINT)output->depth.total(), (UINT16*)output->depth.data);

			//convert depth to camera coordinate
			if (has_depth2xyz) {
				m_mapper->MapDepthFrameToCameraSpace(sz, buf, (UINT)m_cache_depth2xyz.size(), m_cache_depth2xyz.data());
				cv::Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_32FC3, m_cache_depth2xyz.data()).copyTo(output->depth2xyz);
			}

			//convert depth to image coordinate xy
			if (has_depth2image_xy || has_depth2rgba) {
				m_mapper->MapDepthFrameToColorSpace(sz, buf, (UINT)m_cache_depth2rgb_xy.size(), m_cache_depth2rgb_xy.data());
				cv::Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_32FC2, m_cache_depth2rgb_xy.data()).copyTo(output->depth2image_xy);
			}

			//convert depth to rgb
			if (has_depth2rgba) {
				//retrieve the rgb pixels
				output->depth2color.create(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC4);
				output->depth2color.setTo(0);
				for (int k = 0; k < output->depth2color.total(); k++) {
					auto x = m_cache_depth2rgb_xy[k].X;
					auto y = m_cache_depth2rgb_xy[k].Y;
					if (x >= 0 && x < IMAGE_WIDTH && y >= 0 && y < IMAGE_HEIGHT)
					{
						int i = (int)y;
						int j = (int)x;
						auto& pix = output->depth2color.at<cv::Vec4b>(k);
						pix = output->color_1.at<cv::Vec4b>(i, j);
						pix[3] = 255;
					}
				}
			}
		}

		return true;
	}

	void KinectReader::get_depth_camera_intrinsics(
		fMATRIX_3* intmat, fVECTOR_3* distortion_ks)
	{
		CameraIntrinsics ins;
		m_mapper->GetDepthCameraIntrinsics(&ins);
		
		fMATRIX_3 intrinsic_mat;
		intrinsic_mat.setIdentity();
		intrinsic_mat(0, 0) = ins.FocalLengthX;
		intrinsic_mat(1, 1) = ins.FocalLengthY;
		intrinsic_mat(2, 0) = ins.PrincipalPointX;
		intrinsic_mat(2, 1) = ins.PrincipalPointY;
		if (intmat)
			*intmat = intrinsic_mat;

		fVECTOR_3 ks = fVECTOR_3::Zero();
		ks[0] = ins.RadialDistortionSecondOrder;
		ks[1] = ins.RadialDistortionFourthOrder;
		ks[2] = ins.RadialDistortionSixthOrder;
		if (distortion_ks)
			*distortion_ks = ks;
	}

	fMATRIX_3 KinectReader::get_depth_camera_intrinsic_matrix() const
	{
		fMATRIX_3 intmat = fMATRIX_3::Identity();
		double pi = std::acos(-1.0);
		double fov = m_depth_fov.horz_deg / 180 * pi;
		double focal = double(DEPTH_WIDTH) / 2 / std::tan(fov / 2);
		double cx = DEPTH_WIDTH / 2.0;
		double cy = DEPTH_HEIGHT / 2.0;
		intmat(0, 0) = focal;
		intmat(1, 1) = focal;
		intmat(2, 0) = cx;
		intmat(2, 1) = cy;
		return intmat;
	}

	fMATRIX_3 KinectReader::get_color_camera_intrinsic_matrix() const
	{
		fMATRIX_3 intmat = fMATRIX_3::Identity();
		double pi = std::acos(-1.0);
		double fov = m_color_fov.horz_deg / 180 * pi;
		double focal = double(IMAGE_WIDTH) / 2 / std::tan(fov / 2);
		double cx = IMAGE_WIDTH / 2.0;
		double cy = IMAGE_HEIGHT / 2.0;
		intmat(0, 0) = focal;
		intmat(1, 1) = focal;
		intmat(2, 0) = cx;
		intmat(2, 1) = cy;
		return intmat;
	}

	KinectReader::~KinectReader()
	{
		close();
	}

	const int KinectReader::DEPTH_HEIGHT = 424;
	const int KinectReader::DEPTH_WIDTH = 512;
	const int KinectReader::IMAGE_HEIGHT = 1080;
	const int KinectReader::IMAGE_WIDTH = 1920;
}