#pragma once

#include <memory>
#include <functional>

#ifndef NOMINMAX
	#define NOMINMAX //temporary use nominmax to avoid polluting namespace
	#include <Kinect.h>
	#undef NOMINMAX
#else
	#include <Kinect.h>
#endif

#include <igcclib/device/igcclib_device_def.hpp>
#include <igcclib/device/DeviceReader.hpp>
#include <igcclib/vision/igcclib_opencv_def.hpp>
#include <igcclib/core/igcclib_eigen_def.hpp>

namespace _NS_UTILITY {
	class KinectReader : public DeviceReader {

	public:
		template<typename T>
		using UniquePtr_t = std::unique_ptr<T, void(*)(T*)>;

		/** \brief get a frame, return whether the operation is successful */
		bool get_frame(FrameData* output, int frame_component = ~0);

		/** \brief get instrinsic matrix and distortion factors of the depth camera.
		The intrinsic camera is of right-mul format. Distortion factors are k1,k2,k3
		as in opencv convention.*/
		void get_depth_camera_intrinsics(fMATRIX_3* intmat, fVECTOR_3* distortion_ks);

		virtual bool init(void* opt) override;
		virtual void open() override;
		virtual void close() override;
		virtual void destroy() override;
		virtual fMATRIX_3 get_depth_camera_intrinsic() const override;
		virtual fMATRIX_3 get_color_camera_intrinsic() const override;

	public:
		~KinectReader();

	public:
		template<typename T>
		static void safe_release(T* p) {
			if (p) p->Release();
		}

		static const int IMAGE_WIDTH, IMAGE_HEIGHT, DEPTH_WIDTH, DEPTH_HEIGHT;

	private:
		fMATRIX_3 get_depth_camera_intrinsic_matrix() const;
		fMATRIX_3 get_color_camera_intrinsic_matrix() const;

		std::shared_ptr<IKinectSensor> m_sensor;
		std::shared_ptr<ICoordinateMapper> m_mapper;
		std::shared_ptr<IMultiSourceFrameReader> m_reader_multi_src;

		//cache
		std::vector<ColorSpacePoint> m_cache_depth2rgb_xy{ DEPTH_WIDTH * DEPTH_HEIGHT };
		std::vector<CameraSpacePoint> m_cache_depth2xyz{ DEPTH_WIDTH * DEPTH_HEIGHT };

		//intrinsics read from frames
		struct Fov {
			double horz_deg = 0;
			double vert_deg = 0;
		};
		Fov m_depth_fov;
		Fov m_color_fov;
	};
}