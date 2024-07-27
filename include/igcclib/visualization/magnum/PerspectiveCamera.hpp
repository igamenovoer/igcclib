#pragma once

#include <igcclib/visualization/magnum/igcclib_magnum_def.hpp>
#include <igcclib/visualization/magnum/SceneNode.hpp>
#include <igcclib/visualization/magnum/CameraBase.hpp>
#include <Magnum/SceneGraph/Camera.h>

namespace _NS_UTILITY {
	namespace MagnumProc {
		class PerspectiveCamera :public CameraBase {
		public:
			using CameraBase::CameraBase;

			/**
			* \brief set the frustrum of this camera
			*
			* \param fov_horz_degree the horizontal fov in degree
			* \param aspect_ratio_wh the aspect ratio, defined as width/height
			* \param near_plane near plane distance
			* \param far_plane far plane distance
			* \return void
			*/
			void set_frustrum(Magnum::Float fov_horz_degree, 
				Magnum::Float aspect_ratio_wh, 
				Magnum::Float near_plane = 0.1,
				Magnum::Float far_plane = 1e4) 
			{
				Magnum::Matrix4 projmat = Magnum::Matrix4::perspectiveProjection(
					Magnum::Deg(fov_horz_degree), aspect_ratio_wh, near_plane, far_plane
				);
				this->setProjectionMatrix(projmat);
			}

			virtual CameraType get_camera_type() const override{
				return CameraType::PERSPECTIVE;
			}
		};
	}
}
