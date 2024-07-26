#pragma once

#include "def_magnum.h"
#include "SceneNode.h"
#include "CameraBase.h"
#include <Magnum/SceneGraph/Camera.h>

namespace _NS_UTILITY {
	namespace MagnumProc {
		class OrthographicCamera : public CameraBase{
		public:
			using CameraBase::CameraBase;

			/** \brief set the view size */
			void set_frustrum(Magnum::Float width, Magnum::Float height,
				Magnum::Float near_plane = 1e-3, Magnum::Float far_plane = 1e4) {
				auto projmat = Magnum::Matrix4::orthographicProjection({ width, height }, near_plane, far_plane);
				setProjectionMatrix(projmat);
			}

			virtual CameraType get_camera_type() const override {
				return CameraType::ORTHOGRAPHIC;
			}
		};
	}
}
