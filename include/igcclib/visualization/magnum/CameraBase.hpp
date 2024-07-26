#pragma once

#include "igcclib_magnum_def.hpp"
#include "SceneNode.hpp"
#include <Magnum/SceneGraph/Camera.h>

namespace _NS_UTILITY {
	namespace MagnumProc {
		class IGCCLIB_API CameraBase : public SceneNode, public Magnum::SceneGraph::Camera3D {
		public:
			CameraBase() :
				SceneNode{}, Magnum::SceneGraph::Camera3D{ *this }{}
			CameraBase(Object3D* parent) :
				SceneNode{ parent }, Magnum::SceneGraph::Camera3D{ *this }{}

			virtual CameraType get_camera_type() const = 0;
		};
	}
}
