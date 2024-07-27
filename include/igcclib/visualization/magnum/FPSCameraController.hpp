#pragma once

#include <igcclib/visualization/magnum/igcclib_magnum_def.hpp>
#include <igcclib/visualization/magnum/CameraBase.hpp>

namespace _NS_UTILITY {
	namespace MagnumProc {

		/** \brief control a camera like in FPS game, you can move and rotate your head */
		class FPSCameraController {
		public:
			FPSCameraController() {}
			FPSCameraController(CameraBase* camera) {
				set_camera(camera);
			}

			/** \brief set the camera to be controlled, using the camera Y+ axis as the up vector */
			void set_camera(CameraBase* camera) {
				m_camera = camera;
				
				Magnum::Vector3 local_up_vector{ 0,1,0 };
				m_up_vector = camera->convert_vector(local_up_vector, CoordinateSystem::SELF, CoordinateSystem::PARENT);
				camera->get_orientation(m_initial_orientation, CoordinateSystem::PARENT);
			}

			/**
			* \brief set the orientation of the camera, by first rotating left (around up vector), and then rotating up (around local x+)
			*
			* \param left the angle of rotating left
			* \param up the angle of rotating up
			*/
			void set_orientation(const Magnum::Rad& left, const Magnum::Rad& up) {
				//reset orientation
				m_camera->set_orientation(m_initial_orientation, CoordinateSystem::PARENT);

				rotate_by(left, up);
			}

			/**
			* \brief rotate the camera to the left (around up vector), and then up (around local x+)
			*
			* \param left the angle of rotating left
			* \param up the angle of rotating up
			*/
			void rotate_by(const Magnum::Rad& left , const Magnum::Rad& up) {
				//rotate around up vector
				auto pos = m_camera->get_position();
				m_camera->set_position(Magnum::Vector3(0,0,0));
				m_camera->rotate(left, m_up_vector);
				m_camera->set_position(pos);

				//rotate around local x+
				m_camera->rotateXLocal(up);
			}

			/** \brief move the camera forward by a specified distance, the forward vector is camera's z- */
			void move_forward_by(Magnum::Float distance) {
				m_camera->translateLocal(Magnum::Vector3(0, 0, -distance));
			}

			/** \brief strafe the camera left by a specified distance, the left axis is camera's x- */
			void move_left_by(Magnum::Float distance) {
				m_camera->translateLocal(Magnum::Vector3(-distance, 0, 0));
			}

		private:
			CameraBase* m_camera = nullptr;
			Magnum::Vector3 m_up_vector{ 0,1,0 }; //up vector relative to the camera's parent node
			Magnum::Matrix3 m_initial_orientation; //initial orientation of the camera relative to parent
		};
	}
}
