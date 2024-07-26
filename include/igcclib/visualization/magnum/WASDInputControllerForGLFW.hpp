#pragma once

#include <chrono>
#include <functional>

#include "igcclib_magnum_def.hpp"
#include "FPSCameraController.hpp"
#include <GLFW/glfw3.h>


namespace _NS_UTILITY {
	namespace MagnumProc {
		class WASDInputControllerForGLFW {
		public:
			void set_window(GLFWwindow* window);
			void set_camera_controller(FPSCameraController* obj);
			void remove_camera_controller();
			void set_enable(bool tf) { 
				assert_throw(m_window, "cannot set enable before window is set");

				if (tf != m_enable)
					m_state_initialized = false;

				m_enable = tf; 
				if (m_enable)
					glfwSetInputMode(m_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
				else
					glfwSetInputMode(m_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			}
			bool get_enable() const { return m_enable; }
			void set_movement_speed_per_s(Magnum::Float sp) {
				m_speed_move_distance_per_s = sp;
			}
			void set_rotation_speed_per_s(Magnum::Deg deg) {
				m_speed_rotate_deg_per_s = deg;
			}

			void update();
		private:
			using TimePoint_t = decltype(std::chrono::system_clock::now());

			bool m_enable = false;

			//when moving, move by this distance per second
			Magnum::Float m_speed_move_distance_per_s = 1.0;

			//when rotating, rotate by this degree per second
			Magnum::Deg m_speed_rotate_deg_per_s{ 10.0 };

			//the glfw window
			GLFWwindow* m_window = nullptr;

			//state tracking over time
			//is the current tracking state initialized?
			//state tracking needs to be initialized for the first frame
			bool m_state_initialized = false;

			//previous mouse position (x,y)
			std::pair<Magnum::Float, Magnum::Float> m_prev_mouse_position{ 0,0 };	

			//previous time
			TimePoint_t m_prev_time;

			//the state of the enabler key
			int m_keystate_enabler_switch = GLFW_RELEASE;

			//called when the head is going to rotate left and up		
			std::function<void(const Magnum::Rad&, const Magnum::Rad&)> m_action_rotate_left_up;

			//called when we want to move forward
			std::function<void(Magnum::Float)> m_action_move_forward;

			//called when we want to move the left (strafing)
			std::function<void(Magnum::Float)> m_action_move_left;
		};

		inline void WASDInputControllerForGLFW::set_window(GLFWwindow* window)
		{
			m_window = window;
			m_state_initialized = false;
		}

		inline void WASDInputControllerForGLFW::set_camera_controller(FPSCameraController* obj)
		{
			using namespace std::placeholders;
			m_action_rotate_left_up = std::bind(&FPSCameraController::rotate_by, obj, _1, _2);
			m_action_move_forward = std::bind(&FPSCameraController::move_forward_by, obj, _1);
			m_action_move_left = std::bind(&FPSCameraController::move_left_by, obj, _1);
			m_state_initialized = false;
		}

		inline void WASDInputControllerForGLFW::remove_camera_controller()
		{
			m_action_move_forward = nullptr;
			m_action_move_left = nullptr;
			m_action_rotate_left_up = nullptr;
		}

		void WASDInputControllerForGLFW::update()
		{
			if (!m_window) return;
			//enabler is always checked
			int enabler_keystate = glfwGetKey(m_window, GLFW_KEY_SPACE);
			if (m_keystate_enabler_switch == GLFW_PRESS && enabler_keystate == GLFW_RELEASE)
				set_enable(!m_enable);
			m_keystate_enabler_switch = enabler_keystate;

			if (!m_enable) return;
			//query for mouse movement
			double xpos, ypos;
			glfwGetCursorPos(m_window, &xpos, &ypos);

			if (!m_state_initialized)
			{
				//initialize state in this round
				m_prev_mouse_position.first = xpos;
				m_prev_mouse_position.second = ypos;
				m_prev_time = std::chrono::system_clock::now();
				m_state_initialized = true;
				return;
			}

			auto time_now = std::chrono::system_clock::now();
			double delta_second = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - m_prev_time).count() / 1000.0;

			auto dx = xpos - m_prev_mouse_position.first;
			auto dy = ypos - m_prev_mouse_position.second;
			if (m_action_rotate_left_up)
			{
				auto left = - dx * m_speed_rotate_deg_per_s * delta_second;
				auto up = - dy * m_speed_rotate_deg_per_s * delta_second;
				m_action_rotate_left_up(left, up);
			}

			//query for moving forward
			double move_forward_dist = 0;
			if (glfwGetKey(m_window, GLFW_KEY_W) == GLFW_PRESS) {
				move_forward_dist = m_speed_move_distance_per_s * delta_second;
			}
			if (glfwGetKey(m_window, GLFW_KEY_S) == GLFW_PRESS) {
				move_forward_dist = - m_speed_move_distance_per_s * delta_second;
			}
			if (m_action_move_forward)
				m_action_move_forward(move_forward_dist);

			//query for moving left
			double move_left_dist = 0;
			if (glfwGetKey(m_window, GLFW_KEY_A) == GLFW_PRESS){
				move_left_dist = m_speed_move_distance_per_s * delta_second;
			}
			if (glfwGetKey(m_window, GLFW_KEY_D) == GLFW_PRESS) {
				move_left_dist = - m_speed_move_distance_per_s * delta_second;
			}
			if (m_action_move_left)
				m_action_move_left(move_left_dist);

			//update state tracking
			m_prev_mouse_position = { xpos, ypos };
			m_prev_time = std::chrono::system_clock::now();
		}

	}
}