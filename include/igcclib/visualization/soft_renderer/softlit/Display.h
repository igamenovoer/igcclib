#pragma once
//#include <opencv2/core.hpp>
#include <vector>
#include <memory>
#include <glm/vec4.hpp>

//MODIFY
namespace softlit
{
	class Image;
	class Display
	{
	public:
		Display(const uint32_t w, const uint32_t h, bool fullscreen);
		~Display();

		/* Clear depth & frame buffers before re-draw */
		void ClearRenderTarget();

		// Update display color buffer with frame buffer content before presenting
		void UpdateColorBuffer(const std::vector<glm::vec4>& frameBuffer);

		//by defualt, create RGBA image
		void Init(int width, int height, int n_channel = 4);

	private:
		// Color buffer that SDL will use to present frame buffer content
		std::unique_ptr<Image> m_image;
	};
}
