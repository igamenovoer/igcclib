#include <igcclib/visualization/soft_renderer/softlit/Master.h>
#include <igcclib/visualization/soft_renderer/softlit/Display.h>
#include <igcclib/visualization/soft_renderer/softlit/Texture.h>

using std::vector;

namespace softlit
{
    Display::Display(const uint32_t w, const uint32_t h, bool fullscreen)
    {
		Init(w, h);
    }

    Display::~Display()
    {
    }

    void Display::ClearRenderTarget()
    {
		m_image->fill(0);
    }

    void Display::UpdateColorBuffer(const vector<glm::vec4>& frameBuffer)
	{        
		auto imgdata = m_image->get_data();

#pragma omp parallel for
		for (int i = 0; i < frameBuffer.size(); i++) {
			auto pix = imgdata + 4 * i;
			const auto& v = frameBuffer[i];
			pix[0] = uint8_t(v.r * 255);
			pix[1] = uint8_t(v.g * 255);
			pix[2] = uint8_t(v.b * 255);
			pix[3] = uint8_t(v.a * 255);
		}
    }

	void Display::Init(int width, int height, int n_channel /*= 4*/)
	{
		m_image.reset(new Image);
		m_image->Init(nullptr, width, height, n_channel);
		m_image->fill(0);
	}

}