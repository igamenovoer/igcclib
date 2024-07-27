#include <igcclib/visualization/soft_renderer/softlit/Master.h>
#include <igcclib/visualization/soft_renderer/softlit/Texture.h>

namespace softlit
{
	Texture::Texture(const Image& image)
		: m_image(image)
	{
	}

	Texture::~Texture()
	{
	}



	void Image::Init(const uchar* imgdata, int32_t width, int32_t height, int32_t n_channel)
	{
		auto n = width * height * n_channel;
		if (imgdata)
			m_data.assign(imgdata, imgdata + n);
		else
			m_data.resize(n);

		m_width = width;
		m_height = height;
		m_numChannels = n_channel;
	}

}