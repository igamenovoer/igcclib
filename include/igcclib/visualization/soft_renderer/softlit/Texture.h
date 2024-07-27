#pragma once
#include <algorithm>
#include <cstdint>
#include <igcclib/visualization/soft_renderer/softlit/Utils.h>

namespace softlit
{
	static constexpr float g_texColDiv = (1 / 255.f);

	class Texture;

	class Image
	{
	public:
		//MODIFY
		friend class Texture;

		/**
		* \brief allocate the pixel buffer, and copy imgdata into it
		*
		* \param imgdata the pixels to be copied into the image, must be [rgbargba...] format.
		If nullptr, nothing is copied, only allocate memory.
		* \param width width of the image
		* \param height height of the image
		* \param n_channel number of channels of the image
		*/
		void Init(const uchar* imgdata, int32_t width, int32_t height, int32_t n_channel);

		/** \brief get modifiable image data */
		uint8_t* get_data() {
			const auto* p = this;
			return const_cast<uint8_t*>(p->get_data());
		}

		/** \brief get image data */
		const uint8_t* get_data() const {
			if (m_data.size() == 0)
				return nullptr;
			else
				return m_data.data();
		}

		/** \brief fill the image with a constant value */
		void fill(uint8_t val) {
			std::fill(m_data.begin(), m_data.end(), val);
		}

		int32_t get_num_channels() const { return m_numChannels; }
		int32_t get_width() const { return m_width; }
		int32_t get_height() const { return m_height; }

	private:
		//leave it here for compatibility
		//uchar* m_data = nullptr; // Image data

		std::vector<uchar> m_data;
		int32_t m_width = 0;
		int32_t m_height = 0;
		int32_t m_numChannels = 0;
	};

	enum TextureSampler
	{
		SAMPLE_RGB = 3,
		SAMPLE_RGBA = 4
	};

	/* 2D Texture */
	class Texture
	{
	public:
		Texture(const Image& image);
		~Texture();

		//TODO: Filter & address modes

		template<TextureSampler T>
		glm::vec<T, float> Sample(const glm::vec2& uv) const;

		const Image& get_image() const { return m_image; }

	private:
		Image m_image;
	};

	template<>
	inline glm::vec4 Texture::Sample<TextureSampler::SAMPLE_RGBA>(const glm::vec2& uv) const
	{
		DBG_ASSERT(m_image.m_numChannels == 4);

		//TODO: FILTER!!!
		uint32_t idxS = (uint32_t)glm::floor(uv.s * (m_image.m_width-1));
		uint32_t idxT = (uint32_t)glm::floor(uv.t * (m_image.m_height-1));
		uint32_t idx = (idxT * m_image.m_width + idxS) * m_image.m_numChannels;

		float r = (float)(m_image.m_data[idx++]);
		float g = (float)(m_image.m_data[idx++]);
		float b = (float)(m_image.m_data[idx++]);
		float a = (float)(m_image.m_data[idx++]);

		return glm::vec4(r, g, b, a) * g_texColDiv;
	}

	template<>
	inline glm::vec3 Texture::Sample<TextureSampler::SAMPLE_RGB>(const glm::vec2& uv) const
	{
		DBG_ASSERT(m_image.m_numChannels == 3);

		//TODO: FILTER!!!
		uint32_t idxS = (uint32_t)glm::floor(uv.s * (m_image.m_width-1));
		uint32_t idxT = (uint32_t)glm::floor(uv.t * (m_image.m_height-1));
		uint32_t idx = (idxT * m_image.m_width + idxS) * m_image.m_numChannels;

		float r = (float)(m_image.m_data[idx++]);
		float g = (float)(m_image.m_data[idx++]);
		float b = (float)(m_image.m_data[idx++]);

		return glm::vec3(r, g, b) * g_texColDiv;
	}
}