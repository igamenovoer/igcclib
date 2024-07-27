#pragma once

#include <map>
#include <igcclib/visualization/soft_renderer/softlit/Master.h>
#include <igcclib/visualization/soft_renderer/softlit/Texture.h>

namespace softlit
{
	class Texture;

	class SoftVertexShader {
	public:
		using InputChannelMapping = std::map<ShaderInputType, int>;
		virtual ~SoftVertexShader(){}

		/**
		* \brief do vertex shading
		*
		* \param pos position of the vertex
		* \param ubo the transformation matrices
		* \param in	the input vertex attributes
		* \param out	the output vertex attributes
		* \return vec4 output position of the vertex
		*/
		virtual glm::vec4 do_shading(
			const glm::vec3& pos, 
			const UBO* ubo, 
			const Vertex_IN* in, 
			Vertex_OUT* out) const = 0;

		virtual void set_input_mapping(ShaderInputType input_type, int idx_channel) {
			m_input_mapping[input_type] = idx_channel;
		}

		/** \brief return -1 if the input channel has no mapping */
		virtual int get_input_mapping(ShaderInputType input_type) const {
			auto it = m_input_mapping.find(input_type);
			if (it == m_input_mapping.end())
				return -1;
			else
				return it->second;
		}

		const InputChannelMapping& get_input_mapping() const {
			return m_input_mapping;
		}

	protected:
		InputChannelMapping m_input_mapping;
	};

	class SoftFragmentShader {
	public:
		using OutputChannelMapping = std::map<ShaderOutputChannel, int>;
		virtual ~SoftFragmentShader() {}
		SoftFragmentShader(){
			m_output_mapping[ShaderOutputChannel::FINAL_COLOR] = 0;
		}

		/**
		* \brief do fragment shading
		*
		* \param output the output fragment shading result. It must have enough storage to hold
		outputs of all channels, that is, output[num_output_channel()-1] must be available.
		* \param vs the vertex shader
		* \param in	interpolated vertex attributes from the vertex shader
		* \param tbo input texture data
		*/
		virtual void do_shading(
			glm::vec4* output, 
			const SoftVertexShader* vs, 
			const UBO* ubo, 
			const Vertex_OUT* in, 
			Texture** tbo) const = 0;

		virtual void set_output_channel(ShaderOutputChannel output_type, int idx_channel) {
			m_output_mapping[output_type] = idx_channel;
		}

		/** \brief return -1 if the output has no mapping */
		virtual int get_output_mapping(ShaderOutputChannel output_type) const {
			auto it = m_output_mapping.find(output_type);
			if (it == m_output_mapping.end())
				return -1;
			else
				return it->second;
		}		

		virtual const OutputChannelMapping& get_output_mapping() const {
			return m_output_mapping;
		}

		/** \brief number of output channels, useful for allocating memory for do_shading() output */
		virtual int num_output_channel() const {
			return (int)m_output_mapping.size();
		}

	protected:
		OutputChannelMapping m_output_mapping;
	};

	// VS: Out normal
	inline glm::vec4 VS_Simple(const glm::vec3& pos, const UBO* const ubo, const Vertex_IN* const in, Vertex_OUT* out)
	{
		out->PushVertexAttribute(in->attrib_vec3[0]); //push color

		return (ubo->MVP * glm::vec4(pos, 1));
	}

	// FS: Render scaled vertex normals
	inline glm::vec4 FS_Simple(const UBO* const ubo, const Vertex_OUT* const in, Texture** tbo)
	{
		//const vec3 outcol = in->attrib_vec3[0] * 0.5f + 0.5f;
		const glm::vec3 outcol = in->attrib_vec3[0];

		return glm::vec4(outcol, 1);
	}

	inline glm::vec4 VS_Textured(const glm::vec3& pos, const UBO* const ubo, const Vertex_IN* const in, Vertex_OUT* out)
	{
		out->PushVertexAttribute(in->attrib_vec2[0]); // Push uv

		return (ubo->MVP * glm::vec4(pos, 1));
	}

	inline glm::vec4 FS_Textured(const UBO* const ubo, const Vertex_OUT* const in, Texture** tbo)
	{
		auto nch = tbo[0]->get_image().get_num_channels();
		if (nch==3)
		{
			const glm::vec3 texcol = tbo[0]->Sample<TextureSampler::SAMPLE_RGB>(in->attrib_vec2[0]);
			return glm::vec4(texcol, 1);
		}
		else { 
			//nch==4
			glm::vec4 texcol = tbo[0]->Sample<TextureSampler::SAMPLE_RGBA>(in->attrib_vec2[0]);
			return texcol;
		}
	}
}
