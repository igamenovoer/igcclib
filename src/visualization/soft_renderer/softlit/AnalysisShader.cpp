#include <igcclib/visualization/soft_renderer/softlit/AnalysisShader.h>

namespace softlit {

	glm::vec4 AnalysisVertexShader::do_shading(
		const glm::vec3& pos, const UBO* ubo, const Vertex_IN* in, Vertex_OUT* out) const
	{
		//interpolate uv
		{
			auto idx = get_input_mapping(ShaderInputType::UV_2);
			glm::vec2 uv(1, 1);
			if (idx >= 0)
				uv = in->attrib_vec2[idx];
			out->PushVertexAttribute(uv);
		}

		//interpolate color
		{
			auto idx = get_input_mapping(ShaderInputType::VERTEX_COLOR_3);
			glm::vec3 color(1, 1, 1);
			if(idx >= 0)
				color = in->attrib_vec3[idx];
			out->PushVertexAttribute(color);
		}

		//interpolate normal
		{
			auto idx = get_input_mapping(ShaderInputType::VERTEX_NORMAL_3);
			glm::vec3 normal(1, 0, 0);
			if (idx >= 0)
				normal = in->attrib_vec3[idx];

			//world space normal
			glm::vec3 world_normal = glm::inverse(glm::transpose(glm::mat3(ubo->M))) * normal;	//convert to world space
			world_normal = glm::normalize(world_normal);
			out->PushVertexAttribute(world_normal);

			//view space normal
			glm::vec3 view_normal = glm::inverse(glm::transpose(glm::mat3(ubo->MV))) * normal;
			view_normal = glm::normalize(view_normal);
			out->PushVertexAttribute(view_normal);
		}

		//interpolate local position
		out->PushVertexAttribute(pos);

		return (ubo->MVP * glm::vec4(pos, 1));
	}

	AnalysisVertexShader& AnalysisVertexShader::get_default_shader()
	{
		static AnalysisVertexShader shader;
		shader.set_input_mapping(ShaderInputType::UV_2, DefaultShaderInputChannel<ShaderInputType::UV_2>::index);
		shader.set_input_mapping(ShaderInputType::VERTEX_COLOR_3, DefaultShaderInputChannel<ShaderInputType::VERTEX_COLOR_3>::index);
		shader.set_input_mapping(ShaderInputType::VERTEX_NORMAL_3, DefaultShaderInputChannel<ShaderInputType::VERTEX_NORMAL_3>::index);
		return shader;
	}

	std::shared_ptr<AnalysisVertexShader> AnalysisVertexShader::get_default_shader_ptr()
	{
		static std::shared_ptr<softlit::AnalysisVertexShader> p(
			&get_default_shader(), [](void*) {}
		);
		return p;
	}

	void AnalysisFragmentShader::do_shading(
		glm::vec4* output, const SoftVertexShader* vs, 
		const UBO* ubo,
		const Vertex_OUT* in, Texture** tbo) const
	{
		//render uv
		glm::vec2 uv;
		{
			uv = in->attrib_vec2[0];
			auto out_index = get_output_mapping(ShaderOutputChannel::UV);
			if(out_index >= 0)
				output[out_index] = glm::vec4(uv, 0, 1);
		}

		//render color
		glm::vec3 v_color;
		{
			auto out_index = get_output_mapping(ShaderOutputChannel::VERTEX_COLOR);
			v_color = in->attrib_vec3[0];
			if(out_index >= 0)
				output[out_index] = glm::vec4(v_color, 1);
		}

		//render normal
		glm::vec3 world_normal;
		{
			world_normal = glm::normalize(in->attrib_vec3[1]);
			auto out_index = get_output_mapping(ShaderOutputChannel::WORLD_SPACE_NORMAL);
			if (out_index >= 0)
				output[out_index] = glm::vec4(world_normal, 1);
		}

		glm::vec3 view_normal;
		{
			view_normal = glm::normalize(in->attrib_vec3[2]);
			auto out_index = get_output_mapping(ShaderOutputChannel::VIEW_SPACE_NORMAL);
			if (out_index >= 0)
				output[out_index] = glm::vec4(view_normal, 1);
		}

		//render position
		glm::vec3 position;
		{
			auto out_index_world = get_output_mapping(ShaderOutputChannel::WORLD_POSITION);
			auto out_index_view = get_output_mapping(ShaderOutputChannel::VIEW_POSITION);
			position = in->attrib_vec3[3];

			if (out_index_world >= 0)
			{
				glm::vec4 world_position = ubo->M * glm::vec4(position, 1);
				world_position /= world_position.w;
				output[out_index_world] = world_position;
			}

			if (out_index_view >= 0) {
				glm::vec4 view_position = ubo->MV * glm::vec4(position, 1);
				view_position /= view_position.w;
				output[out_index_view] = view_position;
			}
		}

		//get textured color
		if (tbo) {
			auto nch = tbo[0]->get_image().get_num_channels();
			auto out_index = get_output_mapping(ShaderOutputChannel::FINAL_COLOR);
			if (nch == 3)
			{
				const glm::vec3 texcol = tbo[0]->Sample<TextureSampler::SAMPLE_RGB>(uv);
				output[out_index] = glm::vec4(texcol, 1);
			}
			else {
				//nch==4
				glm::vec4 texcol = tbo[0]->Sample<TextureSampler::SAMPLE_RGBA>(uv);
				output[out_index] = texcol;
			}
		}
		else {
			auto out_index = get_output_mapping(ShaderOutputChannel::FINAL_COLOR);
			output[out_index] = glm::vec4(v_color, 1);
		}
	}

	AnalysisFragmentShader& AnalysisFragmentShader::get_default_shader()
	{
		static AnalysisFragmentShader shader;
		shader.set_output_channel(ShaderOutputChannel::FINAL_COLOR, (int)ShaderOutputChannel::FINAL_COLOR);
		shader.set_output_channel(ShaderOutputChannel::UV, (int)ShaderOutputChannel::UV);
		shader.set_output_channel(ShaderOutputChannel::VERTEX_COLOR, (int)ShaderOutputChannel::VERTEX_COLOR);
		shader.set_output_channel(ShaderOutputChannel::WORLD_SPACE_NORMAL, (int)ShaderOutputChannel::WORLD_SPACE_NORMAL);
		shader.set_output_channel(ShaderOutputChannel::VIEW_SPACE_NORMAL, (int)ShaderOutputChannel::VIEW_SPACE_NORMAL);
		shader.set_output_channel(ShaderOutputChannel::WORLD_POSITION, (int)ShaderOutputChannel::WORLD_POSITION);
		shader.set_output_channel(ShaderOutputChannel::VIEW_POSITION, (int)ShaderOutputChannel::VIEW_POSITION);
		return shader;
	}

	std::shared_ptr<AnalysisFragmentShader> AnalysisFragmentShader::get_default_shader_ptr()
	{
		static std::shared_ptr<AnalysisFragmentShader> p(
			&get_default_shader(), [](void*) {}
		);
		return p;
	}

}