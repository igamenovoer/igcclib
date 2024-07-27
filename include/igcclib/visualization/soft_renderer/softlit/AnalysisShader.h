#pragma once

#include <igcclib/visualization/soft_renderer/softlit/Master.h>
#include <igcclib/visualization/soft_renderer/softlit/Shaders.h>

namespace softlit {
	class AnalysisVertexShader : public SoftVertexShader {
	public:
		virtual glm::vec4 do_shading(
			const glm::vec3& pos, 
			const UBO* ubo,
			const Vertex_IN* in, 
			Vertex_OUT* out) const override;
		static AnalysisVertexShader& get_default_shader();
		static std::shared_ptr<AnalysisVertexShader> get_default_shader_ptr();
	};

	class AnalysisFragmentShader : public SoftFragmentShader {
	public:
		virtual void do_shading(
			glm::vec4* output, 
			const SoftVertexShader* vs, 
			const UBO* ubo,
			const Vertex_OUT* in, 
			Texture** tbo) const override;
		static AnalysisFragmentShader& get_default_shader();
		static std::shared_ptr<AnalysisFragmentShader> get_default_shader_ptr();
	};
}
