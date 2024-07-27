#pragma once

#include <igcclib/visualization/soft_renderer/softlit/Primitive.h>

namespace softlit
{
	class MultiChannelFrameBuffer {
	public:
		using ChannelData = std::vector<glm::vec4>;

		/** \brief initialize the framebuffer with image size and number of channels.
		Each channel contains an image whose pixel is glm::vec4 */
		void init(int width, int height, const std::vector<ShaderOutputChannel>& channels) {
			m_data.resize(NUM_SHADER_OUTPUT_CHANNEL);
			for (auto ch : channels)
			{
				m_data[(int)ch].resize(width*height);
			}
			m_width = width;
			m_height = height;
		}

		size_t get_num_channels() const {
			size_t output = 0;
			for (const auto& x : m_data)
				output += x.size() > 0;
			return output;
		}

		int get_width() const { return m_width; }
		int get_height() const { return m_height; }

		void clear_color(ShaderOutputChannel channel_id, const glm::vec4& color) {
			for (auto& value : m_data[(int)channel_id])
				value = color;
		}

		void clear_color(const glm::vec4& color) {
			for (auto& chn : m_data)
				for (auto& x : chn)
					x = color;
		}

		void clear_color() {
			glm::vec4 zero(0, 0, 0, 0);
			clear_color(zero);
		}

		bool has_channel(ShaderOutputChannel channel_id) const {
			int idx = (int)channel_id;
			return m_data.size() > idx && m_data[idx].size() > 0;
		}

		void set_color(ShaderOutputChannel channel_id, int pix_i, int pix_j, const glm::vec4& color) {
			m_data[(int)channel_id][pix_i * m_width + pix_j] = color;
		}

		glm::vec4& get_color(ShaderOutputChannel channel_id, int pix_i, int pix_j) {
			return m_data[(int)channel_id][pix_i * m_width + pix_j];
		}

		const glm::vec4& get_color(ShaderOutputChannel channel_id, int pix_i, int pix_j) const{
			return m_data[(int)channel_id][pix_i * m_width + pix_j];
		}

		const ChannelData& get_channel(ShaderOutputChannel channel_id) const {
			return m_data.at((int)channel_id);
		}

		std::map<ShaderOutputChannel, ChannelData*> get_all_data() { 
			std::map<ShaderOutputChannel, ChannelData*> output;
			for (size_t i = 0; i < m_data.size(); i++) {
				if (m_data[i].size() > 0) {
					output[(ShaderOutputChannel)i] = &m_data[i];
				}
			}
			return output;
		}

		std::map<ShaderOutputChannel, const ChannelData*> get_all_data() const {
			std::map<ShaderOutputChannel, const ChannelData*> output;
			for (size_t i = 0; i < m_data.size(); i++) {
				if (m_data[i].size() > 0) {
					output[(ShaderOutputChannel)i] = &m_data[i];
				}
			}
			return output;
		}

	private:
		std::vector<ChannelData> m_data;
		int m_width = 0;
		int m_height = 0;
	};

	//typedef std::vector<glm::vec4> FrameBuffer;
	using FrameBuffer = MultiChannelFrameBuffer;
	typedef std::vector<float> DepthBuffer;

	class Rasterizer
	{
	public:
		Rasterizer(const RasterizerSetup& setup);
		~Rasterizer();

		const FrameBuffer& getFrameBuffer() const { return m_frameBuffer; }

		/* Rasterize a given 3-D primitive object by transforming its vertices to clip-space via VS, depth testing and calculating fragment output color via FS */
		void Draw(Primitive* prim);

		/* Clear depth and frame buffers pre-draw */
		void ClearBuffers(const glm::vec4& clearColor, const float depthValue = FLT_MAX)
		{
			m_frameBuffer.clear_color(clearColor);
			//fill(m_frameBuffer.begin(), m_frameBuffer.end(), clearColor);
			fill(m_depthBuffer.begin(), m_depthBuffer.end(), depthValue);
		}

	private:
		FrameBuffer m_frameBuffer; // Used to hold rasterized primitives color buffer, colors in [0.f, 1.f]
		DepthBuffer m_depthBuffer; // Used as depth buffer

		RasterizerSetup m_setup;

		/* Edge function to check whether pixel is covered by triangle or not */
		float PixelCoverage(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c) const;

		/* Set ups a triangle based on index into the index buffer of primitive and triangle topology e.g TRIANGLE_LIST, TRIANGLE_STRIP */
		void SetupTriangle(Primitive* primitive, const int64_t idx, 
			glm::vec3& v0, glm::vec3& v1, glm::vec3& v2) const;

		// Set ups a triangle based on index into the index buffer of primitive and triangle topology e.g TRIANGLE_LIST, TRIANGLE_STRIP
		// also output the actual retrieved index
		void SetupTriangle(Primitive* primitive, const int64_t idx,
			glm::vec3& v0, glm::vec3& v1, glm::vec3& v2,
			int64_t& idx0, int64_t& idx1, int64_t& idx2) const;

		// Apply 2D viewport clipping
		bool Clip2D(const glm::vec2& v0, const glm::vec2& v1, const glm::vec2& v2, Viewport&) const;

		/* Generates and fills in perspectively-correct interpolated barycentric fragment shader attributes to be passed */
		void InterpolateAttributes(const float u, const float v, const float w, const Vertex_OUT& out0, const Vertex_OUT& out1, const Vertex_OUT& out2, Vertex_OUT& attribs) const;

		/* Fetch vertex attributes from primitive's attribute buffer and fill in corresponding vertex attributes */
		void FetchVertexAttributes(const VertexAttributes& attribs, const int64_t idx, Vertex_IN& in0, Vertex_IN& in1, Vertex_IN& in2) const;

		void FetchVertexAttributes(const VertexAttributes& attribs, 
			const int64_t idx0, const int64_t idx1, const int64_t idx2,
			Vertex_IN& in0, Vertex_IN& in1, Vertex_IN& in2) const;
	};
}