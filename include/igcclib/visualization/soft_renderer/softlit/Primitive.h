#pragma once
#include <memory>
#include <igcclib/visualization/soft_renderer/softlit/Master.h>

namespace softlit
{
	class Texture;
	class SoftVertexShader;
	class SoftFragmentShader;

	typedef std::vector<glm::vec3> VertexBuffer;
	typedef std::vector<uint64_t> IndexBuffer;
	typedef std::vector<Texture*> TextureBuffer;

	// Per-primitive buffer object
	typedef void* UniformBuffer;

	// vertex_shader to be run per-vertex. returned vec4() is vertex converted to clip-coordinates
	//typedef glm::vec4(*vertex_shader)(const glm::vec3& pos, const UniformBuffer ubo, const Vertex_IN* const in, Vertex_OUT* out);

	// fragment_shader to output final render target values
	//typedef glm::vec4(*fragment_shader)(const UniformBuffer ubo, const Vertex_OUT* const in, Texture** tbo);

	using vertex_shader = std::shared_ptr<SoftVertexShader>;
	using fragment_shader = std::shared_ptr<SoftFragmentShader>;


	class Primitive
	{
	public:
		Primitive(const PrimitiveSetup& setup);
		~Primitive();

		const VertexBuffer& getVertexBuffer() const { return m_vertexBuffer; }
		const IndexBuffer& getIndexBuffer() const { return m_indexBuffer; }

		void setVertexBuffer(const VertexBuffer& vb) { m_vertexBuffer = vb; }
		void setIndexBuffer(const IndexBuffer& ib) { m_indexBuffer = ib; }

		vertex_shader getVS() { return m_VS; }
		fragment_shader getFS() { return m_FS; }
		UniformBuffer getUBO() { return m_ubo; }
		Texture** getTBO() { return m_tbo.data(); }

		void setVS(const vertex_shader& vs) { m_VS = vs; }
		void setFS(const fragment_shader& fs) { m_FS = fs; }
		void setUBO(UniformBuffer ubo) { 
			m_ubo = ubo; 
			m_ubo_object.reset(); //release the previous ubo object
		}

		void addTexture(Texture* texture) { m_tbo.push_back(texture); }

		const VertexAttributes& getVertexAttributes() const { return m_attribs; }
		VertexAttributes& getVertexAttributes() { return m_attribs; }

		void appendAttributeBuffer(const AttributeBuffer<4>& attributes)
		{
			m_attribs.attrib_vec4.push_back(attributes);
		}

		void appendAttributeBuffer(const AttributeBuffer<3>& attributes)
		{
			m_attribs.attrib_vec3.push_back(attributes);
		}

		void appendAttributeBuffer(const AttributeBuffer<2>& attributes)
		{
			m_attribs.attrib_vec2.push_back(attributes);
		}

		const PrimitiveSetup& getPrimitiveSetup() const { return m_setup; }

		void set_id(int id) { m_id = id; }
		int get_id() const { return m_id; }

	private:
		VertexBuffer m_vertexBuffer;
		IndexBuffer m_indexBuffer;

		TextureBuffer m_tbo;

		UniformBuffer m_ubo = nullptr;
		vertex_shader m_VS = nullptr;
		fragment_shader m_FS = nullptr;

		VertexAttributes m_attribs;

		PrimitiveSetup m_setup;
		int m_id = -1; //object id

	//MODIFY
	private:
		//MODIFY
		//texture buffer with lifecycle management
		using TextureBufferEx = std::vector<std::shared_ptr<Texture>>;

		//to store the actual ubo object and manage its lifecycle
		std::shared_ptr<UBO> m_ubo_object;

		//texture owned by this object
		TextureBufferEx m_textures;

	public:
		//texture with life cycle management
		void addTexture(const std::shared_ptr<Texture>& texture) {
			m_tbo.push_back(texture.get());
			m_textures.push_back(texture);
		}

		UBO* getUBO_ex() {
			return (UBO*)m_ubo;
		}

		std::shared_ptr<UBO> getUBO_shared() const {
			if (m_ubo_object)
				return m_ubo_object;
			else return {};
		}

		//UBO with life cycle management
		void setUBO(const std::shared_ptr<UBO>& ubo) {
			m_ubo_object = ubo;
			m_ubo = m_ubo_object.get();
		}
	};
}