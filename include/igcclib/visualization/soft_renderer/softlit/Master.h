#pragma once

#include <assert.h>
#include <stdint.h>
#include <vector>
#include <float.h>

#define GLM_FORCE_INLINE
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <igcclib/visualization/soft_renderer/softlit/Utils.h>
#include <igcclib/visualization/soft_renderer/softlit/RasterizerConfig.h>

//MODIFY
namespace softlit {

	//shader source channels
	enum class ShaderInputType {
		//vec2 channels
		UV_2,

		//vec3 channels
		VERTEX_COLOR_3,
		VERTEX_NORMAL_3,
	};
	const int NUM_SHADER_INPUT_CHANNEL_2 = 1;
	const int NUM_SHADER_INPUT_CHANNEL_3 = 2;

	//shader output channels
	enum class ShaderOutputChannel {
		FINAL_COLOR,
		UV,
		VERTEX_COLOR,
		WORLD_SPACE_NORMAL,
		VIEW_SPACE_NORMAL,
		WORLD_POSITION,
		VIEW_POSITION,
		BARYCENTRIC		//output (mesh_id, face_index, u, v), where (u,v, 1-u-v) is the barycentric coordinate
	};
	const int NUM_SHADER_OUTPUT_CHANNEL = (int)ShaderOutputChannel::BARYCENTRIC + 1;
	const std::vector<ShaderOutputChannel> ALL_OUTPUT_CHANNELS = {
		ShaderOutputChannel::FINAL_COLOR,
		ShaderOutputChannel::UV,
		ShaderOutputChannel::VERTEX_COLOR,
		ShaderOutputChannel::WORLD_SPACE_NORMAL,
		ShaderOutputChannel::VIEW_SPACE_NORMAL,
		ShaderOutputChannel::WORLD_POSITION,
		ShaderOutputChannel::VIEW_POSITION,
		ShaderOutputChannel::BARYCENTRIC //output is (object_id, face_id, bc1, bc2), where bc3 = 1-bc1-bc2
	};

	template<ShaderInputType> struct DefaultShaderInputChannel {};

	template<> struct DefaultShaderInputChannel<ShaderInputType::UV_2> {
		static const int index = 0;
	};

	template<> struct DefaultShaderInputChannel<ShaderInputType::VERTEX_COLOR_3> {
		static const int index = 0;
	};

	template<> struct DefaultShaderInputChannel<ShaderInputType::VERTEX_NORMAL_3> {
		static const int index = 1;
	};

	//uniform buffer object, which only stores transformation matrices
	struct UBO
	{
		glm::mat4 M{ 1.0 }; //model matrix
		glm::mat4 MV{ 1.0 }; //model view matrix
		glm::mat4 MVP{ 1.0 }; //model view projection matrix
	};

	const int MAX_ATTRIBUTES_VEC4 = 3;
	const int MAX_ATTRIBUTES_VEC3 = 10;
	const int MAX_ATTRIBUTES_VEC2 = 3;

	/*
	vec-N only vertex attributes that can be fed into vertex shaders
	*/
	template<int N>
	struct AttributeBuffer
	{
		std::vector<glm::vec<N, float>> m_data; // Buffer of actual data
		std::vector<uint64_t> m_index; // Buffer of index into data

		const glm::vec<N, float>& operator[](size_t i) const
		{
			return m_data[m_index[i]];
		}
	};

	struct VertexAttributes
	{
		std::vector<AttributeBuffer<4>> attrib_vec4;
		std::vector<AttributeBuffer<3>> attrib_vec3;
		std::vector<AttributeBuffer<2>> attrib_vec2;

		VertexAttributes()
		{
			attrib_vec4.reserve(MAX_ATTRIBUTES_VEC4);
			attrib_vec3.reserve(MAX_ATTRIBUTES_VEC3);
			attrib_vec2.reserve(MAX_ATTRIBUTES_VEC2);
		}
	};

	// Vertex attributes to be passed down to VS/FS after getting fetched & populated
	typedef struct Attribute
	{
		std::vector<glm::vec4> attrib_vec4;
		std::vector<glm::vec3> attrib_vec3;
		std::vector<glm::vec2> attrib_vec2;

		Attribute()
		{
			attrib_vec4.reserve(MAX_ATTRIBUTES_VEC4);
			attrib_vec3.reserve(MAX_ATTRIBUTES_VEC3);
			attrib_vec2.reserve(MAX_ATTRIBUTES_VEC2);
		}

		void ResetData()
		{
			attrib_vec4.clear();
			attrib_vec3.clear();
			attrib_vec2.clear();
		}

		void PushVertexAttribute(const glm::vec4& v)
		{
			attrib_vec4.push_back(v);
		}

		void PushVertexAttribute(const glm::vec3& v)
		{
			attrib_vec3.push_back(v);
		}

		void PushVertexAttribute(const glm::vec2& v)
		{
			attrib_vec2.push_back(v);
		}
	}Vertex_IN, Vertex_OUT;

	inline bool has_attribute(const Attribute& attr, ShaderInputType chn) {
		bool ok = false;
		switch (chn) {
		case ShaderInputType::UV_2:
			ok = attr.attrib_vec2.size() >= DefaultShaderInputChannel<ShaderInputType::UV_2>::index + 1;
			break;
		case ShaderInputType::VERTEX_COLOR_3:
			ok = attr.attrib_vec3.size() >= DefaultShaderInputChannel<ShaderInputType::VERTEX_COLOR_3>::index + 1;
			break;
		case ShaderInputType::VERTEX_NORMAL_3:
			ok = attr.attrib_vec3.size() >= DefaultShaderInputChannel<ShaderInputType::VERTEX_NORMAL_3>::index + 1;
			break;
		}
		return ok;
	}
}