#pragma once

//put this file in one and only one .cpp to get SoftLit compile
#include <igcclib/igcclib_master.hpp>

#include <igcclib/soft_renderer/softlit/Master.h>

//must include these first, otherwise they will conflict with Shaders.h
#include <igcclib/soft_renderer/softlit/Texture.h>

#include <igcclib/soft_renderer/softlit/Display.h>
#include <igcclib/soft_renderer/softlit/Primitive.h>
#include <igcclib/soft_renderer/softlit/Rasterizer.h>
#include <igcclib/soft_renderer/softlit/Shaders.h>
#include <igcclib/soft_renderer/softlit/AnalysisShader.h>
#include <igcclib/soft_renderer/softlit_helpers.h>

namespace _NS_UTILITY {
	void to_softlit_primitive(
		softlit::Primitive& output, const TriangularMesh& _tmesh, bool create_texture)
	{
		namespace slit = softlit;
		bool use_global_coordinate = false;
		TriangularMesh tmesh = _tmesh; //make a copy

		output = slit::Primitive(output.getPrimitiveSetup());

		//create vertices
		{
			slit::VertexBuffer vbo;
			auto v = tmesh.get_vertices(use_global_coordinate);
			auto n_vert = v.rows();
			vbo.reserve(n_vert);
			for (decltype(n_vert) i = 0; i < n_vert; i++) {
				glm::vec3 vertex(v(i, 0), v(i, 1), v(i, 2));
				vbo.push_back(vertex);
			}
			output.setVertexBuffer(vbo);
		}

		//create index
		{
			slit::IndexBuffer ibo;
			const auto& faces = tmesh.get_faces();
			ibo.assign(faces.data(), faces.data() + faces.size());
			output.setIndexBuffer(ibo);
		}

		//create attributes
		auto& v_attribs = output.getVertexAttributes();
		v_attribs.attrib_vec2.resize(slit::NUM_SHADER_INPUT_CHANNEL_2);
		v_attribs.attrib_vec3.resize(slit::NUM_SHADER_INPUT_CHANNEL_3);

		//no uv? make fake uv
		if (tmesh.get_num_texcoord_vertices() == 0) 
		{
			auto n_vertex = tmesh.get_num_vertices();
			fMATRIX uv(n_vertex, 2);
			uv.setOnes();
			tmesh.set_texcoord_vertices(uv);
			tmesh.set_texcoord_faces(tmesh.get_faces());
		}

		//no texture? fake a texture
		if (!tmesh.has_texture_image()) {
			uint8_t imgdata[27];
			memset(imgdata, 255, sizeof(imgdata) * sizeof(uint8_t));
			tmesh.set_texture_image(imgdata, 3, 3, ImageFormat::RGB);
		}

		//no normal? fake normals
		if (tmesh.get_num_normal_vertices() == 0) {
			auto n_vertex = tmesh.get_num_vertices();
			fMATRIX normals(n_vertex, 3);
			normals.setZero();
			normals.col(0).setOnes();
			tmesh.set_normal_faces(tmesh.get_faces());
			tmesh.set_normal_vertices(normals);
		}

		//create uv attribute
		if (tmesh.get_num_texcoord_vertices() > 0) {
			slit::AttributeBuffer<2> uv_buf;
			const auto& uv_vertices = tmesh.get_texcoord_vertices();
			const auto& uv_faces = tmesh.get_texcoord_faces();

			//uv vertices
			uv_buf.m_data.reserve(uv_vertices.rows());
			for (Eigen::Index i = 0; i < uv_vertices.rows(); i++) {
				uv_buf.m_data.push_back(glm::vec2(uv_vertices(i, 0), 1.f-uv_vertices(i, 1)));
			}

			//uv faces
			uv_buf.m_index.assign(uv_faces.data(), uv_faces.data() + uv_faces.size());
			auto idx_attrib = slit::DefaultShaderInputChannel<slit::ShaderInputType::UV_2>::index;
			v_attribs.attrib_vec2[idx_attrib] = uv_buf;

			if (create_texture && tmesh.get_texture_data_uint8().size()>0) {
				//create texture object
				softlit::Image img;
				auto n_channel = get_num_channel(tmesh.get_texture_format());
				assert_throw(n_channel == 3 || n_channel == 4, "only RGB and RGBA textures are supported");
				img.Init(
					tmesh.get_texture_data_uint8().data(),
					tmesh.get_texture_width(),
					tmesh.get_texture_height(),
					n_channel
				);
				auto texture = std::make_shared<softlit::Texture>(img);
				output.addTexture(texture);
			}
		}

		//create vertex color
		{
			slit::AttributeBuffer<3> color_buf;
			const auto& faces = tmesh.get_faces();
			color_buf.m_data.push_back(glm::vec3(1, 1, 1));
			color_buf.m_index = slit::IndexBuffer(output.getIndexBuffer().size(), 0);
			auto idx_attrib = slit::DefaultShaderInputChannel<slit::ShaderInputType::VERTEX_COLOR_3>::index;
			v_attribs.attrib_vec3[idx_attrib] = color_buf;
		}

		//create normal
		if (tmesh.get_num_normal_vertices() > 0) {
			slit::AttributeBuffer<3> normal_buf;
			auto normal_vertices = tmesh.get_normal_vertices(use_global_coordinate);
			const auto& normal_faces = tmesh.get_normal_faces();

			normal_buf.m_data.reserve(normal_vertices.rows());
			for (Eigen::Index i = 0; i < normal_vertices.rows(); i++) {
				normal_buf.m_data.push_back(
					glm::vec3(normal_vertices(i, 0), normal_vertices(i, 1), normal_vertices(i, 2))
				);
			}
			normal_buf.m_index.assign(normal_faces.data(), normal_faces.data() + normal_faces.size());

			auto idx_attrib = slit::DefaultShaderInputChannel<slit::ShaderInputType::VERTEX_NORMAL_3>::index;
			v_attribs.attrib_vec3[idx_attrib] = normal_buf;
		}

		//transform
		if (!use_global_coordinate) //if the vertices and normals are local, then we must set the transformation matrix
		{
			auto ubo = std::make_shared<slit::UBO>();
			output.setUBO(ubo);

			fMATRIX_4 tmat = tmesh.get_transmat().transpose();
			to_matrix(tmat, ubo->M);
			ubo->MV = ubo->M;
			ubo->MVP = ubo->MV;
		}

		auto vs = std::make_shared<slit::AnalysisVertexShader>();
		*vs = slit::AnalysisVertexShader::get_default_shader();
		output.setVS(std::dynamic_pointer_cast<slit::SoftVertexShader>(vs));

		auto fs = std::make_shared<slit::AnalysisFragmentShader>();
		*fs = slit::AnalysisFragmentShader::get_default_shader();
		output.setFS(std::dynamic_pointer_cast<slit::SoftFragmentShader>(fs));

		////set shader
		//if (tmesh.get_num_texcoord_vertices() > 0 && tmesh.get_texture_data_uint8().size()>0)
		//{
		//	output.setVS((slit::vertex_shader)&slit::VS_Textured);
		//	output.setFS((slit::fragment_shader)&slit::FS_Textured);
		//}
		//else {
		//	output.setVS((slit::vertex_shader)&slit::VS_Simple);
		//	output.setFS((slit::fragment_shader)&slit::FS_Simple);
		//}
	}
}