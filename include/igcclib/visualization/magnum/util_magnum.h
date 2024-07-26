#pragma once

#include "def_magnum.h"
#include "./../../mesh/TriangularMesh.h"
#include <Magnum/ImageView.h>
#include <Magnum/Array.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Shaders/Flat.h>
namespace _NS_UTILITY
{
	namespace MagnumProc {
		/// <summary>
		/// create magnum ImageView from raw RGB data. The data is wrapped without copying.
		/// </summary>
		/// <param name="data">raw rgb data, layout is [r,g,b,r,g,b...]</param>
		/// <param name="n_pixel">number of pixels</param>
		/// <returns>Magnum ImageView object</returns>
		inline Magnum::ImageView2D imageview_from_data_RGB(uint8_t* data, int height, int width)
		{
			Magnum::Containers::ArrayView<uint8_t> mg_imgdata(data, height * width * 3);
			Magnum::ImageView2D mg_img(Magnum::PixelFormat::RGB8Unorm, { width, height }, mg_imgdata);
			return mg_img;
		}

		/// <summary>
		/// create magnum ImageView from raw RGBA data. The data is wrapped without copying.
		/// </summary>
		/// <param name="data">raw rgb data, layout is [r,g,b,a,r,g,b.a..]</param>
		/// <param name="n_pixel">number of pixels</param>
		/// <returns>Magnum ImageView object</returns>
		inline Magnum::ImageView2D imageview_from_data_RGBA(uint8_t* data, int height, int width)
		{
			Magnum::Containers::ArrayView<uint8_t> mg_imgdata(data, height * width * 4);
			Magnum::ImageView2D mg_img(Magnum::PixelFormat::RGBA8Unorm, { width, height }, mg_imgdata);
			return mg_img;
		}

		/** \brief create a magnum image with specific pixel format */
		inline Magnum::Image2D make_image(int width, int height, Magnum::PixelFormat pixfmt);

		/**
		* \brief convert image to magnum texture. The image will be converted to rgba format.
		*
		* \param output the output magnum texture
		* \param data the image data, in [rgbargba] format
		* \param width image width
		* \param height image height
		* \param flip_ud should we flip the image upside down? Useful when input image's origin is
		at upper left corner, as the magnum texture origin is at lower left corner.
		*/
		inline void to_magnum_texture(
			Magnum::GL::Texture2D& output,
			const uint8_t* data, size_t width, size_t height,
			ImageFormat format,
			bool flip_ud = false);


		/**
		* \brief convert triangular mesh to magnum mesh data, which is useful for creating magnum mesh
		*
		* \param output the output magnum mesh data
		* \param mesh the input triangular mesh
		* \param attribs the attributes to read from mesh, the read attributes are concatenated and interleaved.
		For example, if specified as {POSITION_3, NORMAL_3}, then the output.vertex_data is
		[x,y,z,nx,ny,nz,x,y,z,nx,ny,nz...] where (x,y,z) is position and (nx,ny,nz) is normal.
		* \param invert_texcoord_v set the v in uv coordinate to 1-v, because different system uses
		different uv coordinate system
		*/
		inline void to_magnum_mesh(MeshData& output,
			const TriangularMesh& mesh, const std::vector<VertexAttributeType>& attribs,
			bool invert_texcoord_v = false);

		/** \brief set up the renderer for rendering transparent object */
		inline void set_render_mode_transparency();

		/** \brief set up the renderer for opaque object */
		inline void set_render_mode_opaque();

		//create renderable from triangular mesh
		template<typename SHADER_TYPE>
		void make_renderable(MeshRenderable& output, const TriangularMesh& tmesh, bool use_texture);

		/** \brief create renderable from mesh, if there is texture, the texture will be used automatically.
		The vertex buffer arrangement is [position_3, texcoord_2]*/
		template<> inline void make_renderable<Magnum::Shaders::Flat3D>(
			MeshRenderable& output, const TriangularMesh& tmesh, bool use_texture);
	}
};

namespace _NS_UTILITY {
	namespace MagnumProc {
		inline Magnum::Image2D make_image(int width, int height, Magnum::PixelFormat pixfmt) {
			auto n_byte_per_pixel = Magnum::pixelSize(pixfmt);
			Corrade::Containers::Array<char> _data(n_byte_per_pixel * width * height);
			Magnum::Image2D img{ pixfmt, {width, height}, std::move(_data) };
			return img;
		}

		template<> void make_renderable<Magnum::Shaders::Flat3D>(
			MeshRenderable& output, const TriangularMesh& tmesh, bool use_texture) {

			using namespace Magnum;
			using Shader_t = Shaders::Flat3D;

			//shader operations
			static auto func_modelview_proj = [](ShaderOperations::Shader_t* s,
				const Magnum::Matrix4& modelview, const Magnum::Matrix4& proj) {
				dynamic_cast<Shader_t*>(s)->setTransformationProjectionMatrix(proj * modelview);
			};
			static auto func_set_texture = [](ShaderOperations::Shader_t* s, GL::Texture2D& tex) {
				dynamic_cast<Shader_t*>(s)->bindTexture(tex);
			};

			//create mesh data
			MeshData md;
			to_magnum_mesh(md, tmesh, { VertexAttributeType::POSITION_3, VertexAttributeType::TEXCOORD_2 }, true);

			//create buffer
			output.vertex_buffer = std::make_shared<GL::Buffer>();
			output.index_buffer = std::make_shared<GL::Buffer>();
			output.vertex_buffer->setData(md.vertex_data);
			output.index_buffer->setData(md.index_data);

			//create mesh
			output.mesh = std::make_shared<GL::Mesh>();
			output.mesh->setPrimitive(Magnum::MeshPrimitive::Triangles)
				.addVertexBuffer(*output.vertex_buffer, 0,
					Shader_t::Position{}, Shader_t::TextureCoordinates{})
				.setCount(output.index_buffer->size())
				.setIndexBuffer(*output.index_buffer, 0, Magnum::MeshIndexType::UnsignedInt);

			//create texture and the shader
			if (tmesh.has_texture_image() && use_texture) {
				output.texture = std::make_shared<GL::Texture2D>();

				to_magnum_texture(*output.texture,
					tmesh.get_texture_data_uint8().data(),
					tmesh.get_texture_width(),
					tmesh.get_texture_height(),
					tmesh.get_texture_format());

				//use transparent if texture is present because it has an alpha channel
				output.use_transparency = true;

				//setup shader
				output.shader = std::make_shared<Shader_t>(Shader_t::Flag::Textured);
				output.shader_ops.set_texture = func_set_texture;
			}
			else {
				output.shader = output.shader = std::make_shared<Shader_t>();
			}

			//general shader operations
			output.shader_ops.set_modelview_projection_matrix = func_modelview_proj;	
		}

		void set_render_mode_opaque()
		{			
			using Magnum::GL::Renderer;
			Renderer::disable(Renderer::Feature::Blending);
			Renderer::enable(Renderer::Feature::DepthTest);
		}

		void set_render_mode_transparency()
		{
			using Magnum::GL::Renderer;
			Renderer::enable(Renderer::Feature::Blending);
			Renderer::enable(Renderer::Feature::DepthTest);

			//Renderer::setDepthFunction(Renderer::DepthFunction::Always);
			//Renderer::disable(Renderer::Feature::DepthTest);

			Renderer::setBlendFunction(
				Renderer::BlendFunction::SourceAlpha,
				Renderer::BlendFunction::OneMinusSourceAlpha);
		}

		void to_magnum_mesh(MeshData& output, const TriangularMesh& mesh,
			const std::vector<VertexAttributeType>& attribs,
			bool invert_texcoord_v)
		{
			//check if the attributes are in the mesh
			std::set<VertexAttributeType> arbset(attribs.begin(), attribs.end());
			assert_throw(arbset.size() == attribs.size(), "you have duplicated attributes");

			int nfloat_per_vertex = 0;

			for (auto x : attribs) {
				switch (x) {
				case VertexAttributeType::COLOR_4:
					nfloat_per_vertex += 4;
					break;
				case VertexAttributeType::NORMAL_3:
					assert_throw(mesh.get_normal_faces().size() > 0, "mesh has no normal");
					nfloat_per_vertex += 3;
					break;
				case VertexAttributeType::POSITION_2:
				{
					fVECTOR v;
					mesh.get_vertex(v, 0, false);
					assert_throw(v.size() == 2, "the mesh is not 2-dimensional");
					nfloat_per_vertex += 2;
					break;
				}
				case VertexAttributeType::POSITION_3:
				{
					fVECTOR v;
					mesh.get_vertex(v, 0, false);
					assert_throw(v.size() == 3, "the mesh is not 3-dimensional");
					nfloat_per_vertex += 3;
					break;
				}
				case VertexAttributeType::TEXCOORD_2:
					assert_throw(mesh.get_num_texcoord_vertices() > 0, "the mesh has no texture coordinate");
					nfloat_per_vertex += 2;
					break;
				}
			}

			//initialize buffer
			auto& vdata = output.vertex_data;
			vdata.clear();
			auto& idata = output.index_data;
			idata.clear();
			Magnum::Color4 default_color{ 1,1,1,1 };

			//get mesh data
			const auto& vt = mesh.get_texcoord_vertices();
			const auto& f_vt = mesh.get_texcoord_faces();

			auto vn = mesh.get_normal_vertices();
			const auto& f_vn = mesh.get_normal_faces();

			auto v_geom = mesh.get_vertices();
			const auto& f_geom = mesh.get_faces();

			//record it, break the mesh into single triangles
			vdata.reserve(f_geom.size() * nfloat_per_vertex);
			for (Eigen::Index i = 0; i < f_geom.rows(); i++) {
				for (Eigen::Index k = 0; k < 3; k++) {
					auto idxv_geom = f_geom(i, k);
					for (auto atb : attribs) {
						switch (atb) {
						case VertexAttributeType::COLOR_4:
							vdata.insert(vdata.end(), default_color.data(), default_color.data() + 4);
							break;
						case VertexAttributeType::NORMAL_3:
						{
							auto idxv_normal = f_vn(i, k);
							fVECTOR_3 normal = vn.row(idxv_normal);
							for (int t = 0; t < 3; t++)
								vdata.push_back((Magnum::Float)normal[t]);
							break;
						}
						case VertexAttributeType::POSITION_2:
						case VertexAttributeType::POSITION_3:
						{
							auto v = v_geom.row(idxv_geom);
							for (int t = 0; t < v.size(); t++)
								vdata.push_back((Magnum::Float)v(t));
							break;
						}
						case VertexAttributeType::TEXCOORD_2:
							auto idxv_tex = f_vt(i, k);
							fVECTOR_2 uv = vt.row(idxv_tex);
							vdata.push_back((Magnum::Float)uv[0]);

							auto v = invert_texcoord_v ? 1 - uv[1] : uv[1];
							vdata.push_back((Magnum::Float)v);
							break;
						}
					}
				}
			}

			//create index buffer
			idata.reserve(f_geom.size());
			for (size_t i = 0; i < f_geom.size(); i++)
				idata.push_back((Magnum::UnsignedInt)i);
		}

		void to_magnum_texture(
			Magnum::GL::Texture2D& output,
			const uint8_t* data, size_t width, size_t height,
			ImageFormat format, bool flip_ud
		) {
			using Corrade::Containers::Array;
			using Corrade::Containers::ArrayView;
			using Magnum::ImageView2D;

			assert_throw(format == ImageFormat::RGB || format == ImageFormat::RGBA,
				"image format must be RGB or RGBA");

			std::unique_ptr<Array<char>> imgdata;
			const int n_out_channel = 4;
			auto _data = new Array<char>(width * height * n_out_channel);
			imgdata.reset(_data);

			if (format == ImageFormat::RGBA)
			{
				//input is already rgba, just copy
				memcpy(_data->data(), data, width * height * n_out_channel);
			}
			else {
				//input is rgb, need conversion
				auto x = _data->data();
				for (size_t i = 0; i < width*height; i++) {
					for (int k = 0; k < 3; k++) {
						x[n_out_channel * i + k] = data[3 * i + k];
					}
					for (int k = 3; k < n_out_channel; k++)
						x[n_out_channel * i + k] = 255;
				}
			}

			//flipping
			if (flip_ud) {
				auto data = imgdata->data();
				for(size_t i=0; i<height/2; i++)
					for (size_t j = 0; j < width; j++) {
						auto idx_ij = (i * width + j) * n_out_channel;
						auto idx_flip = ((height - 1 - i)*width + j)*n_out_channel;
						for (int k = 0; k < n_out_channel; k++) {
							std::swap(data[idx_ij + k], data[idx_flip + k]);
						}
					}
			}

			Magnum::Image2D buf_image(
				Magnum::PixelFormat::RGBA8Unorm,
				{ (Magnum::Int)width, (Magnum::Int)height },
				std::move(*imgdata));

			//setSubImage will copy the image into GPU, so don't worry about imgdata being released
			output.setStorage(1, Magnum::GL::TextureFormat::RGBA8,
				{ (Magnum::Int)width, (Magnum::Int)height })
				.setSubImage(0, {}, buf_image);
		}
	}
};