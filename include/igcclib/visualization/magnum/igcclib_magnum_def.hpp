#pragma once
#include <igcclib/igcclib_master.hpp>
#include <igcclib/core/igcclib_eigen_def.hpp>

#include <Magnum/Magnum.h>
#include <Magnum/Mesh.h>
#include <Magnum/Array.h>
#include <Magnum/Image.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Math/Matrix.h>

#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/GL/BufferImage.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Platform/GLContext.h>

#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>

namespace _NS_UTILITY
{
	namespace MagnumProc {
		using Scene3D = Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D>;
		using Object3D = Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D>;

		/// <summary>
		/// vertex structure with Position
		/// </summary>
		struct VertexData_P_3
		{
			Magnum::Vector3 position;
			VertexData_P_3() :position{ 0,0,0 } {}
		};

		/// <summary>
		/// vertex structure with Position and Texture
		/// </summary>
		struct VertexData_PT_3
		{
			Magnum::Vector3 position;
			Magnum::Vector2 texcoord;
			VertexData_PT_3() :position{ 0,0,0 }, texcoord{ 0,0 } {}
		};

		/// <summary>
		/// vertex structure with Position, Texture and Normal
		/// </summary>
		struct VertexData_PTN_3
		{
			Magnum::Vector3 position;
			Magnum::Vector2 texcoord;
			Magnum::Vector3 normal;
			VertexData_PTN_3() : position{ 0,0,0 }, texcoord{ 0,0 }, normal{ 0,0,1 } {}
		};

		/// <summary>
		/// vertex structure with Position and Normal
		/// </summary>
		struct VertexData_PN_3
		{
			Magnum::Vector3 position;
			Magnum::Vector3 normal;
			VertexData_PN_3() : position{ 0,0,0 }, normal{ 0,0,1 } {}
		};

		struct VertexData_PTNC_3
		{
			Magnum::Vector3 position;
			Magnum::Vector2 texcoord;
			Magnum::Vector3 normal;
			Magnum::Vector4 color;

			VertexData_PTNC_3() :
				position{ 0,0,0 }, texcoord{ 0,0 },
				normal{ 0,0,1 }, color{ 1,1,1,1 }{};
		};

		enum class VertexAttributeType {
			POSITION_2, POSITION_3, TEXCOORD_2, NORMAL_3, COLOR_4
		};

		//shader with common methods
		struct ShaderOperations {
			using Shader_t = Magnum::GL::AbstractShaderProgram;
			std::function<void(Shader_t*, Magnum::GL::Texture2D&)> 
				set_texture;
			std::function<void(Shader_t*, const Magnum::Matrix4& modelview, const Magnum::Matrix4& proj)>
				set_modelview_projection_matrix;
		};

		/** \brief data used to create magnum mesh */
		struct MeshData {
			std::vector<Magnum::Float> vertex_data;
			std::vector<Magnum::UnsignedInt> index_data;
		};

		enum class CoordinateSystem {
			WORLD, PARENT, SELF
		};

		/** \brief magnum mesh with gpu buffer */
		struct MeshRenderable {
			using Mesh_t = Magnum::GL::Mesh;
			using Shader_t = Magnum::GL::AbstractShaderProgram;
			using Texture_t = Magnum::GL::Texture2D;
			using Buffer_t = Magnum::GL::Buffer;

			std::shared_ptr<Mesh_t> mesh;
			std::shared_ptr<Buffer_t> vertex_buffer;
			std::shared_ptr<Buffer_t> index_buffer;
			std::shared_ptr<Shader_t> shader;
			std::shared_ptr<Texture_t> texture;
			ShaderOperations shader_ops;

			//this mesh has transparent texture or color?
			bool use_transparency = false;	

			//a function to be called before and after rendering
			//for the user to adjust the opengl state to suite his needs
			std::function<void(MeshRenderable&)> pre_render_function;
			std::function<void(MeshRenderable&)> post_render_function;

			bool is_empty() const {
				return !mesh;
			}
		};

		//camera type
		enum class CameraType {
			ORTHOGRAPHIC, PERSPECTIVE
		};
	}

};