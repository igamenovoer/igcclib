#pragma once

#include <map>

#include "igcclib_magnum_def.hpp"
#include "SceneNode.hpp"

#include <Magnum/SceneGraph/SceneGraph.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>


namespace _NS_UTILITY {
	namespace MagnumProc {
		/*!
		 * \class MeshNode
		 *
		 * \brief a magnum mesh scene node
		 */
		class MeshNode : public SceneNode, public Magnum::SceneGraph::Drawable3D {
		public:
			using Mesh_t = MeshRenderable::Mesh_t;
			using Shader_t = MeshRenderable::Shader_t;
			using Texture_t = MeshRenderable::Texture_t;

		public:
			explicit MeshNode() :
				SceneNode{},
				Magnum::SceneGraph::Drawable3D(*this, nullptr) {}

			explicit MeshNode(Object3D* parent):
				SceneNode{parent},
				Magnum::SceneGraph::Drawable3D(*this, nullptr) {}


			~MeshNode() {
				printf("mesh node %d is deleted\n", (int)this);
			}

			/** \brief set a mesh with all rendering information, only keep a reference. */
			void set_mesh(const MeshRenderable* mesh_obj);

			/** \brief set the mesh with shared ownership */
			void set_mesh(const std::shared_ptr<MeshRenderable>& mesh_obj);

			MeshRenderable* get_mesh();
			const MeshRenderable* get_mesh() const;

			/** \brief draw the mesh using the specified model view transformation and camera */
			void draw(const Magnum::Matrix4& modelview_mat, Magnum::SceneGraph::Camera3D& camera) override;

			/** \brief draw the mesh using model view projection matrices */
			void draw(const Magnum::Matrix4& modelview_mat, const Magnum::Matrix4& proj_mat);

			/** \brief set visibility of this node. If the node is not visible, draw() call will be skipped */
			void set_visible(bool visible) { m_visible = visible; }
			bool get_visible() const { return m_visible; }

		private:
			//the mesh to be rendered
			std::shared_ptr<MeshRenderable> m_mesh;

			//visibility
			bool m_visible = true;

			//just to take memory for testing
			//int large_data[10000];
		};

		// ======================= implementation =============================
		inline void MeshNode::set_mesh(const MeshRenderable* mesh_obj)
		{
			m_mesh.reset( const_cast<MeshRenderable*>(mesh_obj), [](void*) {});
		}

		inline void MeshNode::set_mesh(const std::shared_ptr<MeshRenderable>& mesh_obj)
		{
			m_mesh = mesh_obj;
		}

		inline MeshRenderable* MeshNode::get_mesh()
		{
			return m_mesh.get();
		}

		inline const MeshRenderable* MeshNode::get_mesh() const
		{
			return m_mesh.get();
		}

		inline void MeshNode::draw(const Magnum::Matrix4& modelview_mat, Magnum::SceneGraph::Camera3D& camera)
		{
			if (!m_visible)
				return;

			draw(modelview_mat, camera.projectionMatrix());
		}

		inline void MeshNode::draw(
			const Magnum::Matrix4& modelview_mat, 
			const Magnum::Matrix4& proj_mat)
		{
			if (!m_visible || !m_mesh || m_mesh->is_empty())
				return;

			using Magnum::GL::Renderer;

			auto& ops = m_mesh->shader_ops;
			if(ops.set_modelview_projection_matrix)
				ops.set_modelview_projection_matrix(m_mesh->shader.get(), modelview_mat, proj_mat);
			if (m_mesh->texture && ops.set_texture)
				ops.set_texture(m_mesh->shader.get(), *m_mesh->texture);

			//set blending mode
			if (m_mesh->use_transparency)
				set_render_mode_transparency();
			else
				set_render_mode_opaque();

			//call pre-render function
			if (m_mesh->pre_render_function)
				m_mesh->pre_render_function(*m_mesh);

			m_mesh->mesh->draw(*m_mesh->shader);

			if (m_mesh->post_render_function)
				m_mesh->post_render_function(*m_mesh);
		}

	}
}