#pragma once

#include <map>
#include <sstream>
#include <thread>

#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/Shaders/Flat.h>
#include <GLFW/glfw3.h>

#include "igcclib_magnum.hpp"
#include "MeshNode.hpp"
#include "OrthographicCamera.hpp"




namespace _NS_UTILITY {
	namespace MagnumProc {
		
		/** \brief renderer using magnum as a backend */
		class MagnumRenderer {
		public:
			struct ResourceRenderOptions {
				using PolygonMode_t = Magnum::GL::Renderer::PolygonMode;

				bool use_texture = true; //if texture is available, use texture
				Magnum::Color4 color{ 1,1,1,1 }; //if texture is not used, render it with this color
				PolygonMode_t polygon_mode = PolygonMode_t::Fill; //the polygon mode for rendering this object
				Magnum::Float line_width = 1.0f; //when rendering as lines, this is the line width
				Magnum::Float point_size = 1.0f; //when rendering as points, this is the point size				
			};

			/** \brief the data used to construct resource */
			struct SourceData {
				TriangularMesh mesh;
				std::string key;
				ResourceRenderOptions render_options;
			};

			/** \brief the data used for rendering */
			struct EngineData {
				MeshNode* node = nullptr; //memory managed by scene
			};

			struct Resource {
				SourceData source_data;
				EngineData engine_data;

				bool is_visible() const {
					return is_in_engine() && engine_data.node->get_visible();
				}

				void set_visible(bool visible) {
					if (!is_in_engine()) return;
					engine_data.node->set_visible(visible);
				}

				bool is_in_engine() const {
					return engine_data.node;
				}
			};

			//the parameters for initializing engine
			struct InitParameters{
				size_t image_width = 800;
				size_t image_height = 600;
				CameraBase* camera = nullptr; //reference to a camera
			}; 
			using SizeWH = std::pair<size_t, size_t>;

		public:
			using Camera_t = Magnum::SceneGraph::Camera3D;
			~MagnumRenderer() { close_engine(); }

		public:
			/** \brief get all resources */
			std::map<std::string, Resource*> get_all_resources();

			/** \brief get all resources */
			std::map<std::string, const Resource*> get_all_resources() const;

			/** \brief initialize the engine */
			void init_engine(const InitParameters& params = {});

			/** \brief close the engine and unload all resources */
			void close_engine();

			/** \brief add or set resource, using res.source_data.key as key */
			Resource* set_resource(const Resource& res);

			/** \brief set the visibility of a resource.
			\deprecated should use get_resource() and Resource::set_visible() instead
			*/
			void set_resource_visible(const std::string& key, bool vis);

			/** \brief get the visibility of a resource.
			\deprecated should use get_resource() and Resource::get_visible() instead
			*/
			bool get_resource_visible(const std::string& key) const;

			/** \brief get a resource, return NULL if the resource does not exist */
			Resource* get_resource(const std::string& key);

			/** \brief set the visibility for all resources */
			void set_all_resources_visible(bool vis);

			/** \brief set the render image size */
			void set_image_size(size_t width, size_t height);

			size_t get_image_width() const;
			size_t get_image_height() const;

			/** \brief get the camera */
			CameraBase* get_camera();
			void set_camera(CameraBase* camera);

			/** \brief render the content into an image, the image must have size enough to hold width*height*4 numbers */
			void render_image_RGBA(uint8_t* imgdata);

			/** \brief render and refresh the window */
			void render(bool swap_buffer = true);

			static size_t get_default_image_width() { return 800; }
			static size_t get_default_image_height() { return 600; }

			/** \brief show the render window? */
			void set_window_visible(bool tf);
			bool get_window_visible() const;

			void set_background_color(const Magnum::Color4& color);
			const Magnum::Color4& get_background_color() const;

			Scene3D* get_scene() { return m_scene.get(); }
			SceneNode* get_root_node() { return m_root_node; }

			//the glfw window
			GLFWwindow* get_window();

		private:
			void create_engine_resource(Resource& res);
			void release_engine_resource(Resource& res);

		private:
			//glfw stuff
			GLFWwindow* m_window = nullptr;

			//opengl context
			std::shared_ptr<Magnum::Platform::GLContext> m_gl_context;

			//image size, must be consistent with get_default_image_width() and ...height()
			SizeWH m_image_size = { 800,600 };

			//camera and its node
			CameraBase* m_camera = nullptr;
			Magnum::SceneGraph::DrawableGroup3D m_draw_group;

			//the scene
			std::shared_ptr<Scene3D> m_scene;

			//the root node for all resources
			SceneNode* m_root_node = nullptr;

			//background color
			Magnum::Color4 m_background_color{ 0,0,0,1 };

			//resource storage
			std::map<std::string, std::shared_ptr<Resource>> m_resources;
		};

		//========================= implementation ===========================
		inline std::map<std::string, MagnumRenderer::Resource*> 
			MagnumRenderer::get_all_resources()
		{
			std::map<std::string, Resource*> output;
			for (auto x : m_resources)
				output[x.first] = x.second.get();
			return output;
		}

		inline std::map<std::string, const MagnumRenderer::Resource*> 
			MagnumRenderer::get_all_resources() const
		{
			std::map<std::string, const Resource*> output;
			for (auto x : m_resources)
				output[x.first] = x.second.get();
			return output;
		}

		inline void MagnumRenderer::init_engine(const MagnumRenderer::InitParameters& params)
		{
			if (!glfwInit())
				assert_throw(false, "failed to initialize glfw");

			//create glfw window
			glfwWindowHint(GLFW_DOUBLEBUFFER, true);
			glfwWindowHint(GLFW_ALPHA_BITS, true);
			glfwWindowHint(GLFW_VISIBLE, false);
			
			//generate a random number as its name
			std::string win_name;
			{
				std::ostringstream os;
				os << "magnum renderer: " << std::this_thread::get_id();
				win_name = os.str();
			}

			//create a dummy window
			m_window = glfwCreateWindow(1,1,win_name.c_str(), NULL, NULL);

			//hide the window
			glfwHideWindow(m_window);

			//make it current
			glfwMakeContextCurrent(m_window);

			//create opengl context
			m_gl_context = std::make_shared<Magnum::Platform::GLContext>();

			//resize the window to specified size
			set_image_size(params.image_width, params.image_height);

			//create scene
			m_scene = std::make_shared<Scene3D>();

			//setup camera
			assert_throw(params.camera, "camera is not set");
			set_camera(params.camera);

			//create root node for all resources
			m_root_node = new SceneNode();
			m_root_node->setParent(m_scene.get());
		}

		inline void MagnumRenderer::close_engine()
		{
			set_camera(nullptr);
			m_resources.clear();
			m_scene.reset();
			m_gl_context.reset();
			m_root_node = nullptr;
			glfwTerminate();
		}

		inline MagnumRenderer::Resource* MagnumRenderer::set_resource(const Resource& res)
		{
			auto key = res.source_data.key;

			//do we have an old resource with the same key?
			if (m_resources.find(key) != m_resources.end()) {
				//release old resource
				release_engine_resource(*m_resources[key]);
				m_resources.erase(key);
			}

			auto obj = std::make_shared<Resource>();
			obj->source_data = res.source_data;

			//create renderables
			create_engine_resource(*obj);

			//save this resource
			m_resources[key] = obj;

			return obj.get();
		}

		inline void MagnumRenderer::set_resource_visible(const std::string& key, bool vis)
		{
			m_resources[key]->set_visible(vis);
		}

		inline bool MagnumRenderer::get_resource_visible(const std::string& key) const
		{
			return m_resources.at(key)->is_visible();
		}

		inline MagnumRenderer::Resource* MagnumRenderer::get_resource(const std::string& key)
		{
			return m_resources[key].get();
		}

		inline void MagnumRenderer::set_all_resources_visible(bool vis)
		{
			for (auto& x : m_resources) {
				x.second->set_visible(vis);
			}
		}

		inline void MagnumRenderer::set_image_size(size_t width, size_t height)
		{
			glfwSetWindowSize(m_window, (int)width, (int)height);
			Magnum::GL::defaultFramebuffer.setViewport({ {0,0},{(int)width,(int)height} });
			m_image_size = { width, height };
		}

		inline size_t MagnumRenderer::get_image_width() const
		{
			return m_image_size.first;
		}

		inline size_t MagnumRenderer::get_image_height() const
		{
			return m_image_size.second;
		}

		inline CameraBase* MagnumRenderer::get_camera()
		{
			return m_camera;
		}

		inline void MagnumRenderer::set_camera(CameraBase* camera) {
			if (m_camera)
				m_camera->setParent(nullptr);

			m_camera = camera;
			if (m_camera)
				m_camera->setParent(m_scene.get());
		}

		inline void MagnumRenderer::render_image_RGBA(uint8_t* imgdata)
		{
			//flip the image upside down, because the first pixel is assumed
			//to be at upper left, but the Magnum uses lower left as origin
			auto projmat = m_camera->projectionMatrix();
			auto flip_projmat = Magnum::Matrix4::scaling({ 1.f,-1.f,1.f }) * projmat;
			m_camera->setProjectionMatrix(flip_projmat);
			render(false);
			m_camera->setProjectionMatrix(projmat);

			auto width = m_image_size.first;
			auto height = m_image_size.second;

			//Corrade::Containers::ArrayView<uint8_t> _imgdata(imgdata, width*height * 4);
			//Magnum::ImageView2D imgoutput{Magnum::PixelFormat::RGBA8Unorm, {(int)width, (int)height} };
			//imgoutput.setData(_imgdata);

			Corrade::Containers::Array<char> _imgdata(width * height * 4);
			Magnum::Image2D outimg{ Magnum::PixelFormat::RGBA8Unorm, 
			{(int)width, (int)height}, std::move(_imgdata) };

			Magnum::GL::defaultFramebuffer.read(
				Magnum::GL::defaultFramebuffer.viewport(), outimg
			);

			memcpy(imgdata, outimg.data().data(), width * height * 4);
		}

		inline void MagnumRenderer::render(bool swap_buffer)
		{
			//clear color
			auto& frame_buffer = Magnum::GL::defaultFramebuffer;
			frame_buffer.clear(Magnum::GL::FramebufferClear::Color | Magnum::GL::FramebufferClear::Depth);
			frame_buffer.clearColor(m_background_color);
			
			//draw stuff
			m_camera->draw(m_draw_group);

			//handling window stuff
			if(swap_buffer)
				glfwSwapBuffers(m_window);
			glfwPollEvents();
		}

		inline void MagnumRenderer::set_window_visible(bool tf)
		{
			if (tf)
				glfwShowWindow(m_window);
			else
				glfwHideWindow(m_window);
		}

		inline bool MagnumRenderer::get_window_visible() const
		{
			return glfwGetWindowAttrib(m_window, GLFW_VISIBLE);
		}

		inline void MagnumRenderer::set_background_color(const Magnum::Color4& color)
		{
			m_background_color = color;
		}

		const Magnum::Color4& MagnumRenderer::get_background_color() const
		{
			return m_background_color;
		}

		inline GLFWwindow* MagnumRenderer::get_window()
		{
			return m_window;
		}

		inline void MagnumRenderer::create_engine_resource(Resource& res)
		{
			//instantiate all the engine resources

			//create renderable
			auto obj_render = std::make_shared<MeshRenderable>();
			make_renderable<Magnum::Shaders::Flat3D>(*obj_render, res.source_data.mesh, 
				res.source_data.render_options.use_texture);
			auto shader = dynamic_cast<Magnum::Shaders::Flat3D*>(obj_render->shader.get());
			shader->setColor(res.source_data.render_options.color);

			//set up polygon mode function
			static auto func_pre_render = [](MeshRenderable& obj, const ResourceRenderOptions* opt) {
				Magnum::GL::Renderer::setPolygonMode(opt->polygon_mode);
				if(opt->polygon_mode == ResourceRenderOptions::PolygonMode_t::Point)
					Magnum::GL::Renderer::setPointSize(opt->point_size);
				if (opt->polygon_mode == ResourceRenderOptions::PolygonMode_t::Line)
					Magnum::GL::Renderer::setLineWidth(opt->line_width);
			};

			using namespace std::placeholders;
			obj_render->pre_render_function = std::bind(func_pre_render, _1, &res.source_data.render_options);

			//create mesh node
			auto& eng = res.engine_data;
			eng.node = new MeshNode;
			eng.node->setParent(m_root_node); //now memory managment is transferred to scene
			eng.node->set_mesh(obj_render);

			//add to render group
			m_draw_group.add(*eng.node);
		}

		inline void MagnumRenderer::release_engine_resource(Resource& res)
		{
			//destroy the node
			if (!res.is_in_engine()) return;

			//detach from parent
			res.engine_data.node->setParent(nullptr);
			m_draw_group.remove(*res.engine_data.node);

			//destroy itself
			delete res.engine_data.node;
			res.engine_data.node = nullptr;
		}

	}
}