#define NOMINMAX //prevent windows.h from defining min max

#include <Magnum/Magnum.h>
#include <Magnum/Math/Math.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Shaders/MeshVisualizer.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Platform/Platform.h>
#include <Magnum/Platform/GLContext.h>
#include <Magnum/Platform/GlfwApplication.h>
#include <Magnum/Image.h>
#include <Corrade/Containers/Array.h>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>

#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <map>
#include <numeric>

#include <igcclib/geometry/TriangularMesh.hpp>
#include <igcclib/geometry/igcclib_obj_eigen.hpp>
#include <igcclib/geometry/igcclib_mesh_helpers.hpp>
#include "igcclib_magnum.hpp"
#include "igcclib_magnum_engine.hpp"

namespace _NS_UTILITY {

	namespace MagnumProc {
		/// <summary>
		/// A mesh viewer that can render a textured mesh.
		/// The coordinate system is x+ right, y+ up, z+ outside, and the camera always looks along -z direction.
		/// To view a mesh, you need to first set the mesh into the viewer, and make sure its front side facing the z+
		/// </summary>
		class MeshViewer :public Magnum::Platform::GlfwApplication
		{
		public:
			//mesh and its related rendering info
			struct MeshInfo
			{
				std::string name;
				const TriangularMesh* mesh_obj;

				//Magnum::Shaders::Flat3D gl_shader_no_tex;
				//Magnum::Shaders::Flat3D gl_shader_with_tex;

				using Shader_t = Magnum::Shaders::Phong;
				Magnum::Shaders::Phong gl_shader_no_tex;
				Magnum::Shaders::Phong gl_shader_with_tex;

				//rendering settings
				bool use_lighting = false;
				Magnum::Color4 ambient_color{ 1,1,1,1 };
				Magnum::Color4 diffuse_color{ 1,1,1,1 };

				//opengl info
				Magnum::GL::Mesh gl_mesh;
				Magnum::GL::Buffer gl_vertex;
				Magnum::GL::Buffer gl_index;
				Magnum::GL::Texture2D gl_texture;
				Magnum::GL::Renderer::PolygonMode gl_polygon_mode = Magnum::GL::Renderer::PolygonMode::Fill;
				double gl_point_size = 1.0; //use when drawing polygon mode as POINT

				//rendering options
				bool use_texture;

				//texture in main memory, it is not copyable so we need to store it as pointer
				std::shared_ptr<Magnum::Image2D> texture_image;

				//transformation of this mesh
				Magnum::Matrix4 transmat;

				MeshInfo() :gl_shader_with_tex{ Shader_t::Flag::AmbientTexture | Shader_t::Flag::DiffuseTexture }, mesh_obj(0), use_texture(false) {
					transmat = Magnum::Matrix4::translation({ 0,0,0 });
					set_lighting_enable(false);
				}

				//set the lighting position, it is in local coordinate of the mesh
				void set_light_position(double x, double y, double z) {
					Magnum::Vector3 pos{ (float)x,(float)y,(float)z };
					gl_shader_no_tex.setLightPosition(pos);
					gl_shader_with_tex.setLightPosition(pos);
				}

				void set_light_color(double r, double g, double b, double a = 1.0) {
					Magnum::Color4 color{ (float)r,(float)g,(float)b,(float)a };
					gl_shader_no_tex.setLightColor(color);
					gl_shader_with_tex.setLightColor(color);
				}

				void set_lighting_enable(bool tf) {
					use_lighting = tf;
					update_material();
				}

				void set_ambient_color(double r, double g, double b, double a = 1.0) {
					Magnum::Color4 color{ (float)r,(float)g,(float)b,(float)a };
					ambient_color = color;
					update_material();
				}

				void set_diffuse_color(double r, double g, double b, double a = 1.0) {
					Magnum::Color4 color{ (float)r,(float)g,(float)b,(float)a };
					diffuse_color = color;
					update_material();
				}

				void update_material() {
					if (use_lighting) {
						gl_shader_no_tex.setAmbientColor(ambient_color);
						gl_shader_no_tex.setDiffuseColor(diffuse_color);
						gl_shader_no_tex.setSpecularColor({ 0,0,0,0 });

						gl_shader_with_tex.setAmbientColor(ambient_color);
						gl_shader_with_tex.setDiffuseColor({ 1,1,1,1 });
						gl_shader_with_tex.setSpecularColor({ 0,0,0,0 });
					}
					else {
						gl_shader_no_tex.setAmbientColor(ambient_color);
						gl_shader_no_tex.setDiffuseColor({ 0,0,0,0 });
						gl_shader_no_tex.setSpecularColor({ 0,0,0,0 });

						gl_shader_with_tex.setAmbientColor({ 1,1,1,1 });
						gl_shader_with_tex.setDiffuseColor({ 0,0,0,1 });
						gl_shader_with_tex.setSpecularColor({ 0,0,0,1 });
					}
				}

				void set_color(double r, double g, double b, double a = 1.0) {
					diffuse_color = Magnum::Color4{ (float)r,(float)g,(float)b,(float)a };
					ambient_color = Magnum::Color4{ (float)r,(float)g,(float)b,(float)a };
					update_material();
				}

				void set_polygon_mode(Magnum::GL::Renderer::PolygonMode mode) {
					gl_polygon_mode = mode;
				}

				void set_point_size(double pt) {
					gl_point_size = pt;
				}
			};
			using MeshInfoPtr = std::shared_ptr<MeshInfo>;

		public:
			static std::shared_ptr<MeshViewer> init() {
				int argc = 0;
				Configuration configs;
				configs.setWindowFlags(Configuration::WindowFlag::Hidden | Configuration::WindowFlag::Resizable);

				std::shared_ptr<MeshViewer> app(new MeshViewer({ argc, 0 }, configs));
				return app;
			}

			explicit MeshViewer(const Arguments& args) :
				GlfwApplication(args) {
				_init();
			}

			explicit MeshViewer(const Arguments& args, const Configuration& configs) :
				GlfwApplication(args, configs)
			{
				_init();
			}

			explicit MeshViewer(const Arguments& args, Magnum::NoCreateT) :
				GlfwApplication(args, Magnum::NoCreate)
			{
				_init();
			}

			void set_visible(bool visible)
			{
				if (visible)
					glfwShowWindow(window());
				else
					glfwHideWindow(window());
			}

			bool is_visible() {
				return glfwGetWindowAttrib(window(), GLFW_VISIBLE) == GLFW_TRUE;
			}

			//draw without swapping buffers
			void draw_only();

			void drawEvent() override;
			//void viewportEvent(const Magnum::Vector2i& size) override;
			void viewportEvent(ViewportEvent& evt) override;
			void _init();

			/**
			* \brief add a mesh into the scene
			*
			* \param name the unique name of this mesh
			* \param mesh the mesh to be added
			* \param use_texture whether to use the texture in the mesh. If true and the mesh does have texture,
			that texture will be used. Otherwise, ignore the texture and pure color is used.
			* \return MeshViewer::MeshInfo* the created mesh entry
			*/
			MeshInfo* add_mesh(std::string name, const TriangularMesh& mesh, bool use_texture = true);

			/**
			* \brief replace a mesh
			*
			* \param name name of the mesh to be replaced
			* \param mesh the mesh
			* \param use_texture whether to use the texture in the mesh. If true and the mesh does have texture,
			that texture will be used. Otherwise, ignore the texture and pure color is used.
			* \return MeshViewer::MeshInfo* the created mesh entry
			*/
			MeshInfo* set_mesh(std::string name, const TriangularMesh& mesh, bool use_texture = true);

			/// <summary>
			/// remove a mesh
			/// </summary>
			/// <param name="name">the name of the mesh</param>
			void remove_mesh(std::string name);

			/// <summary>
			/// remove all meshes
			/// </summary>
			void clear_meshes();

			/** \brief reset the renderer into default state */
			void reset();

			/// <summary>
			/// set the window size
			/// </summary>
			/// <param name="width">width in pixel</param>
			/// <param name="height">height in pixel</param>


			/**
			* \brief set the window size. One of the width and height can be 0, where it will be automatically computed
			to preserve the aspect ratio of the view rect.
			*
			* \param width width in pixel
			* \param height height in pixel
			*/
			void set_window_size(int width, int height);

			/// <summary>
			/// set the window size so that its aspect ratio is the same as view range
			/// </summary>
			/// <param name="width">the width of the window</param>
			void set_window_size(int width);

			/// <summary>
			/// set the view range in world coordinate
			/// </summary>
			/// <param name="x">minimum x</param>
			/// <param name="y">minimum y</param>
			/// <param name="width">width of the view</param>
			/// <param name="height">height of the view</param>
			void set_view_range(double x, double y, double width, double height);

			/// <summary>
			/// automatically configure the view range so that it exactly contains the meshes
			/// </summary>
			void set_view_range_tight();

			fVECTOR get_view_range();

			/// <summary>
			/// render the current image, and output in rgba format
			/// </summary>
			/// <param name="rgba">the pixel storage into which the result is written, in [rgbargba...] format, note that is must contain enough space to write, that is, number of pixels x4</param>
			void render_image_RGBA(char* rgba);

			/// <summary>
			/// render the current image in RGB format
			/// </summary>
			/// <param name="rgb">the data to be writte to in [rgbrgb...] format, must contain enough pixel</param>
			void render_image_RGB(char* rgb);

			/// <summary>
			/// render the current image, and output RGBA cv mat image
			/// </summary>
			/// <param name="output">the cv::mat for output</param>
			void render_image_RGBA(cv::Mat& output);

			/// <summary>
			/// render the current image, and output RGB cv mat image
			/// </summary>
			/// <param name="output">the cv::mat for output</param>
			void render_image_RGB(cv::Mat& output);

			/// <summary>
			/// get the mesh and its associated information, you can modify the information as needed
			/// </summary>
			/// <param name="name"></param>
			/// <returns></returns>
			MeshInfo* get_mesh_info(std::string name);

			/// <summary>
			/// set the background color
			/// </summary>
			void set_background_color(double r, double g, double b, double a = 1.0);

		private:
			//update some book-keeping states based on mesh
			void _update_mesh_state();

			//orthographic viewport in (x,y,width,height), in world coordinate
			fVECTOR m_view_range;

			//target window size in (width, height) format
			iVECTOR m_winsize_wh;

			//a list of meshes to be shown in the scene
			std::map<std::string, MeshInfoPtr> m_name2mesh;

			//camera transformation that will be applied to all objects after their individual transformations
			Magnum::Matrix4 m_tmat_camera;

			//transform the image after camera transformation, useful for rendering the scene into an image,
			//you can use this to flip the rendered image upside down,
			//because the framebuffer layout is pixel (0,0) at lower left corner
			//which is different from most image pixel layout where (0,0) is the upper left corner
			Magnum::Matrix4 m_tmat_after_camera;

			//transform all meshes
			Magnum::Matrix4 m_tmat_global;

			//background color
			Magnum::Color4 m_bg_color{ 0.5,0.5,0.5,0 };
		};

		// ============== implementations =================
		inline void MeshViewer::draw_only()
		{
			using namespace Magnum;
			GL::defaultFramebuffer.clear(GL::FramebufferClear::Depth | GL::FramebufferClear::Color);
			GL::defaultFramebuffer.clearColor(m_bg_color);

			//draw all meshes
			for (auto x : m_name2mesh)
			{
				auto info = x.second;

				auto* shader = &info->gl_shader_no_tex;
				if (info->use_texture)
				{
					shader = &info->gl_shader_with_tex;

					//shader.get().bindTexture(info->gl_texture);
					//Matrix4 mat = m_tmat_after_camera * m_tmat_camera * m_tmat_global * info->transmat;
					//shader.get().setTransformationProjectionMatrix(mat);

					shader->bindDiffuseTexture(info->gl_texture);
					shader->bindAmbientTexture(info->gl_texture);
				}

				Matrix4 tmat = m_tmat_global * info->transmat;
				shader->setTransformationMatrix(tmat);
				shader->setNormalMatrix(tmat.rotationScaling());
				shader->setProjectionMatrix(m_tmat_after_camera * m_tmat_camera);

				GL::Renderer::setPolygonMode(info->gl_polygon_mode);
				GL::Renderer::setPointSize({ (float)info->gl_point_size });
				info->gl_mesh.draw(*shader);
			}
		}

		inline void MeshViewer::drawEvent()
		{
			draw_only();
			swapBuffers();
		}

		//inline void MeshViewer::viewportEvent(const Magnum::Vector2i& size)
		//{
		//	Magnum::GL::defaultFramebuffer.setViewport({ { 0,0 },{ size[0],size[1] } });
		//}

		inline void MeshViewer::viewportEvent(ViewportEvent& evt) {
			Magnum::GL::defaultFramebuffer.setViewport({ {0,0}, evt.windowSize() });
		}

		inline void MeshViewer::_init()
		{
			auto identity_mat = Magnum::Matrix4::translation({ 0,0,0 });
			m_tmat_camera = identity_mat;
			m_tmat_global = identity_mat;
			m_tmat_after_camera = identity_mat;

			m_view_range = fVECTOR(4);
			set_view_range(-1, -1, 2, 2);

			m_winsize_wh = iVECTOR(2);
			set_window_size(800, 800);

			m_bg_color = { 0.5,0.5,0.5,0 };

			Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::DepthTest);
		}

		inline MeshViewer::MeshInfo* MeshViewer::add_mesh(
			std::string name, const TriangularMesh& mesh, bool use_texture)
		{
			assert_throw(m_name2mesh.find(name) == m_name2mesh.end(), "a mesh with the same name already exists");
			return set_mesh(name, mesh, use_texture);
		}

		inline MeshViewer::MeshInfo* MeshViewer::set_mesh(
			std::string name, const TriangularMesh& mesh, bool use_texture)
		{
			using namespace MagnumProc;

			//create rendering information
			auto info = std::make_shared<MeshInfo>();
			m_name2mesh[name] = info;

			info->mesh_obj = &mesh;
			info->name = name;
			info->use_texture = false;

			using Shader_T = decltype(info->gl_shader_with_tex);

			if (mesh.get_texcoord_vertices().size() > 0 && use_texture) //have texture coordinate
			{
				if (mesh.get_normal_faces().size() > 0) {
					//has normals
					std::vector<VertexData_PTN_3> vdlist;
					to_vertex_data(mesh.get_vertices(), mesh.get_faces(),
						mesh.get_texcoord_vertices(), mesh.get_texcoord_faces(),
						mesh.get_normal_vertices(false), mesh.get_normal_faces(),
						vdlist);
					info->gl_vertex.setData(vdlist);
					info->gl_mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
						.setCount(vdlist.size())
						.addVertexBuffer(info->gl_vertex, 0,
							Shader_T::Position{},
							Shader_T::TextureCoordinates{},
							Shader_T::Normal{});
				}
				else {
					//no normals
					std::vector<VertexData_PT_3> vdlist;
					to_vertex_data(mesh.get_vertices(), mesh.get_faces(),
						mesh.get_texcoord_vertices(), mesh.get_texcoord_faces(), vdlist);
					info->gl_vertex.setData(vdlist);
					info->gl_mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
						.setCount(vdlist.size())
						.addVertexBuffer(info->gl_vertex, 0, Shader_T::Position{}, Shader_T::TextureCoordinates{});
				}

				//have texture image?
				if (mesh.has_texture_image())
				{
					info->use_texture = true;
					const auto& _texdata = mesh.get_texture_data_uint8();
					int width = mesh.get_texture_width();
					int height = mesh.get_texture_height();
					cv::Mat teximg;
					{
						if (mesh.get_texture_num_channel() == 3)
						{
							cv::Mat tmp(height, width, CV_8UC3, const_cast<uchar*>(_texdata.data()));
							tmp.convertTo(teximg, cv::COLOR_RGB2RGBA);
						}
						else if (mesh.get_texture_num_channel() == 4) {
							cv::Mat tmp(height, width, CV_8UC4, const_cast<uchar*>(_texdata.data()));
							tmp.copyTo(teximg);
						}
					}
					cv::flip(teximg, teximg, 0); //flip along x

					Corrade::Containers::Array<char> imgdata(width * height * 4);
					for (int i = 0; i < imgdata.size(); i++)
						imgdata[i] = teximg.data[i];

					auto* imgptr = new Magnum::Image2D(Magnum::PixelFormat::RGBA8Unorm, { width, height }, std::move(imgdata));
					info->texture_image = std::shared_ptr<Magnum::Image2D>(imgptr);
					info->gl_texture.setStorage(1, Magnum::GL::TextureFormat::RGBA8, info->texture_image->size())
						.setSubImage(0, {}, *info->texture_image);
				}
			}
			else
			{	//no texture coordinate
				const auto& f = mesh.get_faces();
				std::vector<int> indices(mesh.get_faces().rows() * 3);
				std::copy(f.data(), f.data() + f.rows() * 3, indices.begin());

				if (mesh.get_normal_faces().size() > 0) {
					//has normals
					std::vector<VertexData_PN_3> vdlist;
					to_vertex_data(mesh.get_vertices(), mesh.get_faces(),
						mesh.get_normal_vertices(false), mesh.get_normal_faces(),
						vdlist);
					info->gl_vertex.setData(vdlist);
					info->gl_mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
						.setCount(vdlist.size())
						.addVertexBuffer(info->gl_vertex, 0, Shader_T::Position{}, Shader_T::Normal{});
				}
				else {
					//no normal
					std::vector<VertexData_P_3> vdlist;
					to_vertex_data(mesh.get_vertices(), vdlist);

					info->gl_vertex.setData(vdlist);
					info->gl_index.setData(indices);
					info->gl_mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
						.setCount(f.rows() * 3)
						.addVertexBuffer(info->gl_vertex, 0, Shader_T::Position{})
						.setIndexBuffer(info->gl_index, 0, Magnum::GL::MeshIndexType::UnsignedInt);
				}
			}

			_update_mesh_state();
			return info.get();
		}

		inline void MeshViewer::remove_mesh(std::string name)
		{
			assert_throw(m_name2mesh.find(name) != m_name2mesh.end(), "the mesh does not exist");
			m_name2mesh.erase(name);
			_update_mesh_state();
		}

		inline void MeshViewer::clear_meshes()
		{
			m_name2mesh.clear();
		}

		inline void MeshViewer::reset()
		{
			clear_meshes();
			_init();
		}

		inline void MeshViewer::set_window_size(int width, int height)
		{
			assert_throw(width != 0 || height != 0, "cannot set window size because both width and height are 0");

			if (width && height) //both nonzero
				glfwSetWindowSize(window(), width, height);
			{
				assert_throw(m_view_range.size() == 4, "view range is not set yet");
				double dx = m_view_range(2);
				double dy = m_view_range(3);

				if (height == 0)
					height = std::round(dy / dx * width);

				if (width == 0)
					width = std::round(dx / dy * height);

				glfwSetWindowSize(window(), width, height);
			}
		}

		inline void MeshViewer::set_window_size(int width)
		{
			assert_throw(m_view_range.size() == 4, "view range is not set yet");

			//compute the height
			double dx = m_view_range(2);
			double dy = m_view_range(3);

			int height = std::round(dy / dx * width);
			set_window_size(width, height);
		}


		inline void MeshViewer::set_view_range(double x, double y, double width, double height)
		{
			//all objects translate to the center of view
			float cx = x + width / 2;
			float cy = y + height / 2;
			Magnum::Matrix4 translate_to_center = Magnum::Matrix4::translation({ -cx,-cy,0 });

			//apply orthographic projection
			Magnum::Matrix4 ortho_proj = Magnum::Matrix4::orthographicProjection({ float(width), float(height) }, 0, 1e7);

			m_tmat_camera = ortho_proj * translate_to_center;

			m_view_range << x, y, width, height;
		}

		inline void MeshViewer::set_view_range_tight()
		{
			if (m_name2mesh.size() == 0)
				return;

			fVECTOR maxc(3), minc(3);
			double dmax = std::numeric_limits<double>::max();
			//double dmin = std::numeric_limits<double>::min();

			//maxc is set to minimal, minc is set to maximal so that they will be updated by the first mesh
			maxc.fill(-dmax);
			minc.fill(dmax);

			for (auto x : m_name2mesh)
			{
				const auto& v = x.second->mesh_obj->get_vertices();
				fVECTOR _minc = v.colwise().minCoeff();
				fVECTOR _maxc = v.colwise().maxCoeff();

				maxc = maxc.cwiseMax(_maxc);
				minc = minc.cwiseMin(_minc);
			}

			double width = maxc(0) - minc(0);
			double height = maxc(1) - minc(1);

			set_view_range(minc(0), minc(1), width, height);
		}

		inline _NS_UTILITY::fVECTOR MeshViewer::get_view_range()
		{
			return m_view_range;
		}

		inline void MeshViewer::render_image_RGBA(char* rgba)
		{
			auto winsize = windowSize();
			int width = winsize[0];
			int height = winsize[1];

			glfwPollEvents();
			m_tmat_after_camera = Magnum::Matrix4::scaling({ 1,-1,1 });
			draw_only();
			glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
			m_tmat_after_camera = Magnum::Matrix4::scaling({ 1,1,1 });
		}

		inline void MeshViewer::render_image_RGBA(cv::Mat& output)
		{
			auto winsize = windowSize();
			int width = winsize[0];
			int height = winsize[1];

			output.create(height, width, CV_8UC4);
			render_image_RGBA((char*)output.data);
		}

		inline void MeshViewer::render_image_RGB(char* rgb)
		{
			auto winsize = windowSize();
			int width = winsize[0];
			int height = winsize[1];

			Magnum::Containers::Array<char> _data(width * height * 3);
			Magnum::Image2D img(Magnum::PixelFormat::RGB8Unorm, { width, height }, std::move(_data));

			glfwPollEvents();
			m_tmat_after_camera = Magnum::Matrix4::scaling({ 1,-1,1 });
			draw_only();
			glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, rgb);
			m_tmat_after_camera = Magnum::Matrix4::scaling({ 1,1,1 });
		}

		inline void MeshViewer::render_image_RGB(cv::Mat& output)
		{
			auto winsize = windowSize();
			int width = winsize[0];
			int height = winsize[1];

			output.create(height, width, CV_8UC3);
			render_image_RGB((char*)output.data);
		}

		inline MeshViewer::MeshInfo* MeshViewer::get_mesh_info(std::string name)
		{
			return m_name2mesh[name].get();
		}

		inline void MeshViewer::_update_mesh_state()
		{
			//find the max z value, and make it into -1
			double zmax = std::numeric_limits<double>::min();
			for (auto x : m_name2mesh)
			{
				const auto& v = x.second->mesh_obj->get_vertices();
				double z = v.col(2).maxCoeff();
				if (z > zmax)
					zmax = z;
			}

			//shift all models by -zmax
			m_tmat_global = Magnum::Matrix4::translation({ 0,0,(float)-zmax - 10 });
		}

		inline void MeshViewer::set_background_color(double r, double g, double b, double a) {
			m_bg_color = Magnum::Color4{ (float)r,(float)g,(float)b,(float)a };
		}
	}
};
