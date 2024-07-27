#pragma once

#include <memory>
#include <igcclib/visualization/soft_renderer/igcclib_softlit.hpp>
#include <igcclib/visualization/soft_renderer/igcclib_softlit_helpers.hpp>
#include <igcclib/geometry/TriangularMesh.hpp>
#include <igcclib/geometry/igcclib_geometry.hpp>

namespace _NS_UTILITY {

	/*!
	 * \class SoftMeshViewer
	 *
	 * \brief rendering mesh using software renderer
	 */
	class SoftMeshViewer {
	public:
		using RenderOutputType = softlit::ShaderOutputChannel;

		struct MeshInfo {
			//key of the mesh
			std::string name;

			//the triangular mesh object, we only keep a reference
			const TriangularMesh* mesh_obj = nullptr;

			//softlit renderable representing the mesh
			std::unique_ptr<softlit::Primitive> prim;

			//color of the mesh
			fVECTOR_3 diffuse_color{ 1,1,1 };

			//use texture or color, only applicable when texture is present in the mesh
			bool use_texture = false;

			MeshInfo() {}

			void set_transmat(const fMATRIX_4& tmat) {
				//update ubo model matrix
				auto ubo = prim->getUBO_ex();
				to_matrix((fMATRIX_4)tmat.transpose(), ubo->M);

				//ubo view and projection is updated when rendering
			}

			void apply_transmat(const fMATRIX_4& tmat) {
				glm::mat4 _tmat;
				to_matrix((fMATRIX_4)tmat.transpose(), _tmat);
				auto ubo = prim->getUBO_ex();
				ubo->M = _tmat * ubo->M;
			}

			void set_texture_enable(bool tf); 
			void set_mesh_color(const fVECTOR_3& color3f); 
			void set_mesh_id(int id);
		};
		using MeshInfoPtr = std::unique_ptr<MeshInfo>;

		/*!
		 * \class Camera
		 *
		 * \brief the camera, looking along z- in local axis. The local axis is x+ right, y+ up, z+ outside.
		 */
		class Camera {
		protected:
			fVECTOR_3 m_position{ 0,0,0 }; //position
			fMATRIX_3 m_rotation = fMATRIX_3::Identity(); //right-multiply rotation matrix, camera rotation in world space

		public:
			virtual ~Camera() {}

			/** \brief right-multiply projection matrix */
			virtual fMATRIX_4 get_proj_matrix() const = 0;

			/** \brief reset the camera parameters */
			virtual void reset();

			/** \brief right-multiply view matrix */
			fMATRIX_4 get_view_matrix() const;

			/** \brief set the camera position */
			void set_position(const fVECTOR_3& pos);

			/** \brief get the camera position */
			const fVECTOR_3& get_position();

			/** \brief set rotation matrix of the camera */
			void set_rotation(const fMATRIX_3& rotmat);

			/** \brief get rotation matrix of the camera */
			const fMATRIX_3& get_rotation() const;

			/** \brief set position and rotation by 4x4 right-mul extrinsic matrix */
			void set_extrinsic_matrix(const fMATRIX_4& extmat);

			/** \brief set opencv convention extrinsic matrix. In opencv, camera looks at z+, where x+ right, y+ down.
			In contrast, our camera looks at z-, x+ right, y+ up*/
			void set_extrinsic_matrix_opencv(const fMATRIX_4& extmat);

			/**
			* \brief rotate the camera by angle-axis
			*
			* \param angle_rad the angle by which to rotate
			* \param axis the axis around which the camera is rotated
			* \param relative_to_local rotate around local axis or world axis? If true, around local axis, otherwise, world axis.
			When rotating around local axis, the camera position will not change. Rotating around world axis will change camera position
			as the rotating axis originates from world origin.
			*/
			void rotate(double angle_rad, const fVECTOR_3& axis, bool relative_to_local = true);

			/**
			* \brief look at target, such that the target is at the center of view, and the up_vector (in world space)
			is mapped to the up direction (pointing from image bottom to top) in the resulting image.
			*
			* \param target the target point
			* \param up_vector the up vector in world space
			*/
			void look_at(const fVECTOR_3& target, const fVECTOR_3& up_vector);

			//left multiply matrices
			glm::mat4 get_glm_view_matrix() const;
			glm::mat4 get_glm_proj_matrix() const;
			glm::vec3 get_glm_position() const; 
		};

		class OrthographicCamera : public Camera {
		protected:
			//view size in world unit
			double m_width = 1.0;
			double m_height = 1.0;
			double m_near = 1e-3;
			double m_far = 1e4;
		public:			
			virtual fMATRIX_4 get_proj_matrix() const override;

			/** \brief set the view size in world unit */
			void set_view_size(double width, double height);
			double get_view_width() const;
			double get_view_height() const;

			virtual void reset() override;

			/**
			* \brief  Utility function, set the camera rotation to identity
			so that it looks along z- and its x,y axis aligns with the world x,y,
			and move the camera so that a rect (x,y,width,height) in world space is
			viewed by the camera.
			*
			* \param xywh the (x,y,width,height) region in world space
			* \param distance_to_xoy distance from the camera to the world xOy plane
			*/
			void look_at_region(const fVECTOR_4& xywh, double distance_to_xoy = 1e4);
		};

		class PerspectiveCamera : public Camera {
		protected:
			glm::mat4 m_projection_matrix;

		public:
			virtual fMATRIX_4 get_proj_matrix() const override;

			/** \brief directly set the right-mul projection matrix */
			void set_proj_matrix(const fMATRIX_4& mat);

			/**
			* \brief set the projection matrix based on fov in y direction and view aspect ratio width/height
			*
			* \param fovy_deg fov in y direction, in degrees
			* \param aspect_ratio_wh view aspect ratio defined as width/height
			* \param clip_near near clipping plane
			* \param clip_far far clipping plane
			*/
			void set_proj_matrix(double fovy_deg, double aspect_ratio_wh, 
				double clip_near = 1e-2, double clip_far = 1e4);

			/** \brief given 3x3 right-mul camera intrinsic matrix, setup the projection matrix.
			The 3x3 matrix must be of the form [f,0,cx;0,f,cy;0,0,1].*/
			void set_proj_matrix_by_intrinsic(const fMATRIX_3& mat, double clip_near = 1e-2, double clip_far = 1e4);
		};

	public:
		/** \brief initialize with image width, height and camera.*/
		void init(size_t width, size_t height, const std::shared_ptr<Camera>& camera);

		/**
		* \brief replace a mesh
		*
		* \param name name of the mesh to be replaced
		* \param mesh the mesh
		* \param use_texture whether to use the texture in the mesh. If true and the mesh does have texture,
		that texture will be used. Otherwise, ignore the texture and pure color is used.
		* \return MeshViewer::MeshInfo* the created mesh entry
		*/
		MeshInfo* set_mesh(
			const std::string& name, const TriangularMesh& mesh,
			bool use_texture = true, bool use_backface_culling = false);

		/**
		* \brief remove a mesh
		*
		* \param name the name of the mesh
		*/
		void remove_mesh(const std::string& name);

		/** \brief remove all meshes */
		void clear_meshes();

		/** \brief reset the renderer into default state, removing all meshes, reseting the camera, etc. */
		void reset();

		/**
		* \brief modify the image size. One of the width and height can be 0, where it will be automatically computed
		to match with the aspect ratio of the camera view.
		*
		* \param width width in pixel
		* \param height height in pixel
		*/
		void set_image_size(size_t width, size_t height);

		/** \brief query image width */
		size_t get_image_width() const;

		/** \brief query image height */
		size_t get_image_height() const;

		/**
		* \brief adjust the camera position and view size so that all the meshes fit in the view just right, no more, no less.
		*
		* \param camera the camera to be adjusted, whose rotation will be preserved.
		* \param allow_update_camera_z whether to allow moving the camera along local z axis, to adjust its distance to the scene objects.
		If false, the camera only moves in its local xOy plane.
		*/
		void set_view_tight(OrthographicCamera& camera, bool allow_update_camera_z = false);

		/** \brief set the background color */
		void set_background_color(double r, double g, double b, double a = 1.0);

		/**
		* \brief render the current image, and output in rgba format
		*
		* \param output the pixel storage into which the result is written, in [rgbargba...] format, note that is must contain enough space to write, that is, number of pixels x4
		*/
		void render_image_RGBA(uint8_t* output);

		
		/**
		* \brief render the scene into all channels. The rich output is useful for analysis of the scene.
		*
		* \return std::map<softlit::ShaderOutputChannel, float*> the content of each output channel.
		Each output channel is a float array with width*height*4 elements, equivalent to opencv's CV_32FC4.
		For color channel, the value range is within (0,1). For other channels, the value has different meanings.
		*/
		std::map<RenderOutputType, float*> render_all();

		/** \brief get the mesh and its associated information, you can modify the information as needed. If the name does not exist, return NULL */
		MeshInfo* get_mesh_info(const std::string& name);

		/** \brief get the camera currently in use */
		Camera* get_camera();
		const Camera* get_camera() const;

		/** \brief set the camera */
		void set_camera(const std::shared_ptr<Camera>& camera);

	private:
		std::shared_ptr<softlit::Rasterizer> m_renderer; //the renderer

		std::shared_ptr<Camera> m_camera; //the active camera

		//image size
		size_t m_image_width = 0;
		size_t m_image_height = 0;

		//the meshes
		std::map<std::string, MeshInfoPtr> m_name2mesh;

		//background color
		fVECTOR_4 m_bg_color = { 0.5,0.5,0.5,0.0 };
	};

	inline void SoftMeshViewer::init(size_t width, size_t height, const std::shared_ptr<Camera>& camera)
	{
		m_image_width = width;
		m_image_height = height;
		m_camera = camera;

		softlit::RasterizerSetup rs;
		rs.vertexWinding = softlit::VertexWinding::COUNTER_CLOCKWISE;
		rs.viewport = { 0u, 0u, (uint32_t)width, (uint32_t)height };
		m_renderer = std::make_shared<softlit::Rasterizer>(rs);
	}

	inline SoftMeshViewer::MeshInfo* SoftMeshViewer::set_mesh(
		const std::string& name, const TriangularMesh& mesh, 
		bool use_texture /*= true*/, bool use_backface_culling)
	{
		MeshInfoPtr info_ptr = std::make_unique<MeshInfo>();
		MeshInfo& info = *info_ptr;
		info.mesh_obj = &mesh;
		info.name = name;

		//if the mesh has no texture, disable texture use
		use_texture &= mesh.get_texture_data_uint8().size() > 0;

		//setup primitive
		softlit::PrimitiveSetup ps;
		if (use_backface_culling)
			ps.cullMode = softlit::CullMode::CULL_BACK;
		else
			ps.cullMode = softlit::CullMode::CULL_DISABLED;
		info.prim.reset(new softlit::Primitive(ps));
		to_softlit_primitive(*info.prim, mesh);
		info.set_texture_enable(use_texture);

		//record it
		m_name2mesh[name] = std::move(info_ptr);

		return &info;
	}

	inline void SoftMeshViewer::remove_mesh(const std::string& name)
	{
		assert_throw(m_name2mesh.find(name) != m_name2mesh.end(), "the specified mesh is not found");
		m_name2mesh.erase(name);
	}

	inline void SoftMeshViewer::clear_meshes()
	{
		m_name2mesh.clear();
	}

	inline void SoftMeshViewer::reset()
	{
		m_name2mesh.clear();
		m_bg_color = { 0.5,0.5,0.5,0.0 };
	}

	inline void SoftMeshViewer::set_image_size(size_t width, size_t height)
	{
		m_image_height = height;
		m_image_width = width;

		softlit::RasterizerSetup rs;
		rs.vertexWinding = softlit::VertexWinding::COUNTER_CLOCKWISE;
		rs.viewport = { 0u, 0u, (uint32_t)width, (uint32_t)height };
		m_renderer.reset(new softlit::Rasterizer(rs));
	}

	inline size_t SoftMeshViewer::get_image_width() const
	{
		return m_image_width;
	}

	inline size_t SoftMeshViewer::get_image_height() const
	{
		return m_image_height;
	}

	inline void SoftMeshViewer::set_view_tight(OrthographicCamera& camera, bool allow_update_camera_z)
	{
		//transform all meshes to view space
		auto view_mat = camera.get_glm_view_matrix();

		//bounding box of all meshes in view space
		double dmax = std::numeric_limits<double>::max();
		double xmin, ymin, xmax, ymax, zmin, zmax;
		xmin = ymin = zmin = dmax;
		xmax = ymax = zmax = -dmax;


		for (auto iter = m_name2mesh.begin(); iter != m_name2mesh.end(); ++iter) {
			auto prim = iter->second->prim.get();
			const auto& vertices = prim->getVertexBuffer();
			auto model_mat = prim->getUBO_ex()->M;
			auto model_view = view_mat * model_mat;


			for (const auto& v : vertices) {
				glm::vec4 hv(v, 1);
				glm::vec4 v_cam = model_view * hv;
				v_cam /= v_cam.w;

				xmin = std::min(xmin, (double)v_cam.x);
				xmax = std::max(xmax, (double)v_cam.x);

				ymin = std::min(ymin, (double)v_cam.y);
				ymax = std::max(ymax, (double)v_cam.y);

				zmin = std::min(zmin, (double)v_cam.z);
				zmax = std::max(zmax, (double)v_cam.z);
			}
		}

		//new position and view size
		glm::vec4 pos;
		if (allow_update_camera_z)
		{
			pos = glm::vec4(float(xmin + xmax) / 2, float(ymin + ymax) / 2, zmax + 10.f, 1.f);
		}else {
			pos = glm::vec4(float(xmin + xmax) / 2, float(ymin + ymax) / 2, 0.f, 1.f); //new position in camera space
		}
		
		pos = glm::inverse(view_mat) * pos;
		pos /= pos.w; //position in world space

		camera.set_position(fVECTOR_3(pos[0], pos[1], pos[2]));
		camera.set_view_size(xmax - xmin, ymax - ymin);
	}

	inline void SoftMeshViewer::set_background_color(double r, double g, double b, double a /*= 1.0*/)
	{
		m_bg_color = fVECTOR_4(r, g, b, a);
	}

	inline void SoftMeshViewer::render_image_RGBA(uint8_t* output)
	{
		m_renderer->ClearBuffers(glm::vec4((float)m_bg_color[0], (float)m_bg_color[1], (float)m_bg_color[2], (float)m_bg_color[3]));

		//apply view and projection matrices, and render
		auto view_mat = m_camera->get_glm_view_matrix();
		auto proj_mat = m_camera->get_glm_proj_matrix();

		for (auto iter = m_name2mesh.begin(); iter != m_name2mesh.end(); ++iter) {
			auto prim = iter->second->prim.get();
			auto ubo = prim->getUBO_ex();
			ubo->MV = view_mat * ubo->M;
			ubo->MVP = proj_mat * ubo->MV;
			m_renderer->Draw(prim);
		}

		//read image
		//const auto& framebuf = m_renderer->getFrameBuffer();
		const auto& framebuf = m_renderer->getFrameBuffer().get_channel(softlit::ShaderOutputChannel::FINAL_COLOR);
		for (size_t i = 0; i < framebuf.size(); i++) {
			const auto& c = framebuf[i];
			for (int k = 0; k < 4; k++) {
				auto v = std::max(std::min(c[k], 1.0f), 0.0f);
				output[4 * i + k] = (uint8_t)(v * 255.0);
			}
		}
	}

	inline std::map<SoftMeshViewer::RenderOutputType, float*> SoftMeshViewer::render_all()
	{
		m_renderer->ClearBuffers(glm::vec4((float)m_bg_color[0], (float)m_bg_color[1], (float)m_bg_color[2], (float)m_bg_color[3]));

		//apply view and projection matrices, and render
		auto view_mat = m_camera->get_glm_view_matrix();
		auto proj_mat = m_camera->get_glm_proj_matrix();

		for (auto iter = m_name2mesh.begin(); iter != m_name2mesh.end(); ++iter) {
			auto prim = iter->second->prim.get();
			auto ubo = prim->getUBO_ex();
			ubo->MV = view_mat * ubo->M;
			ubo->MVP = proj_mat * ubo->MV;
			m_renderer->Draw(prim);
		}

		//get the frame buffer
		auto rdata = m_renderer->getFrameBuffer().get_all_data();
		std::map<RenderOutputType, float*> output;
		for (auto& it : rdata) {
			output[it.first] = (float*)glm::value_ptr(it.second->at(0));
		}
		return output;
	}

	inline SoftMeshViewer::MeshInfo* SoftMeshViewer::get_mesh_info(const std::string& name)
	{
		if (m_name2mesh.find(name) == m_name2mesh.end())
			return nullptr;
		else
			return m_name2mesh.at(name).get();
	}

	inline SoftMeshViewer::Camera* SoftMeshViewer::get_camera()
	{
		return m_camera.get();
	}

	inline const SoftMeshViewer::Camera* SoftMeshViewer::get_camera() const
	{
		return m_camera.get();
	}

	inline void SoftMeshViewer::set_camera(const std::shared_ptr<Camera>& camera)
	{
		m_camera = camera;
	}

	inline void SoftMeshViewer::MeshInfo::set_texture_enable(bool tf)
	{
		if (tf == use_texture)
			return;
		use_texture = tf;
	}

	inline void SoftMeshViewer::MeshInfo::set_mesh_color(const fVECTOR_3& color3f)
	{
		diffuse_color = color3f;

		auto& atrlist = const_cast<softlit::VertexAttributes&>(prim->getVertexAttributes()).attrib_vec3;
		auto index = softlit::DefaultShaderInputChannel<softlit::ShaderInputType::VERTEX_COLOR_3>::index;
		auto& v = prim->getVertexAttributes().attrib_vec3[index].m_data;
		std::fill(v.begin(), v.end(), glm::vec3(color3f[0], color3f[1], color3f[2]));
	}

	inline void SoftMeshViewer::MeshInfo::set_mesh_id(int id)
	{
		prim->set_id(id);
	}

	inline void SoftMeshViewer::Camera::reset()
	{
		m_rotation.setIdentity();
		m_position.setZero();
	}

	inline fMATRIX_4 SoftMeshViewer::Camera::get_view_matrix() const {
		fMATRIX_4 tmat;
		tmat.setIdentity();
		tmat.block(0, 0, 3, 3) = m_rotation;
		tmat.block(3, 0, 1, 3) = m_position.transpose();
		fMATRIX_4 tmat_inv = tmat.inverse();
		return tmat_inv;
	}

	inline void SoftMeshViewer::Camera::set_position(const fVECTOR_3& pos)
	{
		m_position = pos;
	}

	inline const fVECTOR_3& SoftMeshViewer::Camera::get_position()
	{
		return m_position;
	}

	inline void SoftMeshViewer::Camera::set_rotation(const fMATRIX_3& rotmat)
	{
		m_rotation = rotmat;
	}

	inline const fMATRIX_3& SoftMeshViewer::Camera::get_rotation() const
	{
		return m_rotation;
	}

	inline void SoftMeshViewer::Camera::rotate(double angle_rad, const fVECTOR_3& axis, bool relative_to_local)
	{
		fMATRIX_4 rotmat = rotation_matrix(angle_rad, axis);
		
		//relative to itself
		if (relative_to_local)
			m_rotation = rotmat.block(0, 0, 3, 3) * m_rotation;
		else {
			//relative to world
			fMATRIX_4 tmat = fMATRIX_4::Identity();
			tmat.block(0, 0, 3, 3) = m_rotation;
			tmat.block(3, 0, 1, 3) = m_position.transpose();

			tmat = tmat * rotmat;

			m_position = tmat.block(3, 0, 1, 3).transpose();
			m_rotation = tmat.block(0, 0, 3, 3);
		}
	}

	inline void SoftMeshViewer::Camera::look_at(const fVECTOR_3& target, const fVECTOR_3& up_vector)
	{
		auto viewmat = glm::lookAt(
			glm::vec3(m_position[0], m_position[1], m_position[2]),
			glm::vec3(target[0], target[1], target[2]),
			glm::vec3(up_vector[0], up_vector[1], up_vector[2])
		);
		fMATRIX_4 tmat;
		to_matrix(viewmat, tmat);
		m_rotation = tmat.block(0, 0, 3, 3); //transpose then invert the rotation matrix, which equivalent to no op.
	}

	inline glm::mat4 SoftMeshViewer::Camera::get_glm_view_matrix() const
	{
		glm::mat4 tmat;
		to_matrix((fMATRIX_4)get_view_matrix().transpose(), tmat);
		return tmat;
	}

	inline glm::mat4 SoftMeshViewer::Camera::get_glm_proj_matrix() const
	{
		glm::mat4 tmat;
		to_matrix((fMATRIX_4)get_proj_matrix().transpose(), tmat);
		return tmat;
	}

	inline glm::vec3 SoftMeshViewer::Camera::get_glm_position() const
	{
		return glm::vec3(m_position[0], m_position[1], m_position[2]);
	}

	inline void SoftMeshViewer::Camera::set_extrinsic_matrix(const fMATRIX_4& extmat) {
		fMATRIX_4 frame = extmat.inverse();
		fVECTOR_3 position = frame.leftCols(3).bottomRows(1).transpose();
		fMATRIX_3 rotmat = frame.block(0, 0, 3, 3);
		set_rotation(rotmat);
		set_position(position);
	}

	inline void SoftMeshViewer::Camera::set_extrinsic_matrix_opencv(const fMATRIX_4& extmat)
	{
		fMATRIX_4 frame = extmat.inverse();
		fVECTOR_3 position = frame.leftCols(3).bottomRows(1).transpose();
		fMATRIX_3 rotmat = frame.block(0, 0, 3, 3);
		rotmat.row(1) *= -1;
		rotmat.row(2) *= -1;
		set_rotation(rotmat);
		set_position(position);
	}

	inline fMATRIX_4 SoftMeshViewer::OrthographicCamera::get_proj_matrix() const {
		auto _projmat = glm::ortho(-m_width / 2, m_width / 2, -m_height / 2, m_height / 2, m_near, m_far);
		fMATRIX_4 projmat;
		to_matrix(_projmat, projmat);
		return projmat.transpose();
	}

	inline void SoftMeshViewer::OrthographicCamera::set_view_size(double width, double height)
	{
		m_width = width;
		m_height = height;
	}

	inline double SoftMeshViewer::OrthographicCamera::get_view_width() const
	{
		return m_width;
	}

	inline double SoftMeshViewer::OrthographicCamera::get_view_height() const
	{
		return m_height;
	}

	inline void SoftMeshViewer::OrthographicCamera::reset()
	{
		Camera::reset();
		m_width = 1.0;
		m_height = 1.0;
	}

	inline void SoftMeshViewer::OrthographicCamera::look_at_region(const fVECTOR_4& xywh, double distance_to_xoy)
	{
		m_rotation.setIdentity();
		auto x = xywh[0];
		auto y = xywh[1];
		auto w = xywh[2];
		auto h = xywh[3];

		m_position = { x + w / 2, y + h / 2, distance_to_xoy };
		m_width = w;
		m_height = h;
	}

	inline fMATRIX_4 SoftMeshViewer::PerspectiveCamera::get_proj_matrix() const {
		fMATRIX_4 output;
		to_matrix(m_projection_matrix, output);
		return output.transpose();
	}

	inline void SoftMeshViewer::PerspectiveCamera::set_proj_matrix(const fMATRIX_4& mat) {
		fMATRIX_4 mat_t = mat.transpose();
		to_matrix(mat_t, m_projection_matrix);
	}

	inline void SoftMeshViewer::PerspectiveCamera::set_proj_matrix(
		double fovy_deg, double aspect_ratio_wh,
		double clip_near /* = 1e-2 */, double clip_far /* = 1e4 */) {
		m_projection_matrix = glm::perspective(fovy_deg * MathConstant::Deg2Rad, aspect_ratio_wh, clip_near, clip_far);
	}

	inline void SoftMeshViewer::PerspectiveCamera::set_proj_matrix_by_intrinsic(
		const fMATRIX_3& mat, double clip_near, double clip_far) {
		auto width = mat(2, 0) * 2;
		auto height = mat(2, 1) * 2;
		auto focal = std::abs(mat(0, 0));
		auto aspect_ratio = width / height;
		auto fovy_deg = 2 * std::atan2(height, 2 * focal) * MathConstant::Rad2Deg;
		set_proj_matrix(fovy_deg, aspect_ratio, clip_near, clip_far);
	}

}