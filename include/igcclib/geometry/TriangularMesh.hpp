#pragma once

#include <igcclib/core/igcclib_eigen.hpp>
#include <igcclib/core/igcclib_common.hpp>
#include <igcclib/geometry/igcclib_geometry.hpp>

namespace _NS_UTILITY
{
	/// <summary>
	/// The triangular mesh.
	/// </summary>
	class TriangularMesh: public IExplicitCopyable<TriangularMesh>
	{
	public:
		using TextureImageType = ImageRGBA_u;

		/** \brief user-defined vertex attributes, the format is like texture coordinates, which consists of 
		a list of all values and an Nx3 face array indexing into the values */
		struct VertexAttributePerFace {
			fMATRIX values;
			iMATRIX faces;

			template<typename Archive_t>
			void serialize(Archive_t& ar) {
				ar(values);
				ar(faces);
			}
		};

	protected:
		//name
		std::string m_name;

		//mesh in matrix form
		fMATRIX m_vertices;
		iMATRIX m_faces;

		//UV texture coordinate
		fMATRIX m_tex_vertices; //nx2, each row is a uv coordinate
		iMATRIX m_tex_faces; //index into m_tex_vertices, same size as m_faces, assign a texture coordinate to each vertex in each face

		//normals
		fMATRIX m_normal_vertices; //nx3, each row is a normal
		iMATRIX m_normal_faces; //index into normal_vertices, the vertex normal for each vertex in each face

		//texture image flattened into [RGBARGBA] format,
		//which is useful for OpenCV applications
		std::vector<uint8_t> m_texture_data_uint8;
		ImageFormat m_texture_format = ImageFormat::NONE;
		size_t m_texture_width = 0;
		size_t m_texture_height = 0;

		////vertex color
		//fMATRIX m_color4f_vertices;
		//iMATRIX m_color4f_faces; //index into m_color4f_vertices, same length as m_faces, assign a color to each vertex in each face

		//transformation matrix
		fMATRIX_4 m_transmat;

		//additional per-face vertex attributes
		std::map<int, VertexAttributePerFace> m_vertex_attributes;

	public:
		TriangularMesh(){
			m_transmat.setIdentity();
		};

		virtual ~TriangularMesh(){};

		// ===================== get/set texture =====================
		bool has_texture_image() const { return m_texture_data_uint8.size() > 0;}

		/// <summary>
		/// remove the texture
		/// </summary>
		void clear_texture();	

		/// <summary>
		/// set texture into this mesh
		/// </summary>
		/// <param name="image_data">the texture image data, in RGBRGB format</param>
		/// <param name="width">width of the texture image</param>
		/// <param name="height">height of the texture image</param>
		/// <param name="format">image format, supports all format except NONE</param>
		virtual void set_texture_image(const uint8_t* image_data, size_t width, size_t height, ImageFormat format);		

		/** \brief create an empty texture of specified size */
		virtual void set_texture_image(size_t width, size_t height, ImageFormat format);

		/// <summary>
		/// set the texture image
		/// </summary>
		/// <param name="img">the texture image</param>
		virtual void set_texture_image(const ImageRGBA_u& img);	

		void get_texture_image(ImageRGBA_u& output) const {
			int nch = get_num_channel(m_texture_format);
			output.from_linear_buffer<uint8_t>(
				m_texture_data_uint8, m_texture_width, m_texture_height, nch, 0, 255, 0, 255);
		}

		const std::vector<uint8_t>& get_texture_data_uint8() const
		{
			return m_texture_data_uint8;
		}

		size_t get_texture_width() const {
			return m_texture_width;
		}

		size_t get_texture_height() const {
			return m_texture_height;
		}

		ImageFormat get_texture_format() const {
			return m_texture_format;
		}

		size_t get_texture_num_channel() const {
			return get_num_channel(m_texture_format);
		}

		// ======== user-defined vertex attributes ========
		void set_vertex_attribute(int key, const VertexAttributePerFace& attrib) {
			m_vertex_attributes[key] = attrib;
		}

		/** \brief get a vertex attribute, return nullptr if that attribute does not exist */
		VertexAttributePerFace* get_vertex_attribute(int key) {
			if (m_vertex_attributes.find(key) == m_vertex_attributes.end())
				return nullptr;
			else return &m_vertex_attributes[key];
		}

		/** \brief get a vertex attribute, return nullptr if that attribute does not exist */
		const VertexAttributePerFace* get_vertex_attribute(int key) const {
			if (m_vertex_attributes.find(key) == m_vertex_attributes.end())
				return nullptr;
			else return &m_vertex_attributes.at(key);
		}

		// ====================== get/set name ===================
		const std::string& get_name() const { return m_name; }
		void set_name(const std::string& val) { m_name = val; }

		//check if there is any vertex
		bool is_empty() const {	return m_vertices.size() == 0; }

		// ===================== get/set vertex and face =======================

		/// <summary>
		/// get the vertices in global or local coordinates
		/// </summary>
		/// <param name="is_global_coordinate">if True, return vertices in global coordinate, otherwise
		/// in local coordinate</param>
		/// <returns>the vertices</returns>
		fMATRIX get_vertices(bool is_global_coordinate = true) const;

		/// <summary>
		/// get the vertices in global or local coordinates
		/// </summary>
		/// <param name="is_global_coordinate">if True, return vertices in global coordinate, otherwise
		/// in local coordinate</param>
		/// <param name="output">the output storage</param>
		void get_vertices(fMATRIX& output, bool is_global_coordinate = true) const;


		/// <summary>
		/// get selected vertices
		/// </summary>
		/// <param name="output">output of the vertices</param>
		/// <param name="idx">indices of the vertex</param>
		/// <param name="is_global_coordinate">should return the global coordinate of the vertex?</param>
		void get_vertices(fMATRIX& output, const std::vector<size_t>& idx, bool is_global_coordinate = true) const;	

		/// <summary>
		/// get a single vertex
		/// </summary>
		/// <param name="output">output of the vertex</param>
		/// <param name="idx">index of the vertex</param>
		/// <param name="is_global_coordinate">should return the global coordinate of the vertex?</param>
		void get_vertex(fVECTOR& output, size_t idx, bool is_global_coordinate = true) const;

		/** \brief get the center of the mesh as the mean of vertices */
		fVECTOR_3 get_mean_center(bool is_global_coordinate = true) const;

		/** \brief get the axis-aligned bounding box corners */
		void get_aabb_corner(fVECTOR_3& min_corner, fVECTOR_3& max_corner, bool is_global_coordinate = true) const;

		/// <summary>
		/// modify the vertices
		/// </summary>
		/// <param name="vertices">the input vertices</param>
		/// <param name="is_global_coordinate">if true, the input vertices are assumed to be in global coordinate, otherwise in local coordinate</param>
		void set_vertices(const fMATRIX& vertices, bool is_global_coordinate = true);

		const iMATRIX& get_faces() const { return m_faces; }
		void set_faces(const iMATRIX& faces){ m_faces = faces; }

		//=============== derived vertices ================
		void get_barycentric_vertex(fVECTOR& output, size_t face_index, const fVECTOR_3& bcpos,
			bool is_global_coordinate = true) const;

		//assume that m_vertices and m_normal_vertices are correspondence
		//All m_tex_vertices have been used
		virtual void split_mesh();

		/**
		* \brief create a submesh by selecting some face
		*
		* \param idxfaces indices of faces in selection
		* \param out_mesh the submesh
		* \param out_idx_vertices the extracted vertices
		* \param out_idx_tex_vertices the extracted texcoord vertices
		* \param out_idx_normal_vertices the extracted normal vertices
		*/
		virtual void submesh_by_faces(
			const std::vector<int_type>& idxfaces,
			TriangularMesh* out_mesh,
			std::vector<int_type>* out_idx_vertices = nullptr,
			std::vector<int_type>* out_idx_tex_vertices = nullptr,
			std::vector<int_type>* out_idx_normal_vertices = nullptr
		) const;

		//=================== get/set texture coordinate ==================
		const fMATRIX& get_texcoord_vertices() const { return m_tex_vertices; }
		void set_texcoord_vertices(const fMATRIX& tex_vertices){m_tex_vertices = tex_vertices;}
		const iMATRIX& get_texcoord_faces() const { return m_tex_faces; }
		void set_texcoord_faces(const iMATRIX& tex_faces){m_tex_faces = tex_faces;}

		//=================== get/set transformation matrix ================
		virtual void set_transmat(const fMATRIX_4& transmat) { m_transmat = transmat; }
		virtual void set_transmat(const fMATRIX& transmat) { m_transmat = transmat; }
		virtual void apply_transmat(const fMATRIX_4& tmat) { m_transmat *= tmat;}
		virtual void apply_transmat(const fMATRIX& tmat) { m_transmat *= tmat; }
		virtual const fMATRIX_4& get_transmat() const { return m_transmat; }

		//================= get/set normals ====================
		/// <summary>
		/// set normals
		/// </summary>
		/// <param name="normal_vertices">the normals.</param>
		/// <param name="is_global_coordinate">if true, the normals are in global coordinate,
		/// otherwise they are assumed to be in local coordinate</param>
		void set_normal_vertices(const fMATRIX& normal_vertices, bool is_global_coordinate = true);
		void set_normal_faces(const iMATRIX& normal_faces) { m_normal_faces = normal_faces; }

		/** \brief recompute normal for each vertex. Each vertex is assumed to have a unique normal. */
		void recompute_normal_per_vertex();

		/// <summary>
		/// get the normals
		/// </summary>
		/// <param name="is_global_coordinate">if true, the normals are in global coorindate, otherwise
		/// in local coordinate</param>
		/// <returns>the normals</returns>
		fMATRIX get_normal_vertices(bool is_global_coordinate = true) const;

		/// <summary>
		/// get the normals
		/// </summary>
		/// <param name="is_global_coordinate">if true, the normals are in global coorindate, otherwise
		/// in local coordinate</param>
		void get_normal_vertices(fMATRIX& output, bool is_global_coordinate = true) const;

		const iMATRIX& get_normal_faces() const { return m_normal_faces; }

		// =============== counting ================
		/// <summary>
		/// get the number of vertices
		/// </summary>
		size_t get_num_vertices() const {
			return m_vertices.rows();
		}

		/// <summary>
		/// number of faces
		/// </summary>
		size_t get_num_faces() const {
			return m_faces.rows();
		}

		size_t get_num_texcoord_vertices() const {
			return m_tex_vertices.rows();
		}

		size_t get_num_normal_vertices() const {
			return m_normal_vertices.rows();
		}

		virtual void copy_to(TriangularMesh& dst) const override;

		/** \brief get face centers, the i-th face center is for the i-th face */
		fMATRIX get_face_centers(bool is_global = true) const;

	public:
		//cereal serialization support
		template<typename Archive_t>
		void serialize(Archive_t& ar) {
			ar(m_name);
			ar(m_vertices);
			ar(m_faces);
			ar(m_tex_vertices);
			ar(m_tex_faces);
			ar(m_normal_vertices);
			ar(m_normal_faces);
			ar(m_texture_data_uint8);
			ar(m_texture_format);
			ar(m_texture_width);
			ar(m_texture_height);
			ar(m_transmat);
			ar(m_vertex_attributes);
		}

	public:
		// create triangular mesh from vertex and face array
		static void init_with_vertex_face(TriangularMesh& output,
			const fMATRIX& vertices, const iMATRIX& faces,
			const fMATRIX* uv = 0, const iMATRIX* uv_face = 0, 
			const fMATRIX* normals =0, const iMATRIX* normal_face =0);
	};
};
namespace _NS_UTILITY
{
	inline void TriangularMesh::clear_texture()
	{
		m_texture_data_uint8.clear();
		m_texture_height = 0;
		m_texture_width = 0;
		m_texture_format = ImageFormat::NONE;
	}

	inline void TriangularMesh::set_texture_image(const uint8_t* image_data, size_t width, size_t height, ImageFormat format)
	{
		int n_channel = get_num_channel(format);
		size_t n_pixel = width * height * n_channel;
		m_texture_data_uint8.resize(n_pixel);
		m_texture_data_uint8.assign(image_data, image_data + n_pixel);

		m_texture_format = format;
		m_texture_width = width;
		m_texture_height = height;
	}

	inline void TriangularMesh::set_texture_image(const ImageRGBA_u& img)
	{
		img.to_linear_buffer(m_texture_data_uint8, 0, 255, 0, 255);
		m_texture_width = img.get_width();
		m_texture_height = img.get_height();

		switch (img.get_number_of_channels()) {
		case 1:
			m_texture_format = ImageFormat::GRAY;
			break;
		case 3:
			m_texture_format = ImageFormat::RGB;
			break;
		case 4:
			m_texture_format = ImageFormat::RGBA;
			break;
		default:
			assert_throw(false, "unsupported texture format");
		}
	}

	inline void TriangularMesh::set_texture_image(
		size_t width, size_t height, ImageFormat format)
	{
		m_texture_width = width;
		m_texture_height = height;
		m_texture_format = format;

		auto nch = get_num_channel(format);
		m_texture_data_uint8.resize(width * height * nch);
		memset(m_texture_data_uint8.data(), 0, m_texture_data_uint8.size());
	}

	inline fMATRIX TriangularMesh::get_vertices(bool is_global_coordinate) const {
		fMATRIX output;
		if (is_global_coordinate)
			transform_points(m_vertices, m_transmat, output);
		else
			output = m_vertices;
		return output;
	}

	inline void TriangularMesh::get_vertices(fMATRIX& output, bool is_global_coordinate) const {
		if (is_global_coordinate)
			transform_points(m_vertices, m_transmat, output);
		else
			output = m_vertices;
	}

	inline void TriangularMesh::get_vertices(fMATRIX& output, const std::vector<size_t>& idx, bool is_global_coordinate) const {
		auto nrow = idx.size();
		auto ndim = m_vertices.cols();

		output.resize(nrow, ndim);
		for (size_t i = 0; i < idx.size(); i++)
			output.row(i) = m_vertices.row(idx[i]);

		if (is_global_coordinate)
			transform_points(output, m_transmat, output);
	}

	inline void TriangularMesh::get_vertex(fVECTOR& output, size_t idx, bool is_global_coordinate/* = true*/) const {
		if (!is_global_coordinate)
			output = m_vertices.row(idx);
		else {
			auto ndim = m_vertices.cols();
			fVECTOR tmp = fVECTOR::Ones(ndim + 1);
			tmp.block(0, 0, ndim, 1) = m_vertices.row(idx);
			tmp = m_transmat.transpose() * tmp;
			output = tmp.block(0, 0, ndim, 1) / tmp(ndim);
		}
	}


	inline fVECTOR_3 TriangularMesh::get_mean_center(bool is_global_coordinate /*= true*/) const
	{
		fVECTOR_3 center = m_vertices.colwise().mean();
		if (is_global_coordinate)
		{
			Eigen::Transform<float_type, 3, Eigen::Projective> tform(m_transmat.transpose());
			fVECTOR_4 tmp = tform * center.homogeneous();
			center = tmp.topRows(3) / tmp[3];
		}
		
		return center;
	}

	inline void TriangularMesh::get_aabb_corner(
		fVECTOR_3& min_corner, fVECTOR_3& max_corner, bool is_global_coordinate /*= true*/) const
	{
		if (!is_global_coordinate)
		{
			min_corner = m_vertices.colwise().minCoeff();
			max_corner = m_vertices.colwise().maxCoeff();
		}
		else {
			auto v = get_vertices(true);
			min_corner = v.colwise().minCoeff();
			max_corner = v.colwise().maxCoeff();
		}
	}

	inline void TriangularMesh::copy_to(TriangularMesh& dst) const
	{
		dst = *this; //use default assignment
	}

	inline void TriangularMesh::init_with_vertex_face(
		TriangularMesh& output,
		const fMATRIX& vertices, const iMATRIX& faces, 
		const fMATRIX* uv /*= 0*/, const iMATRIX* uv_face /*= 0*/, 
		const fMATRIX* normals /*=0*/, const iMATRIX* normal_face /*=0*/)
	{
		TriangularMesh& obj = output;
		obj.m_vertices = vertices;
		obj.m_faces = faces;
		if (uv)
		{
			assert_throw(uv_face, "uv is provided but uv_face is missing");
			obj.m_tex_vertices = *uv;
			obj.m_tex_faces = *uv_face;
		}

		if (normals)
		{
			assert_throw(normal_face, "normal is provided by normal_face is missing");
			obj.m_normal_vertices = *normals;
			obj.m_normal_faces = *normal_face;
		}
	}

	inline void TriangularMesh::set_normal_vertices(const fMATRIX& normal_vertices, bool is_global_coordinate) {
		if (is_global_coordinate)
		{
			transform_vectors(normal_vertices, m_transmat.inverse(), m_normal_vertices);
		}
		else
			m_normal_vertices = normal_vertices;
		normalize_rows(m_normal_vertices);
	}

	inline void TriangularMesh::recompute_normal_per_vertex()
	{
		fMATRIX v_normals(m_vertices.rows(), 3);
		fVECTOR num_adjface_per_vertex(m_vertices.rows());
		v_normals.setZero();
		num_adjface_per_vertex.setZero();

		for (Eigen::Index i = 0; i < m_faces.rows(); i++) {
			//compute normal for this face
			auto a = m_faces(i, 0);
			auto b = m_faces(i, 1);
			auto c = m_faces(i, 2);

			fVECTOR_3 ab = m_vertices.row(b) - m_vertices.row(a);
			fVECTOR_3 ac = m_vertices.row(c) - m_vertices.row(a);
			fVECTOR_3 normal = ab.cross(ac).normalized();

			v_normals.row(a) += normal;
			num_adjface_per_vertex(a) += 1;

			v_normals.row(b) += normal;
			num_adjface_per_vertex(b) += 1;

			v_normals.row(c) += normal;
			num_adjface_per_vertex(c) += 1;
		}

		v_normals.array().colwise() /= num_adjface_per_vertex.array();
		v_normals.rowwise().normalize();
		m_normal_vertices = v_normals;
		m_normal_faces = m_faces;
	}

	inline fMATRIX TriangularMesh::get_normal_vertices(bool is_global_coordinate/* = true*/) const {
		fMATRIX output;
		if (is_global_coordinate)
		{
			transform_vectors(m_normal_vertices, m_transmat, output);
			normalize_rows(output);
		}
		else
			output = m_normal_vertices;
		return output;
	}

	inline void TriangularMesh::get_normal_vertices(fMATRIX& output, bool is_global_coordinate /*= true*/) const {
		if (is_global_coordinate)
		{
			transform_vectors(m_normal_vertices, m_transmat, output);
			normalize_rows(output);
		}
		else
			output = m_normal_vertices;
	}

	inline void TriangularMesh::set_vertices(const fMATRIX& vertices, bool is_global_coordinate /*= true*/)
	{
		if (is_global_coordinate)
		{
			//convert it to local coordinate first
			transform_points(vertices, m_transmat.inverse(), m_vertices);
		}
		else
		{
			m_vertices = vertices;
		}
	}

	inline void TriangularMesh::get_barycentric_vertex(fVECTOR& output, size_t face_index, const fVECTOR_3& bcpos,
		bool is_global_coordinate/* = true*/) const {
		iVECTOR_3 idxv = m_faces.row(face_index);
		fMATRIX vts;
		get_vertices(vts, { (size_t)idxv[0], (size_t)idxv[1], (size_t)idxv[2] }, is_global_coordinate);

		//compute barycentric coords
		output = bcpos.transpose() * vts;
	}

	inline void TriangularMesh::submesh_by_faces(
		const std::vector<int_type>& idxfaces,
		TriangularMesh* out_mesh,
		std::vector<int_type>* out_idx_vertices /*= nullptr*/,
		std::vector<int_type>* out_idx_tex_vertices /*= nullptr*/,
		std::vector<int_type>* out_idx_normal_vertices /*= nullptr*/
	) const
	{
		//mesh information
		out_mesh->set_transmat(m_transmat);
		out_mesh->set_texture_image(m_texture_data_uint8.data(),m_texture_width,m_texture_height,m_texture_format);
		out_mesh->set_name(m_name);
		

		//geometry vertex and face
		auto x = get_sub_matrix(m_faces, idxfaces, -1);
		std::vector<size_t> relb_arr_face;
		std::vector<int_type> arr_face(x.size());
		Eigen::Map<iMATRIX>(arr_face.data(), x.rows(), x.cols()) = x;

		auto u_idx_vertex = unique_elements(arr_face, 0, &relb_arr_face);

		auto sub_vertex = get_sub_matrix(m_vertices, u_idx_vertex, -1);
		auto sub_face = Eigen::Map<MATRIX_t<size_t>>(relb_arr_face.data(), relb_arr_face.size() / 3, 3).template cast<int_type>();
		if (out_idx_vertices)
		{
			*out_idx_vertices = u_idx_vertex;
		}
		out_mesh->set_vertices(sub_vertex,false);
		out_mesh->set_faces(sub_face);

		//texture vertex and face	
		if (m_tex_vertices.size() > 0)
		{
			auto x = get_sub_matrix(m_tex_faces, idxfaces, -1);
			std::vector<size_t> relb_arr_face;
			std::vector<int_type> arr_face(x.size());
			Eigen::Map<iMATRIX>(arr_face.data(), x.rows(), x.cols()) = x;

			auto u_idx_vertex = unique_elements(arr_face, 0, &relb_arr_face);

			auto sub_tex_vertex = get_sub_matrix(m_tex_vertices, u_idx_vertex, -1);
			auto sub_tex_face = Eigen::Map<MATRIX_t<size_t>>(relb_arr_face.data(), relb_arr_face.size() / 3, 3).template cast<int_type>();
			if (out_idx_tex_vertices)
			{
				*out_idx_tex_vertices = u_idx_vertex;
			}
			out_mesh->set_texcoord_vertices(sub_tex_vertex);
			out_mesh->set_texcoord_faces(sub_tex_face);
		}

		//normal vertex an face		
		if (m_normal_vertices.size() > 0)
		{
			auto x = get_sub_matrix(m_normal_faces, idxfaces, -1);
			std::vector<size_t> relb_arr_face;
			std::vector<int_type> arr_face(x.size());
			Eigen::Map<iMATRIX>(arr_face.data(), x.rows(), x.cols()) = x;

			auto u_idx_vertex = unique_elements(arr_face, 0, &relb_arr_face);

			auto sub_normal_vertex = get_sub_matrix(m_normal_vertices, u_idx_vertex, -1);
			auto sub_normal_face = Eigen::Map<MATRIX_t<size_t>>(relb_arr_face.data(), relb_arr_face.size() / 3, 3).template cast<int_type>();
			if (out_idx_normal_vertices)
			{
				*out_idx_normal_vertices = u_idx_vertex;
			}
			out_mesh->set_normal_vertices(sub_normal_vertex, false);
			out_mesh->set_normal_faces(sub_normal_face);
		}
	}

	inline fMATRIX TriangularMesh::get_face_centers(bool is_global) const
	{
		fMATRIX output(m_faces.rows(), 3);
		for (Eigen::Index i = 0; i < m_faces.rows(); i++)
		{
			iVECTOR_3 f = m_faces.row(i);
			output.row(i) = (m_vertices.row(f[0]) + m_vertices.row(f[1]) + m_vertices.row(f[2])) / 3;
		}

		if (is_global)
			return transform_points(output, m_transmat);
		else
			return output;
	}

	inline void TriangularMesh::split_mesh()
	{
		assert_throw(m_faces.isApprox(m_normal_faces), "m_vertices and m_normal_vertices are not correspondence");
		//Align m_faces and m_normal_faces to  m_tex_faces		
		if (!m_faces.isApprox(m_tex_faces))
		{
			if (m_tex_vertices.rows() > m_vertices.rows())
			{
				fMATRIX v(m_tex_vertices.rows(), 3);
				fMATRIX vn(m_tex_vertices.rows(), 3);

				for (int i = 0; i < m_faces.rows(); i++)
				{
					v.row(m_tex_faces(i, 0)) = m_vertices.row(m_faces(i, 0));
					vn.row(m_tex_faces(i, 0)) = m_normal_vertices.row(m_normal_faces(i, 0));

					v.row(m_tex_faces(i, 1)) = m_vertices.row(m_faces(i, 1));
					vn.row(m_tex_faces(i, 1)) = m_normal_vertices.row(m_normal_faces(i, 1));

					v.row(m_tex_faces(i, 2)) = m_vertices.row(m_faces(i, 2));
					vn.row(m_tex_faces(i, 2)) = m_normal_vertices.row(m_normal_faces(i, 2));
				}
				m_vertices = v;
				m_normal_vertices = vn;
				m_faces = m_tex_faces;
				m_normal_faces = m_tex_faces;

			}
			else
			{
				fMATRIX uv(m_vertices.rows(), m_tex_vertices.cols());
				for (int i = 0; i < m_faces.rows(); i++)
				{
					uv.row(m_faces(i, 0)) = m_tex_vertices.row(m_tex_faces(i, 0));
					uv.row(m_faces(i, 1)) = m_tex_vertices.row(m_tex_faces(i, 1));
					uv.row(m_faces(i, 2)) = m_tex_vertices.row(m_tex_faces(i, 2));
				}
				m_tex_vertices = uv;
				m_tex_faces = m_faces;
			}
		}
	}
};