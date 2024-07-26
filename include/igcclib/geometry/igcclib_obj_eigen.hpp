#pragma once
//read/write obj file with eigen

#include <igcclib/core/igcclib_common.hpp>
#include <igcclib/core/igcclib_eigen.hpp>
#include <tiny_obj_loader.h>
#include <string>
#include <vector>

namespace _NS_UTILITY
{
	template<typename FLOAT_T, typename INT_T>
	class _MyTrimesh
	{
	public:
		MATRIX_t<FLOAT_T> vertices;
		MATRIX_t<INT_T> faces;

		//let uv_faces[i]=(a,b,c), faces[i]=(x,y,z),  
		//then uv[a] is the texture coordinate for vertices[x] on i-th face
		MATRIX_t<FLOAT_T> uv;
		MATRIX_t<INT_T> uv_faces;

		//just like uv, but for normals
		MATRIX_t<FLOAT_T> normals;
		MATRIX_t<INT_T> normal_faces;

		std::string name;
	};

	/// <summary>
	/// load .obj file and treat the contents as a single mesh.
	/// </summary>
	/// <param name="objfile">the filename</param>
	/// <param name="out_vertices">output vertices</param>
	/// <param name="out_faces">output faces</param>
	/// <param name="out_uv">output uv coordinates</param>
	/// <param name="out_uv_faces">output uv faces</param>
	/// <param name="out_normals">output normals</param>
	/// <param name="out_normal_faces">output normal faces</param>
	template<typename FLOAT_T, typename INT_T>
	inline void load_obj_single_mesh(std::string objfile,
		MATRIX_t<FLOAT_T>* out_vertices, MATRIX_t<INT_T>* out_faces,
		MATRIX_t<FLOAT_T>* out_uv = 0, MATRIX_t<INT_T>* out_uv_faces = 0,
		MATRIX_t<FLOAT_T>* out_normals = 0, MATRIX_t<INT_T>* out_normal_faces = 0)
	{
		using namespace std;
		using tinyobj::shape_t;
		using tinyobj::material_t;
		using tinyobj::attrib_t;
		using f_matrix = MATRIX_t<FLOAT_T>;
		using i_matrix = MATRIX_t<INT_T>;

		vector<shape_t> shapes;
		vector<material_t> materials;
		attrib_t attrib;
		string err;

		bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, objfile.c_str());
		assert_throw(ret, "error when reading file " + objfile);

		// read shapes
		bool has_normal = attrib.normals.size() > 0;
		bool has_texcoord = attrib.texcoords.size() > 0;

		// read face matrix
		vector<INT_T> arr_face;
		vector<INT_T> arr_texface;
		vector<INT_T> arr_normalface;
		for (auto& s : shapes)
		{
			auto& idx = s.mesh.indices;
			for (int i = 0; i < idx.size(); i++)
			{
				arr_face.push_back(idx[i].vertex_index);
				if (has_texcoord)
					arr_texface.push_back(idx[i].texcoord_index);
				if (has_normal)
					arr_normalface.push_back(idx[i].normal_index);
			}
		}
		i_matrix faces = Eigen::Map<i_matrix>(arr_face.data(), arr_face.size() / 3, 3);

		// read vertices
		auto n_vert = attrib.vertices.size() / 3;
		using v_type = typename decltype(attrib.vertices)::value_type;
		f_matrix vertices = Eigen::Map<MATRIX_t<v_type>>(attrib.vertices.data(), n_vert, 3).template cast<FLOAT_T>();

		// read texcoord
		f_matrix texcoord;
		i_matrix texfaces;
		if (has_texcoord)
		{
			texcoord = Eigen::Map<MATRIX_t<v_type>>(attrib.texcoords.data(), attrib.texcoords.size() / 2, 2).template cast<FLOAT_T>();
			texfaces = Eigen::Map<i_matrix>(arr_texface.data(), arr_texface.size() / 3, 3);
		}

		// read normal
		f_matrix normals;
		i_matrix normalfaces;
		if (has_normal)
		{
			normals = Eigen::Map<MATRIX_t<v_type>>(attrib.normals.data(), attrib.normals.size() / 3, 3).template cast<FLOAT_T>();
			normalfaces = Eigen::Map<i_matrix>(arr_normalface.data(), arr_normalface.size() / 3, 3);
		}

		if (out_vertices)
			*out_vertices = vertices;
		if (out_faces)
			*out_faces = faces;
		if (out_uv)
			*out_uv = texcoord;
		if (out_uv_faces)
			*out_uv_faces = texfaces;
		if (out_normals)
			*out_normals = normals;
		if (out_normal_faces)
			*out_normal_faces = normalfaces;
	}


	/// <summary>
	/// load .obj file and treat the contents as a single mesh.
	/// </summary>
	/// <param name="objstring">the obj file string</param>
	/// <param name="out_vertices">output vertices</param>
	/// <param name="out_faces">output faces</param>
	/// <param name="out_uv">output uv coordinates</param>
	/// <param name="out_uv_faces">output uv faces</param>
	/// <param name="out_normals">output normals</param>
	/// <param name="out_normal_faces">output normal faces</param>
	template<typename FLOAT_T, typename INT_T>
	inline void load_obj_single_mesh_from_string(const std::string& objstring,
		MATRIX_t<FLOAT_T>* out_vertices, MATRIX_t<INT_T>* out_faces,
		MATRIX_t<FLOAT_T>* out_uv = 0, MATRIX_t<INT_T>* out_uv_faces = 0,
		MATRIX_t<FLOAT_T>* out_normals = 0, MATRIX_t<INT_T>* out_normal_faces = 0)
	{
		using namespace std;
		using tinyobj::shape_t;
		using tinyobj::material_t;
		using tinyobj::attrib_t;
		using f_matrix = MATRIX_t<FLOAT_T>;
		using i_matrix = MATRIX_t<INT_T>;

		vector<shape_t> shapes;
		vector<material_t> materials;
		attrib_t attrib;
		string err;

		std::stringstream objstream;
		objstream << objstring;

		bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, &objstream);
		assert_throw(ret, "error load objstring !");

		// read shapes
		bool has_normal = attrib.normals.size() > 0;
		bool has_texcoord = attrib.texcoords.size() > 0;

		// read face matrix
		vector<INT_T> arr_face;
		vector<INT_T> arr_texface;
		vector<INT_T> arr_normalface;
		for (auto& s : shapes)
		{
			auto& idx = s.mesh.indices;
			for (int i = 0; i < idx.size(); i++)
			{
				arr_face.push_back(idx[i].vertex_index);
				if (has_texcoord)
					arr_texface.push_back(idx[i].texcoord_index);
				if (has_normal)
					arr_normalface.push_back(idx[i].normal_index);
			}
		}
		i_matrix faces = Eigen::Map<i_matrix>(arr_face.data(), arr_face.size() / 3, 3);

		// read vertices
		auto n_vert = attrib.vertices.size() / 3;
		using v_type = typename decltype(attrib.vertices)::value_type;
		f_matrix vertices = Eigen::Map<MATRIX_t<v_type>>(attrib.vertices.data(), n_vert, 3).template cast<FLOAT_T>();

		// read texcoord
		f_matrix texcoord;
		i_matrix texfaces;
		if (has_texcoord)
		{
			texcoord = Eigen::Map<MATRIX_t<v_type>>(attrib.texcoords.data(), attrib.texcoords.size() / 2, 2).template cast<FLOAT_T>();
			texfaces = Eigen::Map<i_matrix>(arr_texface.data(), arr_texface.size() / 3, 3);
		}

		// read normal
		f_matrix normals;
		i_matrix normalfaces;
		if (has_normal)
		{
			normals = Eigen::Map<MATRIX_t<v_type>>(attrib.normals.data(), attrib.normals.size() / 3, 3).template cast<FLOAT_T>();
			normalfaces = Eigen::Map<i_matrix>(arr_normalface.data(), arr_normalface.size() / 3, 3);
		}

		if (out_vertices)
			*out_vertices = vertices;
		if (out_faces)
			*out_faces = faces;
		if (out_uv)
			*out_uv = texcoord;
		if (out_uv_faces)
			*out_uv_faces = texfaces;
		if (out_normals)
			*out_normals = normals;
		if (out_normal_faces)
			*out_normal_faces = normalfaces;
	}

	/// <summary>
	/// load .obj file string and treat the contents as a single mesh.
	/// </summary>
	/// <param name="obj_string">obj file string</param>
	/// <returns>a _MyTrimesh object containing the mesh information</returns>
	template<typename FLOAT_T = float_type, typename INT_T = int_type>
	inline _MyTrimesh<FLOAT_T, INT_T> load_obj_single_mesh_from_string(const std::string& obj_string)
	{
		_MyTrimesh<FLOAT_T, INT_T> output;
		load_obj_single_mesh_from_string(obj_string, &output.vertices, &output.faces,
			&output.uv, &output.uv_faces, &output.normals, &output.normal_faces);
		return output;
	}
	
	/**
	 * @brief load .obj file and treat the contents as a single mesh.
	 * 
	 * @tparam FLOAT_T floating point type, double or float, for vertices
	 * @tparam INT_T integer type, int, long, uint64_t, etc, for faces
	 * @param objfile .obj file to load
	 * @return _MyTrimesh<FLOAT_T, INT_T> output mesh
	 */
	template<typename FLOAT_T = float_type, typename INT_T = int_type>
	inline _MyTrimesh<FLOAT_T, INT_T> load_obj_single_mesh(std::string objfile)
	{
		_MyTrimesh<FLOAT_T, INT_T> output;
		load_obj_single_mesh(objfile, &output.vertices, &output.faces,
			&output.uv, &output.uv_faces, &output.normals, &output.normal_faces);
		return output;
	}

	/// <summary>
	/// load .obj file and respect the mesh separations in the file
	/// </summary>
	/// <param name="filename">obj file name</param>
	/// <returns>a list of _MyTrimesh</returns>
	template<typename FLOAT_T = float_type, typename INT_T = int_type>
	inline std::vector<_MyTrimesh<FLOAT_T, INT_T>> load_obj_multi_mesh(std::string filename)
	{
		using namespace std;
		using tinyobj::shape_t;
		using tinyobj::material_t;
		using tinyobj::attrib_t;

		vector<shape_t> shapes;
		vector<material_t> materials;
		attrib_t attrib;
		string err;
		std::vector<_MyTrimesh<FLOAT_T,INT_T> > output;
		using f_matrix = MATRIX_t<FLOAT_T>;
		using i_matrix = MATRIX_t<INT_T>;

		bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename.c_str());
		assert_throw(ret, "error when reading file " + filename);

		// read shapes
		bool has_normal = attrib.normals.size() > 0;
		bool has_texcoord = attrib.texcoords.size() > 0;

		// read face matrix
		for (auto& s : shapes)
		{
			auto& idx = s.mesh.indices;
			vector<INT_T> arr_face;
			arr_face.reserve(idx.size());

			vector<INT_T> arr_texface;
			if (has_texcoord)
				arr_face.reserve(idx.size());

			vector<INT_T> arr_normalface;
			if (has_normal)
				arr_normalface.reserve(idx.size());

			for (int i = 0; i < idx.size(); i++)
			{
				arr_face.push_back(idx[i].vertex_index);
				if (has_texcoord)
					arr_texface.push_back(idx[i].texcoord_index);
				if (has_normal)
					arr_normalface.push_back(idx[i].normal_index);
			}

			auto get_vertex_face_re_index = [](
				tinyobj::real_t* rawdata,
				int n_col,
				const vector<int>& arr_face,
				MATRIX_t<FLOAT_T>& out_vertices,
				MATRIX_t<INT_T>& out_faces)
			{
				//re-index face array
				std::vector<size_t> relb_arr_face;
				auto u_idx_vertex = unique_elements(arr_face, 0, &relb_arr_face);
				out_vertices.resize(u_idx_vertex.size(), n_col);
				for (int i = 0; i < u_idx_vertex.size(); i++)
				{
					for (int k = 0; k < n_col; k++)
					{
						int idx = u_idx_vertex[i];
						out_vertices(i, k) = rawdata[n_col * idx + k];
					}
				}
				out_faces = Eigen::Map<MATRIX_t<size_t>>(relb_arr_face.data(), relb_arr_face.size() / 3, 3).template cast<INT_T>();
			};

			//re-index face array
			f_matrix vertices;
			i_matrix faces;
			get_vertex_face_re_index(attrib.vertices.data(), 3, arr_face, vertices, faces);

			//re-index texture array
			f_matrix texcoord;
			i_matrix texfaces;
			if (has_texcoord)
			{
				get_vertex_face_re_index(attrib.texcoords.data(), 2, arr_texface, texcoord, texfaces);
			}

			//re-index normal array
			f_matrix normals;
			i_matrix normalfaces;
			if (has_normal)
			{
				get_vertex_face_re_index(attrib.normals.data(), 3, arr_normalface, normals, normalfaces);
			}

			_MyTrimesh<FLOAT_T, INT_T> obj;
			obj.vertices = vertices;
			obj.faces = faces;
			obj.uv = texcoord;
			obj.uv_faces = texfaces;
			obj.normals = normals;
			obj.normal_faces = normalfaces;
			obj.name = s.name;
			output.push_back(obj);
		}
		return output;
	}
};