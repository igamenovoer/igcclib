#pragma once
#include <igcclib/igcclib_master.hpp>
#include <igcclib/geometry/TriangularMesh.hpp>
#include <igcclib/geometry/igcclib_obj_eigen.hpp>
#include "igcclib/io/igcclib_io_flatbuffer.hpp"
#include "igcclib_io_flatbuffer.hpp"

#include "generated/triangular_mesh_generated.h"
#include "io_general.hpp"
#include <string>
#include <vector>
#include <map>
#include <fstream>

# TODO: here
#include "./../common/util_opencv.h"
#include "./../common/util_opencv_eigen.h"
#include "./../common/util_common.h"
#include "io_filesys.h"

namespace _NS_UTILITY
{
	namespace fbs = fbsdata;

	/// <summary>
	/// load a TriangularMesh from flatbuffer data
	/// </summary>
	/// <param name="fbdata">the flatbuffer data</param>
	/// <param name="output">the output storage</param>
	inline void load_triangular_mesh_fbs(const fbsdata::TriangularMesh* fbdata, TriangularMesh& output)
	{
		fMATRIX vertices, texcoord, normal_vertices, transmat;
		iMATRIX faces, texfaces, normal_faces;
		ImageRGBA_u teximg;

		std::string name = fbdata->name()->str();
		read_matrix_from_fbs(fbdata->vertices(), vertices);
		read_matrix_from_fbs(fbdata->faces(), faces);
		if (fbdata->texcoord())
		{
			read_matrix_from_fbs(fbdata->texcoord(), texcoord);
			read_matrix_from_fbs(fbdata->texfaces(), texfaces);
		}

		if (fbdata->normal_vertices())
		{
			read_matrix_from_fbs(fbdata->normal_vertices(), normal_vertices);
			read_matrix_from_fbs(fbdata->normal_faces(), normal_faces);
		}

		read_matrix_from_fbs(fbdata->transmat(), transmat);

		if (fbdata->texture_image())
		{
			read_image_from_fbs(fbdata->texture_image(), teximg, 0, 255, 0, 255);
		}

		TriangularMesh::init_with_vertex_face(output, vertices, faces, &texcoord, &texfaces, &normal_vertices, &normal_faces);
		output.set_name(name);
		if (!teximg.is_empty())
			output.set_texture_image(teximg);
		output.set_transmat(transmat);
	}

	/// <summary>
	/// load a TriangularMesh from flatbuffer data
	/// </summary>
	/// <param name="filename">the data file</param>
	/// <param name="output">the output triangular mesh</param>
	inline void load_triangular_mesh_fbs(const std::string& filename, TriangularMesh& output)
	{
		std::vector<uint8_t> buf;
		read_file_as_binary(filename, buf);
		auto fbdata = fbs::GetTriangularMesh(buf.data());
		load_triangular_mesh_fbs(fbdata, output);
	}

	/**
	* \brief load a triangular mesh from obj file
	*
	* \param output the output triangular mesh
	* \param filename the obj file name
	* \param texture_filename the texture image file, empty string means no texture
	*/
	inline void load_triangular_mesh_obj(TriangularMesh& output, 
		const std::string& filename, const std::string& texture_filename = "") {
		auto _mesh = load_obj_single_mesh(filename);

		fMATRIX* uv = nullptr;
		fMATRIX* normals = nullptr;
		iMATRIX* uv_face = nullptr;
		iMATRIX* normal_faces = nullptr;

		if (_mesh.uv.size() > 0)
		{
			uv = &_mesh.uv;
			uv_face = &_mesh.uv_faces;
		}

		if (_mesh.normals.size() > 0) {
			normals = &_mesh.normals;
			normal_faces = &_mesh.normal_faces;
		}

		TriangularMesh::init_with_vertex_face(output, _mesh.vertices, _mesh.faces, uv, uv_face, normals, normal_faces);

		if (!texture_filename.empty())
		{
			auto img = cv::imread(texture_filename, cv::IMREAD_UNCHANGED);
			ImageFormat fmt = ImageFormat::NONE;
			switch (img.channels()) {
			case 1:
				fmt = ImageFormat::GRAY;
				break;
			case 3:
				cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
				fmt = ImageFormat::RGB;
				break;
			case 4:
				cv::cvtColor(img, img, cv::COLOR_BGRA2RGBA);
				fmt = ImageFormat::RGBA;
				break;
			}

			img.convertTo(img, CV_8U);
			output.set_texture_image(img.data, img.cols, img.rows, fmt);
		}
	}

	/**
	* \brief load a triangular mesh from obj string
	*
	* \param output the output triangular mesh
	* \param obj_string the obj string	
	*/
	inline void load_triangular_mesh_obj_from_string(TriangularMesh& output,
		const std::string& obj_string) {

		auto _mesh = load_obj_single_mesh_from_string(obj_string);

		fMATRIX* uv = nullptr;
		fMATRIX* normals = nullptr;
		iMATRIX* uv_face = nullptr;
		iMATRIX* normal_faces = nullptr;

		if (_mesh.uv.size() > 0)
		{
			uv = &_mesh.uv;
			uv_face = &_mesh.uv_faces;
		}

		if (_mesh.normals.size() > 0) {
			normals = &_mesh.normals;
			normal_faces = &_mesh.normal_faces;
		}

		TriangularMesh::init_with_vertex_face(output, _mesh.vertices, _mesh.faces, uv, uv_face, normals, normal_faces);		
	}
	
	inline void export_as_obj_string(
		std::ostream& of_obj, const std::string& mtl_name, const std::string& group_name,
		const TriangularMesh& input_mesh)
	{
		auto vertices = input_mesh.get_vertices();
		auto& faces = input_mesh.get_faces();
		auto vnormal_data = input_mesh.get_normal_vertices();
		auto& vnormal_faces = input_mesh.get_normal_faces();
		auto& texcoord_data = input_mesh.get_texcoord_vertices();
		auto& texcoord_faces = input_mesh.get_texcoord_faces();

		if (mtl_name.size() > 0)
		{
			of_obj << "mtllib " + mtl_name + "\n\n";
		}

		//# write vertices
		Eigen::IOFormat v_Fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", ", ", "", "", "v ", "\n");
		for (int i = 0; i < vertices.rows(); i++)
		{
			of_obj << vertices.row(i).format(v_Fmt);
		}
		of_obj << "# " + std::to_string(vertices.rows()) + " vertices\n\n";

		//# write normals
		Eigen::IOFormat vn_Fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", ", ", "", "", "vn ", "\n");
		for (int i = 0; i < vnormal_data.rows(); i++)
		{
			of_obj << vnormal_data.row(i).format(vn_Fmt);
		}
		of_obj << "# " + std::to_string(vnormal_data.rows()) + " vertex normals\n\n";

		//write texture coordinates
		Eigen::IOFormat vt_Fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", ", ", "", "", "vt ", "\n");
		for (int i = 0; i < texcoord_data.rows(); i++)
		{
			of_obj << texcoord_data.row(i).format(vt_Fmt);
		}
		of_obj << "# " + std::to_string(texcoord_data.rows()) + " texture coordinates\n\n";
		//# write faces		
		of_obj << "g " + group_name + "\n";

		if (mtl_name.size() > 0)
		{
			of_obj << "usemtl material_0\n";
		}

		//# write faces
		for (int i = 0; i < faces.rows(); i++)
		{
			std::string f_line = "f ";
			f_line += std::to_string(faces(i, 0) + 1);
			if (texcoord_faces.rows() > 0)
			{
				f_line += "/" + std::to_string(texcoord_faces.row(i)(0) + 1);
				if (vnormal_faces.rows() > 0)
				{
					f_line += "/" + std::to_string(vnormal_faces.row(i)(0) + 1);
				}
			}
			else if (vnormal_faces.rows() > 0)
			{
				f_line += "//" + std::to_string(vnormal_faces.row(i)(0) + 1);
			}
			f_line += " ";

			f_line += std::to_string(faces(i, 1) + 1);
			if (texcoord_faces.rows() > 0)
			{
				f_line += "/" + std::to_string(texcoord_faces.row(i)(1) + 1);
				if (vnormal_faces.rows() > 0)
				{
					f_line += "/" + std::to_string(vnormal_faces.row(i)(1) + 1);
				}
			}
			else if (vnormal_faces.rows() > 0)
			{
				f_line += "//" + std::to_string(vnormal_faces.row(i)(1) + 1);
			}
			f_line += " ";

			f_line += std::to_string(faces(i, 2) + 1);
			if (texcoord_faces.rows() > 0)
			{
				f_line += "/" + std::to_string(texcoord_faces.row(i)(2) + 1);
				if (vnormal_faces.rows() > 0)
				{
					f_line += "/" + std::to_string(vnormal_faces.row(i)(2) + 1);
				}
			}
			else if (vnormal_faces.rows() > 0)
			{
				f_line += "//" + std::to_string(vnormal_faces.row(i)(2) + 1);
			}
			f_line += "\n";
			of_obj << f_line;

		}
		of_obj << "# " + std::to_string(faces.rows()) + "  faces\n\n";
	}


	inline void export_as_mtl_string(std::ostream& of_mtl,const std::string& texture_name)
	{
		of_mtl << "\n";
		of_mtl << "newmtl material_0\n";	
		of_mtl << "    map_Kd " + texture_name + "\n";
	}

	/// <summary>
	/// save a triangular mesh to a obj file
	/// </summary>
	/// <param name="filename">the obj file name</param>
	/// <param name="input">the input triangular mesh</param>
	inline void save_triangular_mesh_as_obj(
		const std::string& filename, 
		const TriangularMesh& input_mesh,
		bool with_texture = true)
	{	
		std::string group_name = input_mesh.get_name();

		std::string path, fn, ext;
		fileparts(filename, path, fn, ext);
		std::string fn_out_obj = filename;
		if (ext != ".obj")
		{
			fn_out_obj = filename + ".obj";
		}
		if (path.empty())
			path = ".";

		std::string fn_out_mtl = path + "//" + fn + ".mtl";
		std::string fn_out_texture = path + "//" + fn + ".png";
		std::string texture_name = fn + ".png";
		std::string mtl_name; 

		ImageRGBA_u tex_img;
		input_mesh.get_texture_image(tex_img);
		/*std::cout << "Write obj texture channels:"<<tex_img.get_number_of_channels() << std::endl;*/
		
		if (!tex_img.is_empty() && with_texture)
		{
			mtl_name = fn + ".mtl";

			cv::Mat img;
			to_opencv_image(tex_img, img);
			if (img.channels() == 3)
				cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
			else if (img.channels() == 4)
				cv::cvtColor(img, img, cv::COLOR_RGBA2BGRA);
			cv::imwrite(fn_out_texture, img);			

			std::ofstream of_mtl(fn_out_mtl);		
			
			if (of_mtl.is_open())
			{
				std::ostringstream of_mtl_string;
				export_as_mtl_string(of_mtl_string, texture_name);
				of_mtl << of_mtl_string.str();				
				of_mtl.close();
			}
			else std::cout << "Unable to open mtl file";
		}

		std::ofstream of_obj(fn_out_obj);
		if (of_obj.is_open())
		{
			std::ostringstream of_obj_string;
			export_as_obj_string(of_obj_string, mtl_name, group_name, input_mesh);
			of_obj << of_obj_string.str();
			of_obj.close();	
		}
		else std::cout << "Unable to open obj file";
		
	}
};