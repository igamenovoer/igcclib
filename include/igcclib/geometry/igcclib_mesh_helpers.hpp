#pragma once

#include <string>
#include <vector>
#include <map>

#include <igcclib/core/igcclib_common.hpp>
#include "igcclib_obj_eigen.hpp"
#include <igcclib/vision/igcclib_opencv_eigen.hpp>
#include "TriangularMesh.hpp"

namespace _NS_UTILITY
{
	inline std::vector<TriangularMesh> load_obj_as_meshlist(std::string filename)
	{
		std::vector<TriangularMesh> output;
		auto meshlist = load_obj_multi_mesh<float_type, int_type>(filename);
		for (auto& m : meshlist)
		{
			TriangularMesh tmesh;
			TriangularMesh::init_with_vertex_face(tmesh,
				m.vertices, m.faces, &m.uv, &m.uv_faces,
				&m.normals, &m.normal_faces);
			tmesh.set_name(m.name);
			output.push_back(tmesh);
		}
		return output;
	}

	inline std::map<std::string, TriangularMesh> load_obj_as_name2mesh(std::string filename)
	{
		auto meshlist = load_obj_as_meshlist(filename);
		std::map<std::string, TriangularMesh> output;
		for (auto& x : meshlist)
		{
			output[x.get_name()] = x;
		}
		return output;
	}

	inline void load_obj(std::string objfile, TriangularMesh& output, std::string texture_file = "")
	{
		auto mesh = load_obj_single_mesh<float_type, int_type>(objfile);

		// create object
		TriangularMesh::init_with_vertex_face(output, mesh.vertices, mesh.faces,&mesh.uv, &mesh.uv_faces,&mesh.normals, &mesh.normal_faces);
		output.set_name(mesh.name);

		// do we have texture?
		if (!texture_file.empty())
		{
			auto _img = cv::imread(texture_file, cv::IMREAD_UNCHANGED);
			if (_img.channels() == 4)
				cv::cvtColor(_img, _img, cv::COLOR_BGRA2RGBA);

			if (_img.channels() == 3)
				cv::cvtColor(_img, _img, cv::COLOR_BGR2RGB);

			TriangularMesh::TextureImageType img;
			to_eigen_image(_img, img);
			output.set_texture_image(img);
		}
	}
};
