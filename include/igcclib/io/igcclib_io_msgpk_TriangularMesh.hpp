#pragma once

#include <igcclib/io/igcclib_io_msgpk_Master.hpp>
#include <igcclib/io/igcclib_io_msgpk_Eigen.hpp>
#include <igcclib/geometry/TriangularMesh.hpp>

_MSGPK_CUSTOM_ADAPTOR_BEGIN

namespace NSConversion {
	namespace TriangularMeshConversion {
		const int num_fields = 11;

		enum class Fields {
			NAME,
			VERTICES_LOCAL,
			FACES,
			UV_VERTICES,
			UV_FACES,
			NORMAL_VERTICES,
			NORMAL_FACES,
			TEXTURE_DATA_UINT8,
			TEXTURE_FORMAT,
			TEXTURE_SIZE_HW,
			TRANSMAT
		};
	};
};

template<>
struct object_with_zone<_NS_UTILITY::TriangularMesh> {
	using Mesh_t = _NS_UTILITY::TriangularMesh;
	using fMATRIX = _NS_UTILITY::fMATRIX;
	using iMATRIX = _NS_UTILITY::iMATRIX;

	//convert mesh to msgobj
	void operator()(msgobj::with_zone& o, Mesh_t const& v) const {
		namespace tmc = NSConversion::TriangularMeshConversion;
		o.type = type::ARRAY;
		o.via.array.size = tmc::num_fields;
		o.via.array.ptr = (msgobj*)o.zone.allocate_align(
			sizeof(msgobj) * o.via.array.size
		);

		o.via.array.ptr[(int)tmc::Fields::NAME] = msgobj(v.get_name(), o.zone);
		o.via.array.ptr[(int)tmc::Fields::VERTICES_LOCAL] = msgobj(v.get_vertices(false), o.zone);
		o.via.array.ptr[(int)tmc::Fields::FACES] = msgobj(v.get_faces(), o.zone);

		o.via.array.ptr[(int)tmc::Fields::UV_FACES] = msgobj(v.get_texcoord_vertices(), o.zone);
		o.via.array.ptr[(int)tmc::Fields::UV_VERTICES] = msgobj(v.get_texcoord_faces(), o.zone);

		o.via.array.ptr[(int)tmc::Fields::NORMAL_VERTICES] = msgobj(v.get_normal_vertices(false), o.zone);
		o.via.array.ptr[(int)tmc::Fields::NORMAL_FACES] = msgobj(v.get_normal_faces(), o.zone);

		std::tuple<size_t, size_t> texsize(v.get_texture_height(), v.get_texture_width());
		o.via.array.ptr[(int)tmc::Fields::TEXTURE_DATA_UINT8] = msgobj(v.get_texture_data_uint8(), o.zone);
		o.via.array.ptr[(int)tmc::Fields::TEXTURE_FORMAT] = msgobj((int)v.get_texture_format(), o.zone);
		o.via.array.ptr[(int)tmc::Fields::TEXTURE_SIZE_HW] = msgobj(texsize, o.zone);

		o.via.array.ptr[(int)tmc::Fields::TRANSMAT] = msgobj(v.get_transmat(), o.zone);
	}
};

template<>
struct convert<_NS_UTILITY::TriangularMesh> {
	using Mesh_t = _NS_UTILITY::TriangularMesh;
	using fMATRIX = _NS_UTILITY::fMATRIX;
	using iMATRIX = _NS_UTILITY::iMATRIX;

	//from msgobj to Mesh_t
	msgobj const& operator()(msgobj const& o, Mesh_t& v) const {
		namespace tmc = NSConversion::TriangularMeshConversion;
		using fd = tmc::Fields;
		if (o.type != type::ARRAY) throw type_error();
		if (o.via.array.size != tmc::num_fields) throw type_error();

		//read data
		auto name = o.via.array.ptr[(int)fd::NAME].as<std::string>();
		auto vertices = o.via.array.ptr[(int)fd::VERTICES_LOCAL].as<fMATRIX>();
		auto faces = o.via.array.ptr[(int)fd::FACES].as<iMATRIX>();

		auto uv_vertices = o.via.array.ptr[(int)fd::UV_VERTICES].as<fMATRIX>();
		auto uv_faces = o.via.array.ptr[(int)fd::UV_FACES].as<iMATRIX>();

		auto normal_vertices = o.via.array.ptr[(int)fd::NORMAL_VERTICES].as<fMATRIX>();
		auto normal_faces = o.via.array.ptr[(int)fd::NORMAL_FACES].as<iMATRIX>();

		auto texture_data_uint8 = o.via.array.ptr[(int)fd::TEXTURE_DATA_UINT8].as<std::vector<uint8_t>>();
		auto texture_size_hw = o.via.array.ptr[(int)fd::TEXTURE_SIZE_HW].as<std::tuple<size_t, size_t>>();
		auto texture_format = o.via.array.ptr[(int)fd::TEXTURE_FORMAT].as<int>();

		using Transmat_t = std::decay<decltype(((Mesh_t*)nullptr)->get_transmat())>::type; //trick to get return type
		auto transmat = o.via.array.ptr[(int)fd::TRANSMAT].as<Transmat_t>();

		//update the mesh
		{
			fMATRIX* _uv_vertices = uv_vertices.size() > 0 ? &uv_vertices : nullptr;
			iMATRIX* _uv_faces = _uv_vertices ? &uv_faces : nullptr;
			fMATRIX* _normal_vertices = normal_vertices.size() > 0 ? &normal_vertices : nullptr;
			iMATRIX* _normal_faces = _normal_vertices ? &normal_faces : nullptr;
			Mesh_t::init_with_vertex_face(v, vertices, faces, _uv_vertices, _uv_faces, _normal_vertices, _normal_faces);
			v.set_name(name);
			v.set_transmat(transmat);
			
			auto height = std::get<0>(texture_size_hw);
			auto width = std::get<1>(texture_size_hw);
			if (texture_data_uint8.size() > 0)
			{
				v.set_texture_image(texture_data_uint8.data(),width, height,
					(_NS_UTILITY::ImageFormat)texture_format);
			}
		}

		return o;
	}
};

//the packing interface, not very necessary because you can always convert the object
//to msgobj and then pack the msgobj instead, though this will leads to more temporal
//memory copies.
template<>
struct pack<_NS_UTILITY::TriangularMesh> {
	using Mesh_t = _NS_UTILITY::TriangularMesh;
	using fMATRIX = _NS_UTILITY::fMATRIX;
	using iMATRIX = _NS_UTILITY::iMATRIX;

	//pack triangular mesh to stream, it is actually the same as object_with_zone<>,
	//but using the packing interface
	template<typename Stream>
	packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, Mesh_t const& v) const {
		namespace tmc = NSConversion::TriangularMeshConversion;

		o.pack_array(tmc::num_fields);

		//the packing order must be consistent with object_with_zone<>
		o.pack(v.get_name());
		o.pack(v.get_vertices(false));
		o.pack(v.get_faces());
		o.pack(v.get_texcoord_vertices());
		o.pack(v.get_texcoord_faces());
		o.pack(v.get_normal_vertices(false));
		o.pack(v.get_normal_faces());
		o.pack(v.get_texture_data_uint8());

		o.pack((int)v.get_texture_format());
		std::tuple<size_t, size_t> texture_size_hw(v.get_texture_height(), v.get_texture_width());
		o.pack(texture_size_hw);

		o.pack(v.get_transmat());

		return o;
	}
};

_MSGPK_CUSTOM_ADAPTOR_END
