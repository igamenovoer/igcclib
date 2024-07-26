#pragma once

#include "msgpk_Master.h"

#include "./../common/def_eigen.h"

_MSGPK_CUSTOM_ADAPTOR_BEGIN

//=============== msgpack/eigen matrix conversions ================
using EigenMatrixShape_t = std::tuple<size_t, size_t>;

template<typename T, int NROW, int NCOL>
struct EigenMatrix_object_with_zone {
	using Matrix_t = _NS_UTILITY::MATRIX_xt<T, NROW, NCOL>;
	void operator()(msgobj::with_zone& o, Matrix_t const& v) const {
		o.type = type::ARRAY;
		o.via.array.size = 2;
		o.via.array.ptr = (msgobj*)o.zone.allocate_align(
			sizeof(msgobj) * o.via.array.size
		);

		std::vector<T> data(v.data(), v.data() + v.size());
		EigenMatrixShape_t shape{ (size_t)v.rows(), (size_t)v.cols() };
		o.via.array.ptr[0] = msgobj(data, o.zone);
		o.via.array.ptr[1] = msgobj(shape, o.zone);
	}
};

template<typename T, int NROW, int NCOL>
struct EigenMatrix_convert {
	using Matrix_t = _NS_UTILITY::MATRIX_xt<T, NROW, NCOL>;
	msgobj const& operator()(msgobj const& o, Matrix_t& v) {
		if (o.type != type::ARRAY) throw type_error(); //must be a list of msgobj
		if (o.via.array.size != 2) throw type_error(); //must have exactly 2 msgobj, one for content, one for shape

		//read data
		auto content = o.via.array.ptr[0].as<std::vector<T>>();
		auto shape = o.via.array.ptr[1].as<EigenMatrixShape_t>();

		//output
		v = Eigen::Map<Matrix_t>(content.data(), std::get<0>(shape), std::get<1>(shape));

		return o;
	}
};

template<typename T, int NROW, int NCOL>
struct EigenMatrix_pack {
	using Matrix_t = _NS_UTILITY::MATRIX_xt<T, NROW, NCOL>;
	template<typename Stream>
	packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, Matrix_t const& v) const {
		//pack class members to stream
		o.pack_array(2);
		std::vector<T> content(v.data(), v.data() + v.size());
		EigenMatrixShape_t shape{ v.rows(), v.cols() };
		o.pack(content);
		o.pack(shape);
		return o;
	}
};

// dynamic size matrix conversions
template<typename T, int NROW, int NCOL>
struct object_with_zone<_NS_UTILITY::MATRIX_xt<T, NROW, NCOL>> :public EigenMatrix_object_with_zone<T, NROW, NCOL> {};

template<typename T, int NROW, int NCOL>
struct convert<_NS_UTILITY::MATRIX_xt<T, NROW, NCOL>> :public EigenMatrix_convert<T, NROW, NCOL> {};

template<typename T, int NROW, int NCOL>
struct pack<_NS_UTILITY::MATRIX_xt<T, NROW, NCOL>> :public EigenMatrix_pack<T, NROW, NCOL> {};

//=============== msgpack/eigen vector conversions ================
using EigenVectorShape_t = size_t;

template<typename T, int NELEM>
struct EigenVector_object_with_zone {
	using Vector_t = _NS_UTILITY::VECTOR_xt<T, NELEM>;
	void operator()(msgobj::with_zone& o, Vector_t const& v) const {
		o.type = type::ARRAY;
		o.via.array.size = 2;
		o.via.array.ptr = (msgobj*)o.zone.allocate_align(
			sizeof(msgobj) * o.via.array.size
		);

		std::vector<T> data(v.data(), v.data() + v.size());
		EigenVectorShape_t shape = v.size();
		o.via.array.ptr[0] = msgobj(data, o.zone);
		o.via.array.ptr[1] = msgobj(shape, o.zone);
	}
};

template<typename T, int NELEM>
struct EigenVector_convert {
	using Vector_t = _NS_UTILITY::VECTOR_xt<T, NELEM>;
	msgobj const& operator()(msgobj const& o, Vector_t& v) {
		if (o.type != type::ARRAY) throw type_error(); //must be a list of msgobj
		if (o.via.array.size != 2) throw type_error(); //must have exactly 2 msgobj, one for content, one for shape

		//read data
		auto content = o.via.array.ptr[0].as<std::vector<T>>();
		auto shape = o.via.array.ptr[1].as<EigenVectorShape_t>();

		//output
		v = Eigen::Map<Vector_t>(content.data(), shape);

		return o;
	}
};

template<typename T, int NELEM>
struct EigenVector_pack {
	using Vector_t = _NS_UTILITY::VECTOR_xt<T, NELEM>;
	template<typename Stream>
	packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, Vector_t const& v) const {
		//pack class members to stream
		o.pack_array(2);
		std::vector<T> content(v.data(), v.data() + v.size());
		EigenVectorShape_t shape = v.size();
		o.pack(content);
		o.pack(shape);
		return o;
	}
};

template<typename T, int NELEM>
struct object_with_zone<_NS_UTILITY::VECTOR_xt<T, NELEM>> :public EigenVector_object_with_zone<T, NELEM> {};

template<typename T, int NELEM>
struct convert<_NS_UTILITY::VECTOR_xt<T, NELEM>> :public EigenVector_convert<T, NELEM> {};

template<typename T, int NELEM>
struct pack<_NS_UTILITY::VECTOR_xt<T, NELEM>> :public EigenVector_pack<T, NELEM> {};

_MSGPK_CUSTOM_ADAPTOR_END