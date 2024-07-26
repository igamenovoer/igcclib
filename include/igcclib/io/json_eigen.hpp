#pragma once

#include "./../common/def_master.h"
#include "./../common/def_eigen.h"
#include "nlohmann/json.hpp"

namespace _NS_UTILITY {
	template<typename T, int NROW, int NCOL>
	void to_json(nlohmann::json& jdata, const MATRIX_xt<T, NROW, NCOL>& mat) 
	{
		nlohmann::json j;

		std::vector<int> shape = { (int)mat.rows(), (int)mat.cols() };
		j["shape"] = shape;

		std::vector<T> content(mat.data(), mat.data() + mat.size());
		j["content"] = content;

		jdata = j;
	}

	template<typename T, int NROW, int NCOL>
	nlohmann::json to_json(const MATRIX_xt<T, NROW, NCOL>& mat)
	{
		nlohmann::json j;
		to_json(j, mat);
		return j;
	}

	template<typename T, int NROW, int NCOL>
	void from_json(const nlohmann::json& jdata, MATRIX_xt<T, NROW, NCOL>& mat)
	{
		std::vector<int> shape;
		jdata.at("shape").get_to(shape);

		std::vector<T> content;
		jdata.at("content").get_to(content);

		mat = Eigen::Map<MATRIX_xt<T, NROW, NCOL>>(content.data(), mat.rows(), mat.cols());
	}
}