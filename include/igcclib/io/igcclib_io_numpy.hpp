#pragma once
#include <igcclib/core/igcclib_eigen_def.hpp>
#include <igcclib/extern/npy.hpp>
#include <vector>
#include <string>

namespace _NS_UTILITY {
	template<typename T, int NROW, int NCOL, int ORDER>
	inline void read_np_array(std::string filename, Eigen::Matrix<T, NROW, NCOL, ORDER>& output);

	template<typename T>
	inline void read_np_array(std::string filename, ImageRGBA_t<T>& output);

	template<typename T, int NROW, int NCOL, int ORDER>
	inline void save_np_array(const std::string& filename, const Eigen::Matrix<T, NROW, NCOL, ORDER>& mat);

	template<typename T, int NROW, int NCOL, int ORDER>
	inline void save_np_array(const std::string& filename, const std::vector < Eigen::Matrix<T, NROW, NCOL, ORDER> >& matlist);

	template<typename T>
	inline void save_np_array(const std::string& filename, const std::vector<size_t>& shape, const T* data);

	template<typename T>
	inline void save_np_array(const std::string& filename, const std::vector<T>& data);
}

namespace _NS_UTILITY {
	template<typename T, int NROW, int NCOL, int ORDER>
	void save_np_array(const std::string& filename, const Eigen::Matrix<T, NROW, NCOL, ORDER>& mat)
	{
		using MAT_TYPE = Eigen::Matrix<T, NROW, NCOL, ORDER>;
		unsigned long shape[] = { (unsigned long)mat.rows(), (unsigned long)mat.cols() };
		std::vector<T> data(mat.data(), mat.data() + mat.rows() * mat.cols());
		npy::SaveArrayAsNumpy(filename, false, 2, shape, data);
	}

	template<typename T, int NROW, int NCOL, int ORDER>
	void save_np_array(const std::string& filename, const std::vector < Eigen::Matrix<T, NROW, NCOL, ORDER> >& matlist)
	{
		using MAT_TYPE = Eigen::Matrix<T, NROW, NCOL, ORDER>;

		Eigen::Index n_elem = matlist.size() > 0 ? matlist[0].size() : 0;
		assert_throw(n_elem > 0, "trying to write a matrix with no element");

		//check if all matrices are of the same size
		for (const auto& m : matlist)
			assert_throw(m.size() == n_elem, "some matrix does not have the same size as others");

		auto nrow = matlist[0].rows();
		auto ncol = matlist[0].cols();
		unsigned long shape[] = { (unsigned long)matlist.size(), (unsigned long)nrow, (unsigned long)ncol };

		std::vector<T> data;
		data.reserve(n_elem * matlist.size());
		for (const auto& m : matlist)
			data.insert(data.end(), m.data(), m.data() + n_elem);

		npy::SaveArrayAsNumpy(filename, false, sizeof(shape)/sizeof(unsigned long), shape, data);
	}

	template<typename T>
	void save_np_array(const std::string& filename, const std::vector<size_t>& shape, const T* data)
	{
		size_t n_elem = 1;
		for (auto x : shape)
			n_elem *= x;
		std::vector<T> _data(data, data + n_elem);
		std::vector<unsigned long> _shape;
		for (auto x : shape)
			_shape.push_back((unsigned long)x);
		npy::SaveArrayAsNumpy(filename, false, (unsigned int)shape.size(), _shape.data(), _data);
	}

	template<typename T>
	void save_np_array(const std::string& filename, const std::vector<size_t>& shape, const std::vector<T>& data)
	{
		size_t n_elem = 1;
		for (auto x : shape)
			n_elem *= x;		
		std::vector<unsigned long> _shape;
		for (auto x : shape)
			_shape.push_back((unsigned long)x);
		npy::SaveArrayAsNumpy(filename, false, (unsigned int)shape.size(), _shape.data(), data);
	}

	template<typename T>
	inline void save_np_array(const std::string& filename, const std::vector<T>& data)
	{
		auto n_elem = data.size();
		std::vector<size_t> shape = { n_elem };
		save_np_array(filename, shape, data);
	}

	template<typename T, int NROW, int NCOL, int ORDER>
	void read_np_array(std::string filename, Eigen::Matrix<T, NROW, NCOL, ORDER>& output) {
		using MAT_TYPE = Eigen::Matrix<T, NROW, NCOL, ORDER>;
		std::vector<unsigned long> shape;
		std::vector<T> data;
		npy::LoadArrayFromNumpy(filename, shape, data);

		assert(shape.size() < 3);

		if (shape.size() == 0) //scalar
		{
			output.resize(1, 1);
			output << data[0];
		}
		else if (shape.size() == 1) //vector
		{
			output = Eigen::Map<MAT_TYPE>(data.data(), shape[0], 1);
		}
		else if (shape.size() == 2) //2d matrix
		{
			output = Eigen::Map<MAT_TYPE>(data.data(), shape[0], shape[1]);
		}
	}

	template<typename T>
	void read_np_array(std::string filename, ImageRGBA_t<T>& output) {
		std::vector<unsigned long> shape;
		std::vector<T> data;
		npy::LoadArrayFromNumpy(filename, shape, data);

		assert(shape.size() >= 2);
		int height = shape[0];
		int width = shape[1];
		int channel = shape.size() > 2 ? shape[2] : 1;

		output.allocate(width, height, channel);
		for (size_t i = 0; i < data.size(); i += channel) {
			for (int k = 0; k < channel; k++) {
				output.get_channel(k).data()[i / channel] = data[i + k];
			}
		}
	}
}