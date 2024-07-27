#pragma once
#include <igcclib/io/igcclib_io_general.hpp>

#include <string>
#include <strstream>
#include <cereal/cereal.hpp>
#include <cereal/archives/portable_binary.hpp>

//compressing content and file

namespace _NS_UTILITY
{
	/**
	* \brief	 save the data into a cereal portable binary archive in compressed format
	*
	* \param filename	the output filename
	* \param data	the data to be saved
	* \param compression_level	level of compression, ranging from 0 to 9, higher value means slower compression speed with higher compression ratio
	*/
	template<typename T>
	IGCCLIB_API void save_binary_by_cereal_compressed(const std::string& filename, const T& data, int compression_level);

	/** \brief load compressed cereal archive */
	template<typename T>
	IGCCLIB_API void load_binary_by_cereal_compressed(const std::string& filename, T* output);

	/**
	* \brief compressing a binary array
	*
	* \param output	the output data
	* \param data	the input data
	* \param n	length of the input data
	* \param compression_level	level of compression, ranging from 0 to 9, 
	higher value means slower compression speed with higher compression ratio
	*/
	IGCCLIB_API void compress_binary_data(std::vector<char>* output, const char* data, size_t n, int compression_level);

	/** \brief decompress binary data */
	IGCCLIB_API void decompress_binary_data(std::vector<char>* output, const char* data, size_t n);
}

//implementations
namespace _NS_UTILITY
{
	/**
	* \brief	 save the data into a cereal portable binary archive in compressed format
	*
	* \param filename	the output filename
	* \param data	the data to be saved
	* \param compression_level	level of compression, ranging from 0 to 9, higher value means slower compression speed with higher compression ratio
	*/
	template<typename T>
	void save_binary_by_cereal_compressed(const std::string& filename, const T& data, int compression_level)
	{
		std::string content;
		{
			std::ostringstream os;
			cereal::PortableBinaryOutputArchive oa(os);
			oa(data);
			content = os.str();
		}

		std::vector<char> compress_data;
		compress_binary_data(&compress_data, content.c_str(), content.size(), compression_level);

		std::ofstream outfile(filename, std::ios::binary | std::ios::trunc);
		outfile.write(compress_data.data(), compress_data.size());
	}

	/** \brief load compressed cereal archive */
	template<typename T>
	void load_binary_by_cereal_compressed(const std::string& filename, T* output)
	{
		if (!output) return;

		//load and decompress data
		std::vector<char> ucdata;
		{
			std::vector<char> data;
			read_file_as_binary(filename, data);
			decompress_binary_data(&ucdata, data.data(), data.size());
		}

		//deserialize
		std::strstream ss(ucdata.data(), ucdata.size());
		cereal::PortableBinaryInputArchive ia(ss);
		ia(*output);
	}
}