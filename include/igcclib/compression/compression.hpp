#pragma once

#include <igcclib/igcclib_master.hpp>
#include <vector>

namespace _NS_UTILITY {
	/**
	* \brief compress data
	*
	* \param in_data data to be compressed
	* \param n length of in_data
	* \return std::vector<uint8_t> the compressed data
	*/
	IGCCLIB_API std::vector<uint8_t> compress_data(const uint8_t* in_data, size_t n);

	/**
	* \brief decompress data, which has been compressed using compress_data()
	*
	* \param in_data the compressed data
	* \param n_input length of the compressed data
	* \param n_output upper bound of the length of the decompressed data
	* \return std::vector<uint8_t> the uncompressed data
	*/
	IGCCLIB_API std::vector<uint8_t> decompress_data(const uint8_t* in_data, size_t n_input, size_t n_output);
};