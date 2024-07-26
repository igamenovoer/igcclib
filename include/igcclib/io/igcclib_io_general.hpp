#pragma once
#include <igcclib/igcclib_master.hpp>

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <cereal/archives/json.hpp>

namespace _NS_UTILITY
{
	/// <summary>
	/// read the entire content of the file into a binary vector
	/// </summary>
	/// <param name="filename">the filename</param>
	/// <param name="output">output storage</param>
	inline void read_file_as_binary(const std::string& filename, std::vector<uint8_t>& output)
	{
		std::ifstream infile(filename, std::ios::binary | std::ios::ate);
		auto filesize = infile.tellg();

		infile.seekg(0, infile.beg);
		output.resize(filesize);
		infile.read((char*)&output[0], filesize);

		infile.close();
	}

	/// <summary>
	/// read the entire content of the file into a binary vector
	/// </summary>
	/// <param name="filename">the filename</param>
	/// <param name="output">output storage</param>
	inline void read_file_as_binary(const std::string& filename, std::vector<char>& output)
	{
		std::ifstream infile(filename, std::ios::binary | std::ios::ate);
		auto filesize = infile.tellg();

		infile.seekg(0, infile.beg);
		output.resize(filesize);
		infile.read((char*)&output[0], filesize);

		infile.close();
	}

	/**
	* \brief read the file as a list of strings, each for a line
	*
	* \param filename the text file
	* \return std::vector<std::string> the list of strings
	*/
	inline std::vector<std::string> read_file_as_multi_string(const std::string& filename) {
		std::ifstream infile(filename);
		std::vector<std::string> output;
		std::string tmp;
		while (std::getline(infile, tmp)) {
			output.push_back(tmp);
		}
		return output;
	}

	/**
	* \brief read the entire file as a single concatenated string
	*
	* \param filename the text file name
	* \return std::string the whole file read into a single string
	*/
	inline std::string read_file_as_single_string(const std::string& filename) {
		std::ifstream infile(filename);
		std::ostringstream os;
		os << infile.rdbuf();
		return os.str();
	}

	template<typename T>
	void save_binary_by_cereal(const std::string& filename, const T& data)
	{
		std::ofstream outfile(filename, std::ios::trunc | std::ios::binary);
		cereal::PortableBinaryOutputArchive oa(outfile);
		oa(data);
	}

	template<typename T>
	void load_binary_by_cereal(const std::string& filename, T* output)
	{
		std::ifstream infile(filename, std::ios::binary);
		cereal::PortableBinaryInputArchive oi(infile);
		oi(*output);
	}

	template<typename T>
	void save_json_by_cereal(const std::string& filename, const T& data)
	{
		std::ofstream outfile(filename);
		cereal::JSONOutputArchive oa(outfile);
		oa(data);
	}

	template<typename T>
	void load_json_by_cereal(const std::string& filename, T* output)
	{
		std::ifstream infile(filename);
		cereal::JSONInputArchive oi(infile);
		oi(*output);
	}
};