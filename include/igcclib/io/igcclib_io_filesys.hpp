#pragma once
#include <igcclib/igcclib_master.hpp>
#include <igcclib/extern/naturalorder.h>

#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <regex>

namespace _NS_UTILITY {

	/**
	* \brief get all sub directories (without recursion) in a directory
	*
	* \param dirname	the input directory
	* \param include_path	whether to return the full paths of the subdirectories
	* \param sort_by_natural_order whether to sort the dir names in natural order
	* \param regex_pattern	only find those sub directories whose names fit this regex pattern
	* \return std::vector<std::string>	a list of sub directory names
	*/
	inline std::vector<std::string> get_subdirs(const std::string& dirname,
		bool include_path = false, 
		bool sort_by_natural_order = false,
		boost::optional<const char*> regex_pattern = boost::none)
	{
		namespace bfs = boost::filesystem;
		std::shared_ptr<std::regex> ptn;
		if (regex_pattern)
			ptn = std::make_shared<std::regex>(*regex_pattern);

		std::vector<std::string> output;
		for (bfs::directory_iterator it(dirname); it != bfs::directory_iterator(); ++it) {
			if (!bfs::is_directory(it->path())) continue;

			if (ptn)
			{
				std::smatch m;
				auto c = it->path().filename().string();
				std::regex_match(c, m, *ptn);
				if (m.empty())
					continue;
			}

			std::string fn;
			if (include_path)
				fn = it->path().string();
			else
				fn = it->path().filename().string();
			output.push_back(fn);
		}

		if (sort_by_natural_order)
			std::sort(output.begin(), output.end(), natsort::natural_less<std::string>);

		return output;
	}

	/// <summary>
	/// get all files in a directory
	/// </summary>
	/// <param name="dirname">the directory</param>
	/// <param name="include_path">whether to include full path in the output</param>
	/// <param name="_ext">file extension, with or without dot. 
	/// If none, file extension will not be considered</param>
	/// <returns>the files in the directory</returns>
	inline std::vector<std::string> get_files_from_dir(std::string dirname, 
		bool include_path = false, 
		bool sort_by_natural_order = false,
		boost::optional<const char*> _ext = boost::none,
		boost::optional<const char*> regex_pattern = boost::none) {

		namespace bfs = boost::filesystem;
		std::string ext = _ext ? *_ext : "";
		if (ext.size() >= 1 && ext[0] != '.') //add .
			ext = "." + ext;

		std::shared_ptr<std::regex> ptn;
		if (regex_pattern)
			ptn = std::make_shared<std::regex>(*regex_pattern);

		std::vector<std::string> output;
		for (bfs::directory_iterator it(dirname); it != bfs::directory_iterator(); ++it) {
			if (!bfs::is_regular_file(it->path())) continue;
			if (!ext.empty() && it->path().extension() != ext) continue;

			if (ptn)
			{
				std::smatch m;
				auto c = it->path().filename().string();
				std::regex_match(c, m, *ptn);
				if(m.empty())
					continue;
			}

			std::string fn;
			if (include_path)
				fn = it->path().string();
			else
				fn = it->path().filename().string();
			output.push_back(fn);
		}

		if (sort_by_natural_order)
			std::sort(output.begin(), output.end(), natsort::natural_less<std::string>);

		return output;
	}

	inline void make_dir(const std::string& dirname) {
		namespace bfs = boost::filesystem;

		//check if that directory exists
		if (!bfs::exists(dirname)) {
			bfs::create_directories(dirname);
		}
	}

	/// <summary>
	/// split file name into path, filename and extension
	/// </summary>
	/// <param name="filename">the filename to be splitted ext_by_first_dot< / param>
	/// <param name="path">file path</param>
	/// <param name="name">file name without extension</param>
	/// <param name="ext">extention with '.'</param>	
	inline void fileparts(const std::string& filename, std::string& path, std::string& name, std::string& ext)
	{
		namespace bfs = boost::filesystem;

		bfs::path p = filename;
		path = p.parent_path().string();
		name = p.stem().filename().string();
		ext = p.extension().string();
	}

	/** \brief delete all files and sub folders in a directory */
	inline void remove_all_in_dir(const std::string& dirname) {
		namespace fs = boost::filesystem;
		fs::path target(dirname);
		if (fs::exists(target))
		{
			for (fs::directory_iterator it_end, it(target); it != it_end; ++it) {
				fs::remove_all(it->path());
			}
		}
	}
};