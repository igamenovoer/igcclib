#pragma once

#include <vector>
#include <algorithm>
#include <iostream>
#include <random>
#include <set>

#include <igcclib/igcclib_master.hpp>
#include <igcclib/extern/naturalorder.h>

namespace _NS_UTILITY
{
	//sort string in natural order
	inline void sort_string_by_natural_order(std::vector<std::string>& strlist);

	//split the range of integers [min,max] into n groups,
	//the range of i-th group is [output[i].first, output[i].second]
	inline std::vector< std::pair<int, int> > split_indices_by_n(int min, int max, int n);

	/// <summary>
	/// sort the data in ascending or descending order, return the sorted index
	/// </summary>
	template<typename T>
	std::vector<size_t> sort_index(const std::vector<T>& data,
		bool is_ascending = true, bool is_stable = false);

	/// <summary>
	/// find unique elements in an array
	/// </summary>
	/// <param name="data">the data to the processed</param>
	/// <param name="out_index">indices into the data where the unique elements are, such that out_unique = data[out_index]</param>
	/// <param name="out_inverse_index">indices into the unique elements that can recover the data, 
	/// such that data=out_unique[out_reverse_index]</param>
	/// <returns>std::vector of the unique elements</returns>
	template<typename T>
	std::vector<T> unique_elements(const std::vector<T>& data,
		std::vector<size_t>* out_index = 0,
		std::vector<size_t>* out_inverse_index = 0);

	template<typename ITER>
	void print_container(ITER it_begin, ITER it_end, int n_per_line = -1);

	template<typename CONTAINER>
	void print_container(const CONTAINER& data, int n_per_line = -1);

	/// <summary>
	/// get elements by indexing into an array, such that output[i]=data[*(it_idx_begin+i)]
	/// </summary>
	/// <param name="data"> the data array</param>
	/// <param name="it_idx_begin">iterator into the begin of the index array</param>
	/// <param name="it_idx_end">iterator into one past the end of the index array</param>
	/// <returns>the retrieved array such that output[i]=data[*(it_idx_begin+i)]</returns>
	template<typename T, typename ITER>
	std::vector<T> get_element_by_index(const std::vector<T>& data, ITER it_idx_begin, ITER it_idx_end);

	template<typename T>
	inline std::vector<T> linspace(T first, T last, int len);

	/** \brief number of elements in a multi dimentional grid */
	inline int get_grid_sizes_num(const std::vector<int>& m_grid_size);

	/// <summary>
	/// randomly select n elements from 0...upper_bound-1, with or without replacement
	/// </summary>
	/// <param name="output">the randomly selected indices</param>
	/// <param name="n">the number of elements to draw</param>
	/// <param name="upper_bound">the open upper_bound of the candidate sequence, note that the max possible
	/// candidate is upper_bound-1</param>
	/// <param name="allow_duplicate">if true, the returned vector may contain duplicates</param>
	/// <returns></returns>
	inline void random_select_index(std::vector<size_t>& output, size_t n, size_t upper_bound, bool allow_duplicate = false);

	/// <summary>
	/// randomly select n elements from 0...upper_bound-1, with or without replacement
	/// </summary>
	/// <param name="n">the number of elements to draw</param>
	/// <param name="upper_bound">the open upper_bound of the candidate sequence, note that the max possible
	/// candidate is upper_bound-1</param>
	/// <param name="allow_duplicate">if true, the returned vector may contain duplicates</param>
	/// <returns></returns>
	inline std::vector<size_t> random_select_index(size_t n, size_t upper_bound, bool allow_duplicate = false);

	/**
	* \brief randomly generate a string
	*
	* \param n number of characeters in the output
	* \param candidates candidate characters, if not set, uses all number and alphabets
	* \return std::string the random string
	*/
	inline std::string random_string(int n, const std::string* candidates = nullptr);

	/** \brief given two sets a and b, return the set a-b */
	template<typename T>
	std::vector<T> compute_set_difference(const std::vector<T>& a, const std::vector<T>& b);
}

// =========================== implementation ================================
namespace _NS_UTILITY {
	std::string random_string(int n, const std::string* candidates) {
		static std::string default_candidate = "1234567890qwertyuiopasdfghjklzxcvbnmQWERTYUIOPASDFGHJKLZXCVBNM";
		if (!candidates)
			candidates = &default_candidate;

		std::default_random_engine gen;
		std::uniform_int_distribution<int> dist(0, (int)candidates->size()-1);
		std::string output(n, 'c');
		for (int i = 0; i < n; i++) {
			int idx = dist(gen);
			output[i] = candidates->at(idx);
		}
		return output;
	}

	std::vector<size_t> random_select_index(size_t n, size_t upper_bound, bool allow_duplicate)
	{
		std::vector<size_t> output;
		random_select_index(output, n, upper_bound, allow_duplicate);
		return output;
	}

	void random_select_index(std::vector<size_t>& output, size_t n, size_t upper_bound, bool allow_duplicate)
	{
		if (upper_bound == 0)
		{
			output.clear();
			return;
		}

		output.resize(n);
		if (allow_duplicate)
		{
			for (size_t i = 0; i < n; i++)
			{
				size_t x = (size_t)(std::rand() / double(RAND_MAX) * (upper_bound - 1));
				output[i] = x;
			}
		}
		else
		{
			std::vector<size_t> cand(upper_bound);
			for (size_t i = 0; i < upper_bound; i++)
				cand[i] = i;
			std::random_shuffle(cand.begin(), cand.end());
			for (size_t i = 0; i < n; i++)
			{
				output[i] = cand[i%cand.size()];
			}
		}
	}

	int get_grid_sizes_num(const std::vector<int>& m_grid_size)
	{
		int num = 1;
		for (auto it = m_grid_size.begin(); it != m_grid_size.end(); it++)
		{
			num *= (*it);
		}
		return num;
	}

	template<typename T>
	std::vector<T> linspace(T first, T last, int len) {
		std::vector<T> result(len);
		T step = (last - first) / (len - 1);
		for (int i = 0; i < len; i++) { result[i] = first + i * step; }
		return result;
	}

	template<typename T, typename ITER>
	std::vector<T> get_element_by_index(const std::vector<T>& data, ITER it_idx_begin, ITER it_idx_end)
	{
		auto n = std::distance(it_idx_begin, it_idx_end);
		std::vector<T> output(n);
		size_t k = 0;
		for (auto it = it_idx_begin; it != it_idx_end; ++it)
			output[k++] = data[*it];
		return output;
	}

	template<typename CONTAINER>
	void print_container(const CONTAINER& data, int n_per_line)
	{
		print_container(data.begin(), data.end(), n_per_line);
	}

	template<typename ITER>
	void print_container(ITER it_begin, ITER it_end, int n_per_line)
	{
		int n_print = 0;
		for (auto it = it_begin; it != it_end; ++it)
		{
			n_print++;
			std::cout << *it << ", ";

			if (n_per_line > 0 && (n_print % n_per_line == 0))
			{
				std::cout << std::endl;
			}
		}
		std::cout << std::endl;
	}

	template<typename T>
	std::vector<T> unique_elements(const std::vector<T>& data,
		std::vector<size_t>* out_index,
		std::vector<size_t>* out_inverse_index)
	{
		auto idx = sort_index(data);
		std::vector<size_t> u_index;
		std::vector<size_t> inv_index(idx.size());

		if (idx.size() > 0)
		{
			u_index.push_back(idx[0]);
			inv_index[idx[0]] = 0;

			for (size_t i = 1; i < idx.size(); i++)
			{
				auto idx_prev = idx[i - 1];
				auto idx_this = idx[i];

				//found new unique element
				if (data[idx_this] != data[idx_prev])
				{
					u_index.push_back(idx_this);
				}

				inv_index[idx_this] = u_index.size() - 1;
			}
		}

		//collect all unique elements
		std::vector<T> u_elem(u_index.size());
		for (size_t i = 0; i < u_index.size(); i++)
		{
			u_elem[i] = data[u_index[i]];
		}

		if (out_index)
			*out_index = u_index;
		if (out_inverse_index)
			*out_inverse_index = inv_index;
		return u_elem;
	}

	template<typename T>
	std::vector<size_t> sort_index(const std::vector<T>& data,
		bool is_ascending, bool is_stable)
	{
		//sort the index by data
		std::vector<size_t> idx(data.size());
		for (size_t i = 0; i < idx.size(); i++)
			idx[i] = i;

		auto cmp = [&](size_t x, size_t y)
		{
			return data[x] < data[y];
		};

		if (is_stable)
			std::stable_sort(idx.begin(), idx.end(), cmp);
		else
			std::sort(idx.begin(), idx.end(), cmp);

		if (!is_ascending)
			std::reverse(idx.begin(), idx.end());
		return idx;
	}

	std::vector< std::pair<int, int> > split_indices_by_n(int min, int max, int n)
	{
		std::vector< std::pair<int, int> > endpts;
		if (min > max)
			return endpts;
		double dx = double(max - min) / n;

		std::pair<int, int> p;
		p.first = min;
		for (int i = 1; i <= n; i++)
		{
			int e = int(min + i * dx);
			if (e < p.first) //this slot is not usable
				continue;

			//a new slot is created
			p.second = e;
			endpts.push_back(p);

			//prepare the new slot
			p.first = p.second + 1;
		}

		//done
		return endpts;
	}

	template<typename T>
	std::vector<T>
		compute_set_difference(const std::vector<T>& a, const std::vector<T>& b)
	{
		std::set<T> _a(a.begin(), a.end());
		for (const auto& x : b) {
			auto iter = _a.find(x);
			if (iter != _a.end())
				_a.erase(iter);
		}
		return std::vector<T>(_a.begin(), _a.end());
	}

	void sort_string_by_natural_order(std::vector<std::string>& strlist)
	{
		std::sort(strlist.begin(), strlist.end(), natsort::natural_less<std::string>);
	}
}