#pragma once

#include <algorithm>
#include <vector>
#include <cmath>
#include <igcclib/igcclib_master.hpp>

namespace _NS_UTILITY {

	/// <summary>
	/// compute the percentile of a series of values. Let p is the percent, say 0.4, 
	/// then this function returns a value t such that t is no less than 40 percent of all values
	/// of the data.
	/// </summary>
	/// <param name="data">the data</param>
	/// <param name="n">the number of values in data</param>
	/// <param name="percent_01">a value between 0 and 1, the returned value will be 
	/// no less than percent_01 portion of the data</param>
	/// <returns>the percentile</returns>
	template<typename T>
	T percentile(const T* data, size_t n, double percent_01) {
		std::vector<T> tmp(data, data + n);
		std::sort(tmp.begin(), tmp.end());
		size_t idx = (size_t)(percent_01 * (n - 1));
		return tmp[idx];
	}

	template<typename T>
	double mean(const T* data, size_t n) {
		double x = 0;
		for (size_t i = 0; i < n; i++)
			x += data[i];
		return x / n;
	}

	template<typename T>
	double standard_deviation(const T* data, size_t n) {
		auto x_mean = mean(data, n);
		double s = 0;
		for (size_t i = 0; i < n; i++) {
			s += (data[i] - x_mean) * (data[i] - x_mean) / (n - 1);
		}
		s = std::sqrt(s);
		return s;
	}
};