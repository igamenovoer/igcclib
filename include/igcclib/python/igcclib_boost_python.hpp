#pragma once

#include <igcclib/igcclib_master.hpp>
#include <boost/python.hpp>

#if BOOST_VERSION / 100 % 1000 >= 64
#include <boost/python/numpy.hpp>
#else
#include <boost/numpy.hpp>
#endif

namespace _NS_UTILITY
{
#if BOOST_VERSION / 100 % 1000 >= 64
	namespace np = boost::python::numpy;
#else
	namespace np = boost::numpy
#endif
	typedef np::ndarray NDARRAY;
	namespace bp = boost::python;
}