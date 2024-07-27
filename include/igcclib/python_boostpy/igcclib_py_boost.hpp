#pragma once
#include <igcclib/python_boostpy/igcclib_py_boost_def.hpp>

namespace _NS_UTILITY
{
	inline void assert_python(bool tf, std::string msg /*= "c++ error occurred"*/)
	{
		if (!tf)
		{
			std::cout << msg << std::endl << std::endl;
			boost::python::throw_error_already_set();
		}
	}

	inline uint64_t np_size(const np::ndarray& arr)
	{
		int nd = arr.get_nd();
		uint64_t n_elem = 1;
		for (int i = 0; i < nd; i++)
		{
			n_elem *= arr.shape(i);
		}
		return n_elem;
	}

	//python interop
	//get numpy array from object
	inline np::ndarray get_ndarray(const boost::python::object& obj)
	{
		np::ndarray arr = bp::extract<np::ndarray>(obj);
		return arr;
	}

	//get numpy array with type conversion if needed
	//return the ndarray that has the data type of T
	//if the original array is already type-satisified, return it without copying
	//if force_contiguous = true, then the array will be copied if it is not C_CONTIGUOUS
	template<typename T>
	IGCCLIB_API NDARRAY get_ndarray_as(const NDARRAY& obj, bool force_contiguous = true)
	{
		auto dtype = np::dtype::get_builtin<T>();
		bool should_copy = obj.get_dtype() != dtype || (force_contiguous && !(obj.get_flags() & NDARRAY::C_CONTIGUOUS));

		if (!should_copy)
			return obj;
		else
			return obj.astype(dtype);
	}

	//get numpy array with type conversion
	//return the type converted ndarray
	//if the original array is already type-satisified, return it without copying
	//if force_contiguous = true, then the array will be copied if it is not C_CONTIGUOUS
	template<typename T>
	IGCCLIB_API NDARRAY get_ndarray_as(const boost::python::object& obj, bool force_contiguous = true)
	{
		auto arr = get_ndarray(obj);
		auto dtype = np::dtype::get_builtin<T>();
		bool should_copy = arr.get_dtype() != dtype || (force_contiguous && !(arr.get_flags() & NDARRAY::C_CONTIGUOUS));

		if (!should_copy)
			return arr;
		else return arr.astype(dtype);
	}

	//get ndarray data with checking
	//if checking failed, throws exception, return NULL
	template<typename T>
	IGCCLIB_API T* get_ndarray_data(NDARRAY& obj, bool check_contiguous = true)
	{
		if (check_contiguous)
			assert_python(obj.get_flags() & NDARRAY::C_CONTIGUOUS, "memory of numpy array should be c-contiguous");
		return reinterpret_cast<T*>(obj.get_data());
	}

	template<typename T>
	IGCCLIB_API const T* get_ndarray_data(const NDARRAY& obj, bool check_contiguous = true)
	{
		if (check_contiguous)
			assert_python(obj.get_flags() & NDARRAY::C_CONTIGUOUS, "memory of numpy array should be c-contiguous");
		return reinterpret_cast<T*>(obj.get_data());
	}

	template <class T>
	IGCCLIB_API boost::python::list to_python_list(std::vector<T> vector) {
		typename std::vector<T>::iterator iter;
		boost::python::list list;
		for (iter = vector.begin(); iter != vector.end(); ++iter) {
			list.append(*iter);
		}
		return list;
	}

	//convert index list of 1d numpy array
	template<typename T>
	IGCCLIB_API np::ndarray to_ndarray(const std::vector<T>& idxlist)
	{
		auto dtype = np::dtype::get_builtin<T>();
		bp::tuple shape = bp::make_tuple(idxlist.size());
		np::ndarray arr_out = np::zeros(shape, dtype);

		//fill in data
		T* data = reinterpret_cast<T*>(arr_out.get_data());
		for (size_t i = 0; i < idxlist.size(); i++)
		{
			data[i] = idxlist[i];
		}

		//done
		return arr_out;
	}
}
