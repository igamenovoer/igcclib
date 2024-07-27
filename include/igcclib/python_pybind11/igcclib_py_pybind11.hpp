#include <igccplib/core/igcclib_eigen_def.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace _NS_UTILITY
{
namespace py = pybind11;

/**
 * @brief Convert eigen matrix to numpy array, only supports 1d and 2d.
 * 
 * @tparam T data type of the eigen matrix
 * @tparam N_ROW number of rows of the eigen matrix, can be Eigen::Dynamic
 * @tparam N_COL number of cols of the eigen matrix, can be Eigen::Dynamic
 * @param x input eigen matrix
 * @param out output numpy array, will have type T and shape (N_ROW, N_COL)
 */
template <typename T, int N_ROW, int N_COL>
IGCCLIB_API void to_numpy_array(const Eigen::Matrix<T, N_ROW, N_COL, Eigen::RowMajor>& x, py::array_t<T>& out){
	using M = Eigen::Matrix<T, N_ROW, N_COL, Eigen::RowMajor>;
	out.resize({x.rows(), x.cols()});
	auto buf = out.request(true);
	assert_throw(buf.size == x.rows() * x.cols(), "size mismatch");
	Eigen::Map<M>((T*)buf.ptr, x.rows(), x.cols()) = x;
}

/**
* @brief Convert numpy array to eigen matrix, only supports 1d and 2d.
1d array will be converted to a row vector.
*
* @tparam T datatype of the numpy array
* @param x the numpy array
* @param out output eigen matrix
*/
template <typename T, int N_ROW, int N_COL>
IGCCLIB_API void to_matrix(py::array_t<T> x, Eigen::Matrix<T, N_ROW, N_COL, Eigen::RowMajor> &out)
{
    using M = Eigen::Matrix<T, N_ROW, N_COL, Eigen::RowMajor>;

    // size check and allocate
    if (N_ROW != Eigen::Dynamic && N_COL != Eigen::Dynamic)
    {
        // fixed size matrix, must have exactly the same number of elements
        assert_throw(x.size() == N_ROW * N_COL, "size mismatch");
        out = Eigen::Map<M>(x.mutable_data(), N_ROW, N_COL);
    }
    else if (N_ROW != Eigen::Dynamic)
    {
        // fixed row size, must have a multiple of N_ROW elements
        assert_throw(x.size() % N_ROW == 0, "size mismatch");
        int n_col = x.size() / N_ROW;
        out.resize(N_ROW, n_col);
        out = Eigen::Map<M>(x.mutable_data(), N_ROW, n_col);
    }
    else if (N_COL != Eigen::Dynamic)
    {
        // fixed col size, must have a multiple of N_COL elements
        assert_throw(x.size() % N_COL == 0, "size mismatch");
        int n_row = x.size() / N_COL;
        out = Eigen::Map<M>(x.mutable_data(), n_row, N_COL);
    }
    else
    {
        // fully dynamic, follow the shape of the input array
        if (x.ndim() == 1)
        {
            out = Eigen::Map<M>(x.mutable_data(), 1, x.size());
        }
        else if (x.ndim() == 2)
        {
            out = Eigen::Map<M>(x.mutable_data(), x.shape()[0], x.shape()[1]);
        }
        else
        {
            assert_throw(false, "input must be 1d or 2d");
        }
    }
}
} // namespace _NS_UTILITY