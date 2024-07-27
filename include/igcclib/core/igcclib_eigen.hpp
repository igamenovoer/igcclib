#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <set>

#include <igcclib/core/igcclib_eigen_def.hpp>

namespace _NS_UTILITY
{
	// =============== declarations ======================
	template<typename T>
	size_t count_nonzeros(const MATRIX_t<T>& mat);

	/** \brief find the nonzero elements in vec, return their indices in output_indices */
	template<typename T, typename D>
	void find_nonzeros(const VECTOR_t<T>& vec, std::vector<D>* output_indices);

	/** \brief find the nonzero elements in vec, return their indices in output_indices */
	template<typename T, typename D>
	void find_nonzeros(const VECTOR_t<T>& vec, VECTOR_t<D>* output_indices);

	/** \brief find the nonzero elements in a matrix, return their locations (i,j) in output_ij */
	template<typename T, typename D>
	void find_nonzeros(const MATRIX_t<T>& mat, iMATRIX* output_ij);

	template<typename MAT_1, typename MAT_2>
	bool is_same_size(const MAT_1& x, const MAT_2& y);

	//read/write eigen matrix
	template <typename MATRIX>
	void load_matrix(std::string filename, MATRIX& output);

	template <typename MATRIX>
	void save_matrix(std::string filename, const MATRIX& input);

	template<typename T>
	void save_sparse_matrix(std::string filename, const SP_MATRIX_t<T>& spmat);

	template<typename T, typename VEC_TYPE>
	void to_vector(const std::vector<T>& input, VEC_TYPE* output);

	template<typename T>
	VECTOR_t<T> to_vector(const std::vector<T>& input);

	/** \brief convert a list of points in matrix form to vector form */
	template<typename T, int NDIM = Eigen::Dynamic>
	void to_point_list(const MATRIX_t<T>& pts, std::vector<VECTOR_xt<T, NDIM>>* output);

	/** \brief convert a list of points in vector form to matrix form */
	template<typename T, int NDIM = Eigen::Dynamic>
	void to_point_matrix(const std::vector < VECTOR_xt<T, NDIM> >& pts, MATRIX_t<T>* output);

	//parse a string containing multiple numbers into a vector
	//the numbers are delimited by any character contained in delimiters
	//NOTE: delimiters contains multiple delimiters where delimiters[i] is the i-th delimiter
	template<typename T>
	void parse_as_vector(std::string content, VECTOR_t<T>* output, std::string delimiters = " ");

	template<typename T, int OPT>
	Eigen::SparseMatrix<T, OPT> get_sub_matrix_by_rows(const Eigen::SparseMatrix<T, OPT>& mat, const std::set<Eigen::Index>& idxrow);

	//get submatrix by rows and cols
	template<typename MAT_TYPE, typename IDXLIST_T_1, typename IDXLIST_T_2> //use generic matrix type because Eigen matrix may have fixed size
	void get_sub_matrix(const MAT_TYPE& mat, const IDXLIST_T_1& idxrow, const IDXLIST_T_2& idxcol, MAT_TYPE& output);

	//get submatrix by rows and cols
	template<typename MAT_TYPE, typename IDXLIST_T_1, typename IDXLIST_T_2>
	MAT_TYPE get_sub_matrix(const MAT_TYPE& mat, const IDXLIST_T_1& idxrow, const IDXLIST_T_2& idxcol);

	//get submatrix with multiple rows and a single column, if idxcol=-1, then all columns are retrieved
	template<typename MAT_TYPE, typename IDX_LIST_TYPE>
	void get_sub_matrix(const MAT_TYPE& mat, const IDX_LIST_TYPE& idxrow, int idxcol, MAT_TYPE& output);

	//get submatrix with multiple rows and a single column, if idxcol=-1, then all columns are retrieved
	template<typename MAT_TYPE, typename IDX_LIST_TYPE>
	MAT_TYPE get_sub_matrix(const MAT_TYPE& mat, const IDX_LIST_TYPE& idxrow, int idxcol);

	/// <summary>
	/// access elements by mask, just like x[mask] in numpy
	/// </summary>
	/// <param name="mat">the data matrix</param>
	/// <param name="mask">the mask matrix, where data(i,j) will be read if mask(i,j)!=0 </param>
	/// <returns>the data in mat covered by mask, 
	/// the order is in row-major order of mask</returns>
	template<typename T, typename D>
	VECTOR_t<T> get_values_by_mask(const MATRIX_t<T>& mat, const MATRIX_t<D>& mask);

	/// <summary>
	/// like numpy x[mask]=y, assign values to matrix based on non zero values in mask
	/// </summary>
	/// <param name="mat">the matrix to be modified</param>
	/// <param name="mask">the mask</param>
	/// <param name="values">the values</param>
	template<typename T, typename D>
	void set_values_by_mask(MATRIX_t<T>& mat,
		const MATRIX_t<D>& mask,
		const VECTOR_t<T>& values);

	/// <summary>
	/// like numpy x[mask]=y, assign values to matrix based on non zero values in mask
	/// </summary>
	/// <param name="mat">the matrix to be modified</param>
	/// <param name="mask">the mask</param>
	/// <param name="value">the specified value</param>
	template<typename T, typename D>
	void set_values_by_mask(MATRIX_t<T>& mat,
		const MATRIX_t<D>& mask,
		double value);

	/// <summary>
	/// compute pca decomposition of points using svd
	/// </summary>
	/// <param name="pts">the input points</param>
	/// <param name="out_components">the output basis, each row is a basis, sorted by variance from large to small.
	/// Note that for 3D points, these basis might be right or left handed, you need to use det to make adjustment.
	/// Setting to nullptr will ignore this output.
	/// </param>
	/// <param name="out_variances">the variance of each basis, setting it to nullptr will ignore this output</param>
	template<typename MAT_TYPE_IN, typename MAT_TYPE_OUT, typename VEC_TYPE>
	void pca_by_svd(const MAT_TYPE_IN& pts, MAT_TYPE_OUT* out_components, VEC_TYPE* out_variances);

	/// <summary>
	/// stack a list of ROW vectors into a matrix, along vertical direction.
	/// </summary>
	/// <param name="vecs">the list of vectors</param>
	/// <param name="output">the output matrix</param>
	template<typename T, int D>
	void row_stack(const std::vector<VECTOR_xt<T, D>>& vecs, MATRIX_t<T>& output);	

	/// <summary>
	/// stack a list of ROW vectors into a matrix, along vertical direction
	/// </summary>
	/// <param name="vecs">the list of vectors</param>
	/// <param name="output">the output matrix</param>
	template<typename T>
	void row_stack(const std::vector<VECTOR_t<T>>& vecs, MATRIX_t<T>& output);

	/** \brief concatenate a list of vectors into a new vector */
	template<typename T>
	void concatenate(const std::vector<const VECTOR_t<T>*>& vecs, VECTOR_t<T>& output);

	template<typename T>
	void row_stack(const std::vector<const MATRIX_t<T>*>& matlist, MATRIX_t<T>& output);

	template<typename T>
	void row_stack(const std::vector<MATRIX_t<T>>& matlist, MATRIX_t<T>& output);

	template<typename T>
	void row_stack(const std::vector<SP_MATRIX_t<T>>& matlist, SP_MATRIX_t<T>& output);

	template<typename T>
	void row_stack(const std::vector<const SP_MATRIX_t<T>*>& matlist, SP_MATRIX_t<T>& output);

	template<typename T, int NROW, int NCOL>
	void column_stack(const std::vector< MATRIX_xt<T, NROW, NCOL> >& matlist, MATRIX_t<T>& output);

	template<typename T, int D>
	void column_stack(const std::vector< VECTOR_xt<T, D> >& veclist, MATRIX_t<T>& output);

	/// <summary>
	/// like numpy img[mask] = valmat. If img has N channels, then valmat has N cols, each row 
	/// of valmat is the color of a channel. The order of mask is row-major.
	/// </summary>
	/// <param name="img">the image to be modified</param>
	/// <param name="mask">the mask where nonzeros defines locations to modify</param>
	/// <param name="valmat">the colors, each row is a color.</param>
	template<typename T, typename U, typename R>
	void set_values_by_mask(ImageRGBA_t<T>& img, const MATRIX_t<U>& mask, const MATRIX_t<R>& valmat);

	/// <summary>
	/// like numpy v = img[mask]
	/// </summary>
	/// <param name="img">the image</param>
	/// <param name="mask">the reading mask</param>
	/// <returns>the colors, each row is a color</returns>
	template<typename T, typename U>
	MATRIX_t<T> get_values_by_mask(const ImageRGBA_t<T>& img, const MATRIX_t<U>& mask);

	template<typename T, int NROW, int NCOL>
	inline MATRIX_t<T> add_column(const MATRIX_xt<T, NROW, NCOL>& mat, T val);


	// =============== implementations =======================
	template<typename T, int NDIM>
	void to_point_list(const MATRIX_t<T>& pts, std::vector<VECTOR_xt<T, NDIM>>* output)
	{
		if (!output) return;
		output->resize(pts.rows());
		for (Eigen::Index i = 0; i < pts.rows(); i++) {
			output->at(i) = pts.row(i);
		}
	}

	/** \brief convert a list of points in vector form to matrix form */
	template<typename T, int NDIM>
	void to_point_matrix(const std::vector < VECTOR_xt<T, NDIM> >& pts, MATRIX_t<T>* output) {
		if (!output) return;
		output->resize(pts.size(), pts[0].size());
		for (Eigen::Index i = 0; i < pts.size(); i++) {
			output->row(i) = pts[i];
		}
	}

	template<typename T, int NROW, int NCOL>
	inline MATRIX_t<T> add_column(const MATRIX_xt<T, NROW, NCOL>& mat, T val) {
		MATRIX_t<T> output(mat.rows(), mat.cols() + 1);
		output.fill(val);
		output.block(0, 0, mat.rows(), mat.cols()) = mat;
		return output;
	}

	template<typename T>
	size_t count_nonzeros(const MATRIX_t<T>& mat) {
		const T* data = mat.data();
		size_t n = 0;
		for (size_t i = 0; i < mat.size(); i++) {
			n += data[i] != 0;
		}
		return n;
	}

	template<typename MAT_1, typename MAT_2>
	bool is_same_size(const MAT_1& x, const MAT_2& y) {
		return x.rows() == y.rows() && x.cols() == y.cols();
	}

	//read/write eigen matrix
	template <typename MATRIX>
	void load_matrix(std::string filename, MATRIX& output)
	{
		using namespace std;
		ifstream infile(filename);
		if (!infile)
			throw GENERIC_ERROR("error reading file");

		typedef typename MATRIX::RealScalar DTYPE;
		vector<DTYPE> data;
		string line;

		int nrow = 0, ncol = 0;
		while (true)
		{
			if (!getline(infile, line)) break; //end of file
			istringstream is(line);
			DTYPE x;
			while (is >> x)
			{
				data.push_back(x);
			}
			if (is.fail() && !is.eof())
				throw GENERIC_ERROR("file format is wrong");
			nrow++;
		}

		if (data.size() == 0)
		{
			//empty file?
			output.resize(0, 0);
			return;
		}

		//determine matrix dimension
		ncol = data.size() / nrow;
		if (nrow * ncol != data.size())
			throw GENERIC_ERROR("not a rectangular matrix");

		//fill in the matrix
		output.resize(nrow, ncol);
		for (int i = 0; i < nrow; i++)
			for (int j = 0; j < ncol; j++)
			{
				output(i, j) = data[i*ncol + j];
			}

		infile.close();
	}

	template <typename MATRIX>
	void save_matrix(std::string filename, const MATRIX& input)
	{
		//open the file
		using namespace std;
		ofstream outfile(filename, ios::trunc);
		if (!outfile.is_open())
			throw GENERIC_ERROR("error opening file for output");

		auto nrow = input.rows();
		auto ncol = input.cols();
		for (int i = 0; i < nrow; i++)
		{
			for (int j = 0; j < ncol; j++)
			{
				outfile << input(i, j);
				if (j < ncol - 1)
					outfile << " ";
			}
			if (i < nrow - 1)
				outfile << endl;
		}
		outfile.close();
	}

	template<typename T>
	void save_sparse_matrix(std::string filename, const SP_MATRIX_t<T>& spmat)
	{
		//open the file
		using namespace std;
		ofstream outfile(filename, ios::trunc);
		if (!outfile.is_open())
			throw GENERIC_ERROR("error opening file for output");

		using InnerIterator = typename SP_MATRIX_t<T>::InnerIterator;

		for (int k = 0; k < spmat.outerSize(); k++)
		{
			for (InnerIterator it(spmat, k); it; ++it)
			{
				outfile << it.row() << " " << it.col() << " " << it.value() << "\n";
			}
		}

		outfile.close();
	}

	template<typename T, typename VEC_TYPE>
	void to_vector(const std::vector<T>& input, VEC_TYPE* output)
	{
		output->resize(input.size());
		for (int i = 0; i < input.size(); i++)
			(*output)(i) = input[i];
	}

	template<typename T>
	VECTOR_t<T> to_vector(const std::vector<T>& input)
	{
		VECTOR_t<T> output;
		output.resize(input.size());
		for (int i = 0; i < input.size(); i++)
			output(i) = input[i];
		return output;
	}



	//parse a string containing multiple numbers into a vector
	//the numbers are delimited by any character contained in delimiters
	//NOTE: delimiters contains multiple delimiters where delimiters[i] is the i-th delimiter
	template<typename T>
	void parse_as_vector(std::string content, VECTOR_t<T>* output, std::string delimiters)
	{
		//replace all delimiters as space
		for (char x : delimiters)
		{
			std::replace(content.begin(), content.end(), x, ' ');
		}

		//parse
		T x;
		std::istringstream is(content);
		std::vector<T> data;
		while (is >> x)
		{
			data.push_back(x);
		}
		*output = Eigen::Map<VECTOR_t<T>>(data.data(), data.size());
	}

	// ======================= matrix slicing ================================

	//get submatrix by rows and cols
	template<typename MAT_TYPE, typename IDXLIST_T_1, typename IDXLIST_T_2> //use generic matrix type because Eigen matrix may have fixed size
	void get_sub_matrix(const MAT_TYPE& mat, const IDXLIST_T_1& idxrow, const IDXLIST_T_2& idxcol, MAT_TYPE& output)
	{
		output.resize(idxrow.size(), idxcol.size());
		for (size_t i = 0; i < idxrow.size(); i++)
			for (size_t j = 0; j < idxcol.size(); j++)
			{
				output(i, j) = mat(idxrow[i], idxcol[j]);
			}
	}

	//get submatrix by rows and cols
	template<typename MAT_TYPE, typename IDXLIST_T_1, typename IDXLIST_T_2>
	MAT_TYPE get_sub_matrix(const MAT_TYPE& mat, const IDXLIST_T_1& idxrow, const IDXLIST_T_2& idxcol)
	{
		MAT_TYPE submat(idxrow.size(), idxcol.size());
		get_sub_matrix(mat, idxrow, idxcol, submat);
		return submat;
	}

	//get submatrix with multiple rows and a single column, if idxcol=-1, then all columns are retrieved
	template<typename MAT_TYPE, typename IDX_LIST_TYPE>
	void get_sub_matrix(const MAT_TYPE& mat, const IDX_LIST_TYPE& idxrow, int idxcol, MAT_TYPE& output)
	{
		std::vector<size_t> col_index;
		if (idxcol >= 0)
			col_index.push_back(idxcol);
		else
		{
			col_index.reserve(mat.cols());
			for (Eigen::Index i = 0; i < mat.cols(); i++)
				col_index.push_back((size_t)i);
		}
		get_sub_matrix(mat, idxrow, col_index, output);
	}

	//get submatrix with multiple rows and a single column, if idxcol=-1, then all columns are retrieved
	template<typename MAT_TYPE, typename IDX_LIST_TYPE>
	MAT_TYPE get_sub_matrix(const MAT_TYPE& mat, const IDX_LIST_TYPE& idxrow, int idxcol)
	{
		MAT_TYPE output;
		get_sub_matrix(mat, idxrow, idxcol, output);
		return output;
	}

	/// <summary>
	/// access elements by mask, just like x[mask] in numpy
	/// </summary>
	/// <param name="mat">the data matrix</param>
	/// <param name="mask">the mask matrix, where data(i,j) will be read if mask(i,j)!=0 </param>
	/// <returns>the data in mat covered by mask, 
	/// the order is in row-major order of mask</returns>
	template<typename T, typename D>
	VECTOR_t<T> get_values_by_mask(const MATRIX_t<T>& mat, const MATRIX_t<D>& mask) {
		assert_throw(mat.rows() == mask.rows() && mat.cols() == mask.cols(), 
			"size does not match");

		//count non zero elements
		size_t nnz = (mask.array() != 0).count();

		//read data
		VECTOR_t<T> output(nnz);
		size_t k = 0;
		auto data_mask = mask.data();
		auto data_mat = mat.data();
		for (size_t i = 0; i < mask.rows() * mask.cols(); i++) {
			if (data_mask[i] != 0)
				output(k++) = data_mat[i];
		}

		return output;
	}

	/// <summary>
	/// like numpy x[mask]=y, assign values to matrix based on non zero values in mask
	/// </summary>
	/// <param name="mat">the matrix to be modified</param>
	/// <param name="mask">the mask</param>
	/// <param name="values">the values</param>
	template<typename T, typename D>
	void set_values_by_mask(MATRIX_t<T>& mat, 
		const MATRIX_t<D>& mask, 
		const VECTOR_t<T>& values) {
		assert_throw(mat.rows() == mask.rows() && mat.cols() == mask.cols(),
			"size does not match");

		size_t nnz = (mask.array() != 0).count();
		assert_throw(values.rows() * values.cols() == nnz, "number of non zeros in mask does not match with the length of values");

		auto data_mat = mat.data();
		auto data_mask = mask.data();
		size_t k = 0;
		for (size_t i = 0; i < mask.rows() * mask.cols(); i++) {
			if (data_mask[i])
				data_mat[i] = values(k++);
		}
	}

	/// <summary>
	/// like numpy x[mask]=y, assign values to matrix based on non zero values in mask
	/// </summary>
	/// <param name="mat">the matrix to be modified</param>
	/// <param name="mask">the mask</param>
	/// <param name="value">the specified value</param>
	template<typename T, typename D>
	void set_values_by_mask(MATRIX_t<T>& mat,
		const MATRIX_t<D>& mask,
		double value) {
		assert_throw(is_same_size(mat, mask),"size does not match");

		auto data_mat = mat.data();
		auto data_mask = mask.data();
		for (size_t i = 0; i < mask.rows() * mask.cols(); i++) {
			if (data_mask[i])
				data_mat[i] = (T)value;
		}
	}

	// =======================================================================

	/// <summary>
	/// compute pca decomposition of points using svd
	/// </summary>
	/// <param name="pts">the input points</param>
	/// <param name="out_components">the output basis, each row is a basis, sorted by variance from large to small.
	/// Note that for 3D points, these basis might be right or left handed, you need to use det to make adjustment.
	/// Setting to nullptr will ignore this output.
	/// </param>
	/// <param name="out_variances">the variance of each basis, setting it to nullptr will ignore this output</param>
	template<typename MAT_TYPE_IN, typename MAT_TYPE_OUT, typename VEC_TYPE>
	void pca_by_svd(const MAT_TYPE_IN& pts, MAT_TYPE_OUT* out_components, VEC_TYPE* out_variances)
	{
		using T = typename MAT_TYPE_IN::value_type;
		using fvector = VECTOR_t<T>;
		using fmatrix = MATRIX_t<T>;

		using T_OUT_MAT = typename MAT_TYPE_OUT::value_type;
		using T_OUT_VEC = typename VEC_TYPE::value_type;

		fvector center = pts.colwise().mean();
		fmatrix _pts = pts.rowwise() - center.transpose();
		auto npts = pts.rows();
		fmatrix covmat = _pts.transpose() / (npts - 1) * _pts;

		auto svd = covmat.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
		//auto svd = covmat.bdcSvd(Eigen::ComputeFullU);

		if(out_components)
			*out_components = svd.matrixU().transpose().template cast<T_OUT_MAT>();

		if(out_variances)
			*out_variances = svd.singularValues().template cast<T_OUT_VEC>();
	}

	/// <summary>
	/// stack a list of vectors into a matrix, along vertical direction
	/// </summary>
	/// <param name="vecs">the list of vectors</param>
	/// <param name="output">the output matrix</param>
	template<typename T, int D>
	void row_stack(const std::vector<VECTOR_xt<T, D>>& vecs, MATRIX_t<T>& output) {
		size_t nrow = vecs.size();
		output.resize(nrow, D);
		for (size_t i = 0; i < nrow; i++) {
			output.row(i) = vecs[i];
		}
	}

	/// <summary>
	/// stack a list of vectors into a matrix, along vertical direction
	/// </summary>
	/// <param name="vecs">the list of vectors</param>
	/// <param name="output">the output matrix</param>
	template<typename T>
	void row_stack(const std::vector<VECTOR_t<T>>& vecs, MATRIX_t<T>& output) {
		if (vecs.empty())
			return;

		size_t nrow = vecs.size();

		//check dimension
		int ndim = vecs[0].size();
		for (const auto& x : vecs) {
			assert_throw(ndim == x.size(), "some vectors have different dimensions");
		}

		//stack
		output.resize(nrow, ndim);
		for (size_t i = 0; i < vecs.size(); i++) {
			output.row(i) = vecs[i];
		}
	}

	template<typename T>
	void row_stack(const std::vector<const MATRIX_t<T>*>& matlist, MATRIX_t<T>& output) {
		if (matlist.empty())
			return;

		//check dimensions
		int ndim = matlist[0]->cols();
		for (size_t i = 0; i < matlist.size(); i++) {
			assert_throw(ndim == matlist[i]->cols(), "input matrices do not have the same number of columns");
		}
		size_t nrow = 0;
		for (auto it = matlist.begin(); it != matlist.end(); ++it) {
			nrow += (*it)->rows();
		}

		//fill output
		output.resize(nrow, ndim);
		size_t idxrow = 0;
		for (auto it = matlist.begin(); it != matlist.end(); ++it) {
			auto row_count = (*it)->rows();
			output.block(idxrow, 0, row_count, ndim) = **it;
			idxrow += row_count;
		}
	}

	template<typename T>
	void row_stack(const std::vector<MATRIX_t<T>>& matlist, MATRIX_t<T>& output) {
		std::vector<const MATRIX_t<T>*> _matlist(matlist.size());
		for (size_t i = 0; i < matlist.size(); i++)
			_matlist[i] = &matlist[i];
		row_stack(_matlist, output);
	}

	template<typename T>
	void row_stack(const std::vector<SP_MATRIX_t<T>>& matlist, SP_MATRIX_t<T>& output) {
		std::vector<const SP_MATRIX_t<T>*> _matlist(matlist.size());
		for (size_t i = 0; i < matlist.size(); i++)
			_matlist[i] = &matlist.at(i);
		row_stack(_matlist, output);
	}


	template<typename T>
	void row_stack(const std::vector<const SP_MATRIX_t<T>*>& matlist, SP_MATRIX_t<T>& output)
	{
		if (matlist.size() == 0)
			return;

		using trip_type = TRIP_t<T>;
		using mat_type = SP_MATRIX_t<T>;
		using inner_iter_type = typename mat_type::InnerIterator;

		//all matrices must have the same number of columns
		auto n_col = matlist.at(0)->cols();
		for (const auto& m : matlist)
			assert_throw(m->cols() == n_col, "not all matrices have the same number of columns");

		//count the number of nonzero entries
		size_t nnz = 0;
		for (const auto& m : matlist)
			nnz += m->nonZeros();

		//stack the matrices
		std::vector<trip_type> triplist;
		triplist.reserve(nnz);

		Eigen::Index idxrow_start = 0;
		for (auto iter = matlist.begin(); iter != matlist.end(); ++iter) {
			const auto& mat = **iter;
			for (int k = 0; k < mat.outerSize(); k++) {
				for (inner_iter_type it(mat, k); it; ++it) {
					triplist.emplace_back(idxrow_start + it.row(), it.col(), it.value());
				}
			}
			idxrow_start += mat.rows();
		}

		//create new matrix
		mat_type _output(idxrow_start, n_col);
		_output.setFromTriplets(triplist.begin(), triplist.end());
		output = _output;
	}

	template<typename T>
	void concatenate(const std::vector<const VECTOR_t<T>*>& vecs, VECTOR_t<T>& output)
	{
		if (vecs.size() == 0)
		{
			output.resize(0);
			return;
		}

		size_t n = 0;
		for (auto p : vecs)
			n += p->size();

		output.resize(n);
		size_t k = 0;
		for (auto p : vecs)
		{
			memcpy(output.data() + k, p->data(), p->size() * sizeof(T));
			k += p->size();
		}
	}

	/// <summary>
	/// like numpy img[mask] = valmat. If img has N channels, then valmat has N cols, each row 
	/// of valmat is the color of a channel. The order of mask is row-major.
	/// </summary>
	/// <param name="img">the image to be modified</param>
	/// <param name="mask">the mask where nonzeros defines locations to modify</param>
	/// <param name="valmat">the colors, each row is a color.</param>
	template<typename T, typename U, typename R>
	void set_values_by_mask(ImageRGBA_t<T>& img, const MATRIX_t<U>& mask, const MATRIX_t<R>& valmat) {
		assert_throw(img.get_width() == mask.cols() && img.get_height() == mask.rows(),
			"size mismatch");
		assert_throw(img.get_number_of_channels() == valmat.cols(), "number of image channels does not match columns of valmat"); 

		int nch = img.get_number_of_channels();
		std::vector<MATRIX_t<T>*> chlist(nch);
		for (int i = 0; i < nch; i++)
			chlist[i] = &img.get_channel(i);

		size_t idx = 0;
		for(size_t i=0; i<mask.rows(); i++)
			for (size_t j = 0; j < mask.cols(); j++) {
				if (mask(i, j)) {
					for (int k = 0; k < nch; k++) {
						(*chlist[k])(i, j) = valmat(idx, k);
					}
					idx++;
				}
			}
	}

	/// <summary>
	/// like numpy v = img[mask]
	/// </summary>
	/// <param name="img">the image</param>
	/// <param name="mask">the reading mask</param>
	/// <returns>the colors, each row is a color</returns>
	template<typename T, typename U>
	MATRIX_t<T> get_values_by_mask(const ImageRGBA_t<T>& img, const MATRIX_t<U>& mask) {
		assert_throw(img.get_width() == mask.cols() && img.get_height() == mask.rows(), "size mismatch");
		int nch = img.get_number_of_channels();
		int nnz = count_nonzeros(mask);

		std::vector<const MATRIX_t<T>*> chlist(nch);
		for (int i = 0; i < nch; i++)
			chlist[i] = &img.get_channel(i);
		
		MATRIX_t<T> output(nnz, nch);
		size_t idx = 0;
		for(size_t i=0; i<mask.rows(); i++)
			for (size_t j = 0; j < mask.cols(); j++) {
				if (mask(i, j)) {
					for (int k = 0; k < nch; k++) {
						output(idx, k) = (*chlist[k])(i, j);
					}
					idx++;
				}
			}

		return output;
	}

	template<typename T, int OPT>
	Eigen::SparseMatrix<T, OPT> get_sub_matrix_by_rows(const Eigen::SparseMatrix<T, OPT>& mat, const std::set<Eigen::Index>& idxrow)
	{
		using InnerIterator_t = typename Eigen::SparseMatrix<T, OPT>::InnerIterator;
		std::vector<Eigen::Triplet<T>> triplist;
		using TripIndexType = typename Eigen::SparseMatrix<T, OPT>::StorageIndex;

		VECTOR_b mask(mat.rows());
		mask.setZero();
		for (auto k : idxrow)
			mask(k) = true;

		VECTOR_t<Eigen::Index> row_indices_old2new(mat.rows());
		{
			Eigen::Index idx = 0;
			for (auto k : idxrow)
				row_indices_old2new(k) = idx++;
		}

		for (Eigen::Index k = 0; k < mat.outerSize(); k++)
		{
			for (InnerIterator_t it(mat, k); it; ++it)
			{
				//use this row?
				if(!mask(it.row())) continue;
				auto row_idx = row_indices_old2new(it.row());
				triplist.emplace_back((TripIndexType)row_idx, (TripIndexType)it.col(), it.value());
			}
		}

		//create a new matrix
		Eigen::SparseMatrix<T, OPT> output(idxrow.size(), mat.cols());
		output.setFromTriplets(triplist.begin(), triplist.end());
		return output;
	}

	template<typename T, int NROW, int NCOL>
	void column_stack(const std::vector< MATRIX_xt<T, NROW, NCOL> >& matlist, MATRIX_t<T>& output)
	{
		if (matlist.empty())
		{
			output = MATRIX_t<T>();
			return;
		}
			
		Eigen::Index nrow = matlist[0].rows();
		Eigen::Index ncol = 0;
		for (const auto& x : matlist)
			ncol += x.cols();

		output.resize(nrow, ncol);
		Eigen::Index idxcol = 0;
		for (size_t i = 0; i < matlist.size(); i++)
		{
			const auto& x = matlist[i];
			output.block(0, idxcol, nrow, x.cols()) = x;
			idxcol += x.cols();
		}
	}

	template<typename T, int D>
	void column_stack(const std::vector< VECTOR_xt<T, D> >& veclist, MATRIX_t<T>& output)
	{
		if (veclist.empty())
		{
			output = MATRIX_t<T>();
			return;
		}

		auto nrow = veclist[0].size();
		auto ncol = veclist.size();

		output.resize(nrow, ncol);
		for (size_t i = 0; i < veclist.size(); i++)
			output.col(i) = veclist[i];
	}

	template<typename T, typename D>
	void find_nonzeros(const VECTOR_t<T>& vec, std::vector<D>* output_indices)
	{
		if (!output_indices) return;
		output_indices->clear();
		for (Eigen::Index i = 0; i < vec.size(); i++)
		{
			if (vec(i)) output_indices->push_back(i);
		}
	}

	template<typename T, typename D>
	void find_nonzeros(const VECTOR_t<T>& vec, VECTOR_t<D>* output_indices)
	{
		if (!output_indices) return;

		std::vector<D> _output_indices;
		find_nonzeros(vec, &_output_indices);
		*output_indices = Eigen::Map<VECTOR_t<D>>(_output_indices.data(), _output_indices.size());
	}

	template<typename T, typename D>
	void find_nonzeros(const MATRIX_t<T>& mat, iMATRIX* output_ij)
	{
		if (!output_ij) return;
		std::vector<iVECTOR_2> ijs;
		for(Eigen::Index i=0; i<mat.rows(); i++)
			for (Eigen::Index j = 0; j < mat.cols(); j++)
			{
				if (mat(i, j))
					ijs.emplace_back(i, j);
			}
		row_stack(ijs, *output_ij);
	}
}