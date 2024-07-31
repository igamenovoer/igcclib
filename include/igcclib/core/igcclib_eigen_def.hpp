//Eigen-related definitions
#pragma once
#include <Eigen/Eigen>
#include <igcclib/igcclib_master.hpp>

namespace _NS_UTILITY
{
	template <typename T>
	using MATRIX_t = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

	template <typename T>
	using MATRIX_col_t = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

	template <typename T, int N_ROW, int N_COL>
	using MATRIX_xt = Eigen::Matrix<T, N_ROW, N_COL, Eigen::RowMajor>;

	template <typename T>
	using MATRIX_3t = Eigen::Matrix<T, 3, 3, Eigen::RowMajor>;

	template <typename T>
	using MATRIX_4t = Eigen::Matrix<T, 4, 4, Eigen::RowMajor>;

	template<typename T>
	using VECTOR_t = Eigen::Matrix<T, Eigen::Dynamic, 1>;

	template<typename T, int N_ELEM>
	using VECTOR_xt = Eigen::Matrix<T, N_ELEM, 1>;

	template<typename T>
	using VECTOR_2t = Eigen::Matrix<T, 2, 1>;

	template<typename T>
	using VECTOR_3t = Eigen::Matrix<T, 3, 1>;

	template<typename T>
	using VECTOR_4t = Eigen::Matrix<T, 4, 1>;

	template<typename T>
	using SP_MATRIX_t = Eigen::SparseMatrix<T, Eigen::RowMajor>;

	template<typename T>
	using SP_MATRIX_col_t = Eigen::SparseMatrix<T, Eigen::ColMajor>;

	typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MATRIX_f;
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MATRIX_d;
	typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MATRIX_i;
	typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MATRIX_b;
	typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MATRIX_u8;
	typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> MATRIX_3d;
	typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> MATRIX_4d;

	typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SP_MATRIX_d;
	typedef Eigen::SparseMatrix<float, Eigen::RowMajor> SP_MATRIX_f;
	typedef Eigen::SparseMatrix<int, Eigen::RowMajor> SP_MATRIX_i;
	typedef Eigen::SparseMatrix<uint8_t, Eigen::RowMajor> SP_MATRIX_u8;



	typedef Eigen::Triplet<int> TRIP_i;
	typedef Eigen::Triplet<double> TRIP_d;
	typedef Eigen::Triplet<float> TRIP_f;
	typedef Eigen::Triplet<bool> TRIP_b;

	template<typename T>
	using TRIP_t = Eigen::Triplet<T>;

	typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VECTOR_f;
	typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VECTOR_d;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VECTOR_i;
	typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VECTOR_b;
	typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> VECTOR_u8;

	typedef Eigen::Matrix<double, 2, 1> VECTOR_2d;
	typedef Eigen::Matrix<double, 4, 1> VECTOR_4d;
	typedef Eigen::Matrix<int, 4, 1> VECTOR_4i;

	typedef Eigen::Matrix<int, 3, 1> VECTOR_3i;
	typedef Eigen::Matrix<uint8_t, 3, 1> VECTOR_3u8;
	typedef Eigen::Matrix<double, 1, 3> VECTOR_3d_row;
	typedef Eigen::Matrix<double, 3, 1> VECTOR_3d;

	//objects using float_type
	using fMATRIX = Eigen::Matrix<float_type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
	using fMATRIX_2 = Eigen::Matrix<float_type, 2, 2, Eigen::RowMajor>;
	using fMATRIX_3 = Eigen::Matrix<float_type, 3, 3, Eigen::RowMajor>;
	using fMATRIX_4 = Eigen::Matrix<float_type, 4, 4, Eigen::RowMajor>;
	template<int N> using fMATRIX_n = Eigen::Matrix<float_type, N, N>;


	using fSP_MATRIX = Eigen::SparseMatrix<float_type, Eigen::RowMajor>;
	using fSP_MATRIX_col = SP_MATRIX_col_t<float_type>;
	using fTRIP = Eigen::Triplet<float_type>;
	using fVECTOR = Eigen::Matrix<float_type, Eigen::Dynamic, 1>;
	using fVECTOR_2 = Eigen::Matrix<float_type, 2, 1>;
	using fVECTOR_3 = Eigen::Matrix<float_type, 3, 1>;
	using fVECTOR_4 = Eigen::Matrix<float_type, 4, 1>;
	template<int N> using fVECTOR_n = Eigen::Matrix<float_type, N, 1>;
	
	using fQuaternion = Eigen::Quaternion<float_type>;

	//objects using int_type
	using iMATRIX = Eigen::Matrix<int_type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
	using iMATRIX_2 = Eigen::Matrix<int_type, 2, 2, Eigen::RowMajor>;
	using iMATRIX_3 = Eigen::Matrix<int_type, 3, 3, Eigen::RowMajor>;
	using iMATRIX_4 = Eigen::Matrix<int_type, 4, 4, Eigen::RowMajor>;
	using iSP_MATRIX = Eigen::SparseMatrix<int_type, Eigen::RowMajor>;
	using iTRIP = Eigen::Triplet<int_type>;
	using iVECTOR = Eigen::Matrix<int_type, Eigen::Dynamic, 1>;
	using iVECTOR_2 = Eigen::Matrix<int_type, 2, 1>;
	using iVECTOR_3 = Eigen::Matrix<int_type, 3, 1>;
	using iVECTOR_4 = Eigen::Matrix<int_type, 4, 1>;

	template<typename T>
	class ImageRGBA_t
	{
	public:
		using dtype = T;

		ImageRGBA_t<T>() {}
		ImageRGBA_t<T>(size_t width, size_t height)
		{
			allocate(width, height);
			clear(0, 0, 0, 0);
		}

		//image channels, if it is a gray image, only the R channel and possibly the A channel will be non-empty
		MATRIX_t<T> r, g, b, a;
		void allocate(size_t width, size_t height, size_t n_channel = 4)
		{
			if(n_channel >= 1)
				r.resize(height, width);

			if(n_channel >= 2)
				g.resize(height, width);

			if(n_channel >= 3)
				b.resize(height, width);

			if(n_channel >= 4)
				a.resize(height, width);
		}

		/// <summary>
		/// convert the image to another data type, the image values will be scaled and offset before conversion.
		/// That is, output = input * value_scale + value_offset
		/// </summary>
		template<typename D>
		void convert_to(ImageRGBA_t<D>& output, double value_scale, double value_offset = 0) const
		{
			for (int i = 0; i < get_number_of_channels(); i++)
			{
				MATRIX_d x = get_channel(i).template cast<double>();
				x.array() = x.array() * value_scale + value_offset;
				output.get_channel(i) = x.template cast<D>();
			}
		}

		void clear(T r, T g, T b, T a)
		{
			this->r.fill(r);
			this->g.fill(g);
			this->b.fill(b);
			this->a.fill(a);
		}

		void set_RGB(const MATRIX_t<T>& r, const MATRIX_t<T>& g, const MATRIX_t<T>& b)
		{
			int width = r.cols();
			int height = r.rows();
			assert_throw(g.rows() == height && g.cols() == width, "size mismatch between r and g");
			assert_throw(b.rows() == height && b.cols() == width, "size mismatch between r and b");

			this->r = r;
			this->g = g;
			this->b = b;
			this->a = decltype(this->a)();
		}

		void set_RGBA(const MATRIX_t<T>& r, const MATRIX_t<T>& g, const MATRIX_t<T>& b, const MATRIX_t<T>& a)
		{
			int width = r.cols();
			int height = r.rows();
			assert_throw(g.rows() == height && g.cols() == width, "size mismatch between r and g");
			assert_throw(b.rows() == height && b.cols() == width, "size mismatch between r and b");
			assert_throw(a.rows() == height && a.cols() == width, "size mismatch between r and a");

			this->r = r;
			this->g = g;
			this->b = b;
			this->a = a;
		}

		/// <summary>
		/// is this image empty?
		/// </summary>
		bool is_empty() const
		{
			return r.rows() == 0;
		}

		/// <summary>
		/// set the image as a grayscale image
		/// </summary>
		/// <param name="gray">the gray channel</param>
		/// <param name="alpha">the alpha channel</param>
		void set_gray(const MATRIX_t<T>& gray, const MATRIX_t<T>& alpha)
		{
			assert_throw(gray.rows() == alpha.rows() && gray.cols() == alpha.cols(), "size mismatch between gray and alpha");
			this->r = gray;
			this->a = alpha;

			//remove GB channels
			this->g = decltype(this->g)();
			this->b = decltype(this->b)();
		}

		/// <summary>
		/// set the image as a grayscale image
		/// </summary>
		/// <param name="gray">the value of the gray channel</param>
		void set_gray(const MATRIX_t<T>& gray)
		{
			this->r = gray;

			this->g = decltype(this->g)();
			this->b = decltype(this->b)();
			this->a = decltype(this->a)();
		}

		// is this a grayscale image?
		bool is_gray() const
		{ return r.rows() > 0 && g.rows() == 0 && b.rows() == 0; }

		// size of the image
		int get_width() const { return (int)r.cols(); }
		int get_height() const{ return (int)r.rows(); }

		// has alpha channel?
		int has_alpha() const { return a.rows() > 0; }

		// access channel
		int get_number_of_channels() const {
			int n = 0;
			n += this->r.rows() > 0;
			n += this->g.rows() > 0;
			n += this->b.rows() > 0;
			n += this->a.rows() > 0;
			return n;
		}

		MATRIX_t<T>& get_channel(size_t i) {
			if (i == 0)
				return this->r;
			else if (i == 1)
				return this->g;
			else if (i == 2)
				return this->b;
			else if (i == 3)
				return this->a;
			else
				throw std::logic_error("channel out of range");
		}

		const MATRIX_t<T>& get_channel(size_t i) const {
			if (i == 0)
				return this->r;
			else if (i == 1)
				return this->g;
			else if (i == 2)
				return this->b;
			else if (i == 3)
				return this->a;
			else
				throw std::logic_error("channel out of range");
		}

		/// <summary>
		/// convert the image into linear buffer, in RGBARGBA format.
		/// The pixel data will be normalized to [out_min, out_max]
		/// </summary>
		/// <param name="output">the output buffer</param>
		/// <param name="in_min">min possible value the input image </param>
		/// <param name="in_max">max possible value of the input image</param>
		/// <param name="out_min">min possible value of the output buffer</param>
		/// <param name="out_max">max possible value of the output buffer</param>
		template<typename D>
		void to_linear_buffer(std::vector<D>& output,
			float_type in_min, float_type in_max, float_type out_min, float_type out_max) const
		{
			int width = get_width();
			int height = get_height();
			int nch = get_number_of_channels();

			std::vector<const MATRIX_t<T>*> matlist;

			for (int i = 0; i < nch; i++)
				matlist.push_back(&get_channel(i));

			int w = 0;
			output.resize(width * height * nch);
			auto in_len = in_max - in_min;
			auto out_len = out_max - out_min;
			for (int i = 0; i < width * height; i++)
			{
				for (int k = 0; k < matlist.size(); k++)
				{
					output[w++] = (D)((float_type((*matlist[k])(i)) - in_min) / in_len * out_len + out_min);
				}
			}
		}

		template<typename D>
		void from_linear_buffer(const std::vector<D>& input,
			size_t width, size_t height, size_t n_channel,
			float_type in_min, float_type in_max, float_type out_min, float_type out_max)
		{
			assert_throw(input.size() == width * height * n_channel, "data length does not match");

			//collect the channels
			this->allocate(width, height, n_channel);
			std::vector<MATRIX_t<T>*> matlist;
			for (size_t i = 0; i < n_channel; i++)
				matlist.push_back(&get_channel(i));

			auto in_len = in_max - in_min;
			auto out_len = out_max - out_min;

			for (size_t i = 0; i < width * height; i++)
			{
				for (size_t k = 0; k < n_channel; k++)
				{
					//read a pixel and normalize it
					T value = (T)((float_type(input[i * n_channel + k]) - in_min) / in_len * out_len + out_min);

					//write it
					(*matlist[k])(i) = value;
				}
			}
		}
	};

	using ImageRGBA_d = ImageRGBA_t<double>;
	using ImageRGBA_f = ImageRGBA_t<float>;
	using ImageRGBA_u = ImageRGBA_t<uint8_t>;
	using ImageRGBA_i = ImageRGBA_t<int>;
	using fImageRGBA = ImageRGBA_t<float_type>;
	using iImageRGBA = ImageRGBA_t<int_type>;
};

#define CODEGEN_CEREAL_EIGEN_MATRIX(nrow, ncol, order) \
	template<typename CerealArch_t, typename Data_t> \
	void save(CerealArch_t& ar, const Eigen::Matrix<Data_t, nrow, ncol, order>& mat) { \
		_save<CerealArch_t, Data_t, nrow, ncol, order>(ar, mat);\
	}\
	template<typename CerealArch_t, typename Data_t>\
	void load(CerealArch_t& ar, Eigen::Matrix<Data_t, nrow, ncol, order>& mat) { \
		_load<CerealArch_t, Data_t, nrow, ncol, order>(ar, mat);\
	}

namespace cereal {
	template<typename CerealArch_t, typename Data_t, int NROW, int NCOL, int ORDER>
	void _save(CerealArch_t& ar, const Eigen::Matrix<Data_t, NROW, NCOL, ORDER>& mat) {
		ar(mat.rows());
		ar(mat.cols());

		//save data
		std::vector<Data_t> buf(mat.data(), mat.data() + mat.rows() * mat.cols());
		ar(buf);
	}

	template<typename CerealArch_t, typename Data_t, int NROW, int NCOL, int ORDER>
	void _load(CerealArch_t& ar, Eigen::Matrix<Data_t, NROW, NCOL, ORDER>& mat) {
		Eigen::Index nrow, ncol;
		ar(nrow);
		ar(ncol);

		//save data
		std::vector<Data_t> buf;
		ar(buf);

		using MAT_TYPE = Eigen::Matrix<Data_t, NROW, NCOL, ORDER>;
		mat = Eigen::Map<MAT_TYPE>(buf.data(), nrow, ncol);
	}

	//for matrix
	CODEGEN_CEREAL_EIGEN_MATRIX(Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor);
	CODEGEN_CEREAL_EIGEN_MATRIX(4, 4, Eigen::RowMajor);
	CODEGEN_CEREAL_EIGEN_MATRIX(3, 3, Eigen::RowMajor);
	CODEGEN_CEREAL_EIGEN_MATRIX(2, 2, Eigen::RowMajor);

	//for vector
	CODEGEN_CEREAL_EIGEN_MATRIX(Eigen::Dynamic, 1, Eigen::ColMajor);
	CODEGEN_CEREAL_EIGEN_MATRIX(4, 1, Eigen::ColMajor);
	CODEGEN_CEREAL_EIGEN_MATRIX(3, 1, Eigen::ColMajor);
	CODEGEN_CEREAL_EIGEN_MATRIX(2, 1, Eigen::ColMajor);
}
