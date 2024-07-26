#pragma once
#include <vector>
#include <memory>
#include <nanoflann.hpp>
#include <igcclib/core/igcclib_eigen.hpp>

namespace _NS_UTILITY
{
	/// <summary>
	/// point dataset used in nanoflann's kdtree
	/// </summary>
	template<typename T>
	class PointDataset
	{
	public:
		typedef std::vector<T> Point;
		std::vector<Point> m_pts;

		// return number of points
		inline size_t kdtree_get_point_count() const { return m_pts.size(); }

		// Returns the dim'th component of the idx'th point in the class:
		// Since this is inlined and the "dim" argument is typically an immediate value, the
		//  "if/else's" are actually solved at compile time.
		inline T kdtree_get_pt(const size_t idx, size_t dim) const
		{
			return m_pts[idx][dim];
		}

		// Optional bounding-box computation: return false to default to a standard bbox computation loop.
		//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
		//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

		template<typename _T>
		void set_points(const MATRIX_t<_T>& pts)
		{
			//assert_throw(N == pts.cols(), "dimension mismatch");
			m_pts.resize(pts.rows());
			for (size_t i = 0; i < pts.rows(); i++)
			{
				m_pts[i].resize(pts.cols());
				for (size_t k = 0; k < pts.cols(); k++)
				{
					m_pts[i][k] = pts(i, k);
				}
			}
		}

		//create an empty point with specific dimension
		Point make_point(int ndim)
		{
			Point x(ndim);
			return x;
		}
	};

	template<typename T>
	using PointDatasetNDim_d = PointDataset<double>;
	using PointDatasetNDim_f = PointDataset<float>;
	using fPointDatasetNDim = PointDataset<float_type>;

	template<typename T>
	using _NF_KDTREE_STATIC = nanoflann::KDTreeSingleIndexAdaptor<
		nanoflann::L2_Simple_Adaptor<T, PointDataset<T>>, PointDataset<T>>;

	template<typename TREE_TYPE, typename POINT_SET_TYPE>
	class KDTreeSearcher
	{
	public:
		typedef TREE_TYPE KDTREE;
		typedef typename TREE_TYPE::ElementType DATA_TYPE;

	private:
		std::shared_ptr<POINT_SET_TYPE> m_point_data;
		std::shared_ptr<TREE_TYPE> m_kdtree;
		typedef KDTreeSearcher<TREE_TYPE, POINT_SET_TYPE> self_t;

	public:
		/// <summary>
		/// query k-nearest neighbors
		/// </summary>
		/// <param name="pts">the query points, reach row is a point</param>
		/// <param name="K">number of nearest neighbors</param>
		/// <param name="distmat">distmat(i,j) is the distance from i-th point to its j-th nearest neighbor</param>
		/// <param name="indexmat">indexmat(i,j) is the index of the j-th nearest point for pts(i,:)</param>
		void query(const MATRIX_t<DATA_TYPE>& pts, int K, MATRIX_t<DATA_TYPE>* distmat, MATRIX_i* indexmat)
		{
			nanoflann::KNNResultSet<DATA_TYPE> res(K);

			std::vector<size_t> idx(K);
			std::vector<DATA_TYPE> dist(K);

			int n_dim = pts.cols();

			//assert(false);
			//_NF_KDTREE_STATIC<double>* m_kdtree;

			nanoflann::SearchParams dummy;

			size_t n_query_points = pts.rows();
			MATRIX_t<DATA_TYPE> _distmat(n_query_points, K);
			MATRIX_i _indexmat(n_query_points, K);

			auto p = m_point_data->make_point(n_dim);
			for (size_t i = 0; i < n_query_points; i++)
			{
				res.init(&idx[0], &dist[0]);
				for (int k = 0; k < n_dim; k++)
					p[k] = pts(i, k);
				m_kdtree->findNeighbors(res, &p[0], dummy);

				//write result
				for (int k = 0; k < K; k++)
				{
					_distmat(i, k) = std::sqrt(dist[k]);
					_indexmat(i, k) = (int)idx[k];
				}
			}

			if (distmat)
				*distmat = _distmat;
			if (indexmat)
				*indexmat = _indexmat;
		}

		/// <summary>
		/// query for neighbors within a specific distance
		/// </summary>
		/// <param name="point"></param>
		/// <param name="radius"></param>
		/// <param name="out_dist"></param>
		/// <param name="out_index"></param>
		void query_ball_point(const VECTOR_t<DATA_TYPE>& point, double radius, VECTOR_t<DATA_TYPE>* out_dist, VECTOR_i* out_index)
		{
			std::vector<std::pair<size_t, double>> ret_matches;
			nanoflann::SearchParams params;
			params.sorted = true;

			int n_dim = point.size();
			auto p = m_point_data->make_point(n_dim);
			for (int i = 0; i < n_dim; i++)
				p[i] = point(i);

			//note that, the input distance to nanoflann is the distance squared, and the returned distances are also squared
			auto n_match = m_kdtree->radiusSearch(&p[0], radius * radius, ret_matches, params);
			VECTOR_t<DATA_TYPE> _out_dist(ret_matches.size());
			VECTOR_i _out_index(ret_matches.size());
			for (int i = 0; i < ret_matches.size(); i++)
			{
				_out_dist(i) = std::sqrt(ret_matches[i].second);
				_out_index(i) = ret_matches[i].first;
			}

			if (out_dist)
				*out_dist = _out_dist;
			if (out_index)
				*out_index = _out_index;
		}

		void init_with_points(const MATRIX_t<DATA_TYPE>& pts) {
			init_with_points(*this, pts);
		}

		/// <summary>
		/// create kdtree with points
		/// </summary>
		/// <param name="pts">the points, each row is a point</param>
		/// <returns>A KDTreeSearcher</returns>
		static void init_with_points(self_t& output, const MATRIX_t<DATA_TYPE>& pts)
		{
			using namespace std;
			//assert_throw(pts.cols() == ndim(), "dimension of points does not match with that of the tree");

			//create dataset
			output = self_t();
			output.m_point_data = make_shared<POINT_SET_TYPE>();
			output.m_point_data->set_points(pts);

			//create tree
			int n_dim = pts.cols();
			output.m_kdtree = make_shared<KDTREE>(n_dim, *output.m_point_data);
			output.m_kdtree->buildIndex();
		}

	public:
		KDTreeSearcher<TREE_TYPE, POINT_SET_TYPE>() {}
		virtual ~KDTreeSearcher<TREE_TYPE, POINT_SET_TYPE>() {}
	};

	/// <summary>
	/// n-dimensional kdtree with T type
	/// </summary>
	template<typename T>
	using KDTREE_STATIC_nt = KDTreeSearcher<_NF_KDTREE_STATIC<T>, PointDataset<T>>;

	/// <summary>
	/// n-dimensional kdtree with double type
	/// </summary>
	using KDTREE_STATIC_nd = KDTreeSearcher<_NF_KDTREE_STATIC<double>, PointDataset<double>>;

	/// <summary>
	/// n-dimensional kdtree with float type
	/// </summary>
	using KDTREE_STATIC_nf = KDTreeSearcher<_NF_KDTREE_STATIC<float>, PointDataset<float>>;

	using fKDTREE_STATIC_n = KDTreeSearcher<_NF_KDTREE_STATIC<float_type>, PointDataset<float_type>>;
};


