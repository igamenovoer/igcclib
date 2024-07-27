#pragma once

#include <igcclib/igcclib_master.hpp>
#include <igcclib/core/igcclib_eigen_def.hpp>
#include <igcclib/extern/splinter/bspline.h>
#include <igcclib/extern/splinter/bsplinebuilder.h>

namespace _NS_UTILITY {
	/** \brief a b-spline curve */
	template<int NDIM, typename T = float_type>
	class IGCCLIB_API BSplineCurve {
	public:
		using Point_t = VECTOR_xt<T, NDIM>;

		/** \brief evaluate the curve at point t */
		Point_t evaluate_point(float_type t) const;

		/** \brief evaluate the first or second  derivative at point t.
		If arc-length parameterization is used, the 2nd order derivative is curve normal.*/
		Point_t evaluate_derivative(float_type t, int order = 1) const;

		/** \brief evaluate the curve at points t */
		MATRIX_t<T> evaluate_point(const VECTOR_t<T>& ts) const;

		/** \brief evaluate the first or second derivatives at parameters ts.
		If arc-length parameterization is used, the 2nd order derivative is curve normal.*/
		MATRIX_t<T> evaluate_derivative(const VECTOR_t<T>& ts, int order = 1) const;

		/**
		* \brief fit to a list of points
		*
		* \param pts the points to be fitted
		* \param ts the parameter t for each point
		* \param w_smooth smoothing weight
		* \param n_degree the degree of the curve
		* \param end_point_multiplicity in order to force the end point of the curve
		to tightly match the input samples, we need to duplicate the end points during fitting.
		If this number is less than 1, it is interpreted as the fraction of the number
		of sample points. Otherwise, it is interpreted as the duplication count.
		*/
		void init_by_fit(
			const MATRIX_t<T>& pts, const VECTOR_t<T>& ts, int n_degree,
			double w_smooth = 0.02, double end_point_multiplicity = 10.0);

		/**
		* \brief fit to a list of points, using arc-length parameterization.
		*
		* \param pts the points to be fitted
		* \param w_smooth smoothing weight
		* \param n_degree the degree of the curve
		* \param end_point_multiplicity in order to force the end point of the curve
		to tightly match the input samples, we need to duplicate the end points during fitting.
		If this number is less than 1, it is interpreted as the fraction of the number
		of sample points. Otherwise, it is interpreted as the duplication count.
		*/
		void init_by_fit(const MATRIX_t<T>& pts, int n_degree,
			double w_smooth = 0.02, double end_point_multiplicity = 10.0);

		double get_t_min() const { return m_tmin; }
		double get_t_max() const { return m_tmax; }
		double is_empty() const {
			return m_tmin == 0 && m_tmax == 0;
		}

		struct _BSplineData
		{
			SPLINTER::DenseVector coefs;
			std::vector<std::vector<double>> knots;
			std::vector<unsigned int> basis_degrees;

			template<typename Arch_t>
			void serialize(Arch_t& ar)
			{
				ar(coefs);
				ar(knots);
				ar(basis_degrees);
			}
		};

		template<typename Arch_t>
		void save(Arch_t& ar) const {
			std::vector<_BSplineData> bsdata;
			for (const auto& x : m_splines)
			{
				_BSplineData data;
				data.basis_degrees = x.getBasisDegrees();
				data.coefs = x.getCoefficients();
				data.knots = x.getKnotVectors();
				bsdata.push_back(data);
			}
			ar(bsdata);
			ar(m_tmin, m_tmax);
		}

		template<typename Arch_t>
		void load(Arch_t& ar) {
			std::vector<_BSplineData> bsdata;
			ar(bsdata);
			for (auto& x : bsdata)
			{
				m_splines.emplace_back(x.coefs, x.knots, x.basis_degrees);
			}

			ar(m_tmin, m_tmax);
		}

	private:
		using Spline_t = SPLINTER::BSpline;

		/** \brief spline for each dimension */
		std::vector<Spline_t> m_splines;

		/** \brief range of parameter t */
		double m_tmin = 0;
		double m_tmax = 0;
	};

	using fBSplineCurve_3 = BSplineCurve<3, float_type>;
	using fBSplineCurve_2 = BSplineCurve<2, float_type>;

	/** \brief given a polyline, compute arclength parameter. If normalize = true, the parameter is normalized to unit range. */
	inline fVECTOR make_arclength_parameter(const fMATRIX& pts, bool normalize = true);
}

namespace _NS_UTILITY {
	fVECTOR make_arclength_parameter(const fMATRIX& pts, bool normalize) {
		fVECTOR ts(pts.rows());
		ts(0) = 0;
		for (Eigen::Index i = 1; i < pts.rows(); i++) {
			ts(i) = ts(i-1) + (pts.row(i) - pts.row(i - 1)).norm();
		}
		if (normalize)
			ts /= ts(ts.size() - 1);
		return ts;
	}

	template<int NDIM, typename T>
	inline MATRIX_t<T> BSplineCurve<NDIM, T>::evaluate_derivative(
		const VECTOR_t<T>& ts, int order) const {

		assert_throw(order == 1 || order == 2, "order must be 1 or 2");

		MATRIX_t<T> output(ts.size(), m_splines.size());
		std::vector<double> x(1);
		for (Eigen::Index k = 0; k < ts.size(); k++) {
			x[0] = ts(k);
			for (int i = 0; i < NDIM; i++) {
				double y = 0;
				if (order == 1)
					y = m_splines[i].evalJacobian(x)[0];
				else
					y = m_splines[i].evalHessian(x)[0][0];
				output(k,i) = y;
			}
		}
		return output;
	}

	template<int NDIM, typename T>
	inline MATRIX_t<T> BSplineCurve<NDIM, T>::evaluate_point(const VECTOR_t<T>& ts) const {
		std::vector<double> x(1);
		MATRIX_t<T> output(ts.size(), m_splines.size());
		for (Eigen::Index k = 0; k < ts.size(); k++)
		{
			x[0] = ts(k);
			for (int i = 0; i < NDIM; i++) {
				auto y = m_splines[i].eval(x);
				output(k,i) = y;
			}
		}
		return output;
	}

	template<int NDIM, typename T>
	inline void BSplineCurve<NDIM, T>::init_by_fit(
		const MATRIX_t<T>& pts, int n_degree, double w_smooth, double end_point_multiplicity) 
	{
		double len = 0;
		VECTOR_t<float_type> ts(pts.rows());

		ts(0) = 0;
		for (int i = 0; i < pts.rows() - 1; i++) {
			len += (pts.row(i) - pts.row(i + 1)).norm();
			ts(i + 1) = len;
		}
		ts /= len;

		init_by_fit(pts, ts, n_degree, w_smooth, end_point_multiplicity);
	}

	template<int NDIM, typename T /*= float_type*/>
	inline typename BSplineCurve<NDIM, T>::Point_t BSplineCurve<NDIM, T>::evaluate_derivative(
		float_type t, int order) const
	{
		assert_throw(order == 1 || order == 2, "order must be 1 or 2");
		Point_t output;
		std::vector<double> x = { t };
		for (int i = 0; i < NDIM; i++) {
			double y = 0;
			if (order == 1)
				y = m_splines[i].evalJacobian(x)[0];
			else
				y = m_splines[i].evalHessian(x)[0][0];
			output[i] = y;
		}
		return output;
	}

	template<int NDIM, typename T /*= float_type*/>
	inline typename BSplineCurve<NDIM,T>::Point_t BSplineCurve<NDIM, T>::evaluate_point(float_type t) const
	{
		std::vector<double> x = { t };
		Point_t output;
		for (int i = 0; i < NDIM; i++) {
			auto y = m_splines[i].eval(x);
			output[i] = y;
		}
		return output;
	}

	template<int NDIM, typename T /*= float_type*/>
	inline void BSplineCurve<NDIM, T>::init_by_fit(
		const MATRIX_t<T>& pts, const VECTOR_t<T>& ts, int n_degree, 
		double w_smooth, double end_point_multiplicity)
	{
		assert_throw(pts.rows() == ts.size(), "number of parameter is different from number of sites");
		m_tmin = ts.minCoeff();
		m_tmax = ts.maxCoeff();

		int n_endpt = 1;
		if (end_point_multiplicity >= 1.0)
			n_endpt = (int)std::round(end_point_multiplicity);
		else
			n_endpt = (int)(pts.rows() * end_point_multiplicity);
		
		if (n_endpt < 1)
			n_endpt = 1;

		m_splines.clear();

		//find end point index
		Eigen::Index idx_start, idx_end;
		auto t_end = ts.maxCoeff(&idx_end);
		auto t_start = ts.minCoeff(&idx_start);

		for (int dim = 0; dim < NDIM; dim++) {
			SPLINTER::DataTable samples(true);

			auto y = pts.col(dim);
			for (int i = 0; i < ts.size(); i++) {
				samples.addSample(ts(i), y(i));
			}

			//add end point multiplicity
			for (int i = 0; i < n_endpt - 1; i++) {
				samples.addSample(t_start, y(idx_start));
				samples.addSample(t_end, y(idx_end));
			}

			//fit
			auto spline = SPLINTER::BSpline::Builder(samples)
				.degree(n_degree)
				.smoothing(SPLINTER::BSpline::Smoothing::PSPLINE)
				.alpha(w_smooth)
				.build();
			m_splines.push_back(spline);
		}
	}

}