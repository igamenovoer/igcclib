#pragma once

#include <list>
#include <set>

#include <igcclib/core/igcclib_eigen.hpp>

namespace _NS_UTILITY
{
	// ================= delarations ============================
	/// <summary>
	/// transform points by a matrix, by pts.dot(transmat)
	/// </summary>
	/// <param name="pts">N-d points</param>
	/// <param name="transmat">(N+1)-d matrix</param>
	/// <param name="output">output points, can be the same object as input</param>
	template<typename T, typename TMAT>
	inline void transform_points(const MATRIX_t<T>& pts, const TMAT& transmat, MATRIX_t<T>& output);

	/** \brief fit a circle to 2d points */
	inline void fit_circle_2(const fMATRIX& pts, fVECTOR_2* out_center, float_type* out_radius);

	/** \brief transform points by a right-mul matrix, by pts.dot(transmat) */
	template<typename T, typename TMAT>
	inline MATRIX_t<T> transform_points(const MATRIX_t<T>& pts, const TMAT& transmat);

	/** \brief transform a single 3d point */
	inline fVECTOR_3 transform_single_point(const fVECTOR_3& p, const fMATRIX_4& transmat);

	/** \brief transform a 3d vector using only the upper-left 3x3 part of the transformation matrix */
	inline fVECTOR_3 transform_single_vector(const fVECTOR_3& p, const fMATRIX_4& transmat);

	/** \brief transform vectors by a transformation matrix, only the affine part of the transformation matrix will be used. */
	template<typename T, typename TMAT>
	inline void transform_vectors(const MATRIX_t<T>& vecs, const TMAT& transmat, MATRIX_t<T>& output);

	/** \brief transform vectors by a transformation matrix, the vector will be multiplied by transmat.T.inv */
	template<typename T, typename TMAT>
	inline MATRIX_t<T> transform_vectors(const MATRIX_t<T>& vecs, const TMAT& transmat);

	/// <summary>
	/// for each row, divide its magnitude
	/// </summary>
	/// <param name="mat">the input Eigen matrix, will be modified</param>
	template<typename MAT>
	inline void normalize_rows(MAT& mat);

	/// <summary>
	/// create 4x4 right-multiply translation matrix
	/// </summary>
	/// <param name="x">translation in x</param>
	/// <param name="y">translation in y</param>
	/// <param name="z">translation in z</param>
	/// <returns>the right-multiply translation matrix</returns>
	template<typename T>
	inline MATRIX_4t<T> translation_matrix(double x, double y, double z);

	/// <summary>
	/// create 4x4 right-multiply translation matrix
	/// </summary>
	/// <param name="delta">the translation vector</param>
	/// <returns>the right-multiply translation matrix</returns>
	template<typename T>
	inline MATRIX_4t<T> translation_matrix(VECTOR_3t<T> delta);

	/// <summary>
	/// create 4x4 right-multiply rotation matrix
	/// </summary>
	/// <param name="angle">rotation angle in rad</param>
	/// <param name="axis">the rotation axis</param>
	/// <returns>the 4x4 right-multiply rotation matrix</returns>
	template<typename T>
	inline MATRIX_4t<T> rotation_matrix(double angle, VECTOR_3t<T> axis);

	/// <summary>
	/// create 4x4 right-multiply rotation matrix
	/// </summary>
	/// <param name="x_rad">rotation angle by x_axis in rad</param>
	/// <param name="y_rad">rotation angle by y_axis in rad</param>
	/// <param name="z_rad">rotation angle by z_axis in rad</param>
	/// <returns>the 4x4 right-multiply rotation matrix</returns>
	inline fMATRIX_4 rotation_matrix_by_angles(double x_rad, double y_rad, double z_rad);

	/// <summary>
	/// create 4x4 right-multiply rotation matrix
	/// </summary>
	/// <param name="angle">rotation angle in rad</param>
	/// <param name="axis">the rotation axis</param>
	/// <param name="pivot">the center of rotation</param>
	/// <returns>the 4x4 right-multiply rotation matrix</returns>
	template<typename T>
	inline MATRIX_4t<T> rotation_matrix(double angle, VECTOR_3t<T> axis, VECTOR_3t<T> pivot);


	/**
	* \brief create a rotation matrix which rotates the object in yaw-pitch-roll order.
	* where yaw is rotating around the up vector, pitch round the right, yaw around the front
	*
	* \param yaw	rotation around the front
	* \param pitch	rotation around the right
	* \param roll	rotation around the up
	* \return the 4x4 right-mul rotation matrix
	*/
	template<typename T>
	inline MATRIX_4t<T> rotation_matrix(double yaw, double pitch, double roll, 
		const VECTOR_3t<T>& front, const VECTOR_3t<T>& right, const VECTOR_3t<T>& up);

	/// <summary>
	/// create 4x4 right-multiply scale matrix
	/// </summary>
	/// <param name="scale_vec">scale int x,y,z axis</param>
	/// <returns>the 4x4 right-multiply scale matrix</returns>
	template<typename T>
	inline MATRIX_4t<T> scale_matrix(const VECTOR_3t<T>& scale_vec);

	template<typename T>
	inline  MATRIX_4t<T> transformation_matrix(const VECTOR_3t<T>& trans_vec, const VECTOR_3t<T>& rot_angles_rad, const VECTOR_3t<T>& scale_vec);


	/*find a similarity transformation that maps pts_1 to pts_2, which means
	pts_2 = pts_1.dot(R)*s + t.

	The method is described in :
	S.Umeyama, "Least-squares estimation of transformation parameters between two point patterns,"
	in IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 13, no. 4, pp. 376 - 380, Apr 1991.

	return transmat, 4x4 matrix for 3d points*/
	template<typename T>
	inline MATRIX_4t<T> _find_transform_similarity(const MATRIX_t<T>& pts_1, const MATRIX_t<T>& pts_2);

	/**
	* \brief find 3x3 right-multiply matrix that transforms points in camera coordinate to image coordinate.
	* the camera matrix has projective freedom (8 degrees of freedom).
	*
	* \param pts_camera 3d points in camera coordinate
	* \param pts_image_xy 2d points in image xy coordinate
	* \return the 3x3 camera matrix M such that pts*M projects the points to image coordinate
	*/
	template<typename T>
	MATRIX_t<T> find_camera_matrix_projective(const MATRIX_t<T>& pts_camera, const MATRIX_t<T>& pts_image_xy);

	/** \brief find a transformation that transforms pts_1 to pts_2 */
	template<typename T>
	MATRIX_t<T> find_transform_projective(const MATRIX_t<T>& pts_1, const MATRIX_t<T>& pts_2);

	template<typename T>
	inline MATRIX_t<T> find_transform_similarity(const MATRIX_t<T>& pts_1, const MATRIX_t<T>& pts_2);

	template<typename T>
	inline MATRIX_t<T> find_transform_rotate_translate(const MATRIX_t<T>& pts_1, const MATRIX_t<T>& pts_2);

	/// <summary>
	/// create a right-handed coordinate frame given z direction, and possibly x direction as well.
	/// </summary>
	template<typename T>
	inline MATRIX_3t<T> make_frame(const VECTOR_3t<T>& zdir, const VECTOR_3t<T>* xdir = 0);

	/// <summary>
	/// create laplacian matrix from a mesh
	/// </summary>
	/// <param name="output">the output laplacian matrix</param>
	/// <param name="faces">the faces of the mesh</param>
	/// <param name="n_vert">number of vertices</param>
	/// <param name="normalize">should we normalize each row of the laplacian so that the negative
	/// values sum to one?</param>
	/// <param name="flatten_coordinate">If false, the output laplacian matrix is NxN where N is the 
	/// number of vertices, which can be used to left-multiply with the Nx3 vertices. If true, the 
	/// vertex coordinates are flattened into [x,y,z,x,y,z...] or [x,x,..,y,y,..z,z], 
	/// and the output laplacian matrix is DNxDN, D=vertex dimension</param>
	/// <param name="n_vert_dim">number of vertex dimensions, applicable when flatten_coordinate=true</param>
	/// <param name="vertex_order">only applicable when flatten_coordinate=true, this determines
	/// the flattened coordinate order</param>
	template<typename T>
	void laplacian_from_mesh(SP_MATRIX_t<T>& output, const iMATRIX& faces,
		size_t n_vert, bool normalize = false, bool flatten_coordinate = false,
		int n_vert_dim = -1,
		VertexOrderInFormulation vertex_order = VertexOrderInFormulation::XYZXYZ);

	/**
	* \brief project a list of points to a plane
	*
	* \param output the projected points
	* \param pts	the points to be projected
	* \param p0	a point on the plane
	* \param normal	the normal of the plane
	*/
	template<typename T>
	void project_points_to_plane(MATRIX_t<T>* output,
		const MATRIX_t<T>& pts, const VECTOR_xt<T, 3>& p0,
		const VECTOR_xt<T, 3>& normal);

	/**
	* \brief project a list of points to a plane
	*
	* \param pts	the points to be projected
	* \param p0	a point on the plane
	* \param normal	the normal of the plane
	* \return the projected points
	*/
	template<typename T>
	MATRIX_t<T> project_points_to_plane(
		const MATRIX_t<T>& pts, const VECTOR_xt<T, 3>& p0,
		const VECTOR_xt<T, 3>& normal);

	/**
	* \brief find intersection between a ray and a plane
	*
	* \param output the intersection point
	* \param ray_p0	a point on the ray
	* \param ray_dir the ray direction
	* \param plane_p0	a point on the plane
	* \param plane_normal	the plane normal
	* \return bool	whether there is an intersection point
	*/
	template<typename T>
	bool intersect_ray_plane(VECTOR_3t<T>* output, 
		const VECTOR_3t<T>& ray_p0, const VECTOR_3t<T>& ray_dir, 
		const VECTOR_3t<T>& plane_p0, const VECTOR_3t<T>& plane_normal);
}

namespace _NS_UTILITY
{
	template<typename T>
	inline MATRIX_4t<T> rotation_matrix(double yaw, double pitch, double roll,
		const VECTOR_3t<T>& front, const VECTOR_3t<T>& right, const VECTOR_3t<T>& up)
	{
		auto yaw_mat = rotation_matrix(yaw, up);
		auto pitch_mat = rotation_matrix(pitch, right);
		auto roll_mat = rotation_matrix(roll, front);

		MATRIX_4t<T> rotmat = roll_mat * pitch_mat * yaw_mat;
		return rotmat;
	}

	template<typename T>
	MATRIX_4t<T> scale_matrix(const VECTOR_3t<T>& scale_vec)
	{
		MATRIX_4t<T> scale_mat;
		scale_mat.setZero();
		scale_mat(0, 0) = scale_vec(0);
		scale_mat(1, 1) = scale_vec(1);
		scale_mat(2, 2) = scale_vec(2);
		scale_mat(3, 3) = 1.0;
		return scale_mat;
	}

	template<typename T>
	bool intersect_ray_plane(VECTOR_3t<T>* output,
		const VECTOR_3t<T>& ray_p0, const VECTOR_3t<T>& ray_dir,
		const VECTOR_3t<T>& plane_p0, const VECTOR_3t<T>& plane_normal) {

		auto a = ray_dir.dot(plane_normal);
		auto b = (plane_p0 - ray_p0).dot(plane_normal);

		if (std::abs(a) < 1e-8) //ray is parallel to plane
			return false;
		auto t = b / a;
		if (t < 0) //plane is in the back of the ray
			return false;

		if (output)
			*output = ray_p0 + ray_dir * t;
		return true;
	}

	template<typename T>
	void project_points_to_plane(MATRIX_t<T>* output,
		const MATRIX_t<T>& pts, const VECTOR_xt<T, 3>& p0,
		const VECTOR_xt<T, 3>& normal)
	{
		output->resize(pts.rows(), 3);
		for (Eigen::Index i = 0; i < pts.rows(); i++) {
			VECTOR_xt<T, 3> p = pts.row(i);
			output->row(i) = p + (p0 - p).dot(normal) * normal;
		}
	}

	template<typename T>
	MATRIX_t<T> project_points_to_plane(
		const MATRIX_t<T>& pts, const VECTOR_xt<T, 3>& p0,
		const VECTOR_xt<T, 3>& normal)
	{
		MATRIX_t<T> output;
		project_points_to_plane(&output, pts, p0, normal);
		return output;
	}

	fVECTOR_3 transform_single_vector(const fVECTOR_3& p, const fMATRIX_4& transmat) {
		fVECTOR_3 output = transmat.block(0, 0, 3, 3).transpose() * p;
		return output;
	}

	fVECTOR_3 transform_single_point(const fVECTOR_3& p, const fMATRIX_4& transmat)
	{
		fVECTOR_4 q(p[0], p[1], p[2], 1.0);
		q = transmat.transpose() * q;
		q /= q[3];
		fVECTOR_3 output(q[0], q[1], q[2]);
		return output;
	}

	template<typename T, typename TMAT>
	inline MATRIX_t<T> transform_vectors(const MATRIX_t<T>& vecs, const TMAT& transmat)
	{
		MATRIX_t<T> output;
		transform_vectors(vecs, transmat, output);
		return output;
	}

	template<typename T, typename TMAT>
	inline MATRIX_t<T> transform_points(const MATRIX_t<T>& pts, const TMAT& transmat)
	{
		MATRIX_t<T> output;
		transform_points(pts, transmat, output);
		return output;
	}

	// ================== implementations =======================
	/// <summary>
	/// transform points by a matrix, by pts.dot(transmat)
	/// </summary>
	/// <param name="pts">N-d points</param>
	/// <param name="transmat">(N+1)-d matrix</param>
	/// <param name="output">output points, can be the same object as input</param>
	template<typename T, typename TMAT>
	inline void transform_points(const MATRIX_t<T>& pts, const TMAT& transmat, MATRIX_t<T>& output)
	{
		auto n_pts = pts.rows();
		auto n_dim = pts.cols();

		//check matrix size
		assert_throw(transmat.rows() == n_dim + 1 && transmat.cols() == n_dim + 1, "transformation matrix does not have proper shape");

		MATRIX_t<T> pts_aug(n_pts, n_dim + 1);
		pts_aug.fill(1);
		pts_aug.block(0, 0, n_pts, n_dim) = pts;
		
		MATRIX_t<T> pts_compute = pts_aug * transmat.template cast<T>();
		pts_compute.array().colwise() /= pts_compute.col(n_dim).array();
		
		output = pts_compute.block(0, 0, n_pts, n_dim);
	}

	/// <summary>
	/// transform vectors by a transformation matrix, the vector will be multiplied by transmat.T.inv
	/// </summary>
	template<typename T, typename TMAT>
	inline void transform_vectors(const MATRIX_t<T>& vecs, const TMAT& transmat, MATRIX_t<T>& output)
	{
		auto ndim = vecs.cols();
		auto nrow = vecs.rows();
		//output = vecs * transmat.block(0, 0, ndim, ndim).transpose().inverse();
		output = vecs * transmat.block(0, 0, ndim, ndim);
	}

	/// <summary>
	/// for each row, divide its magnitude
	/// </summary>
	/// <param name="mat">the input Eigen matrix, will be modified</param>
	template<typename MAT>
	inline void normalize_rows(MAT& mat)
	{
		mat.rowwise().normalize();
	}

	/// <summary>
	/// create 4x4 right-multiply translation matrix
	/// </summary>
	/// <param name="x">translation in x</param>
	/// <param name="y">translation in y</param>
	/// <param name="z">translation in z</param>
	/// <returns>the right-multiply translation matrix</returns>
	template<typename T>
	inline MATRIX_4t<T> translation_matrix(double x, double y, double z)
	{
		Eigen::Transform<T, 3, Eigen::Affine> tform;
		tform = Eigen::Translation<T, 3>(x, y, z);
		return tform.matrix().transpose();
	}

	/// <summary>
	/// create 4x4 right-multiply translation matrix
	/// </summary>
	/// <param name="delta">the translation vector</param>
	/// <returns>the right-multiply translation matrix</returns>
	template<typename T>
	inline MATRIX_4t<T> translation_matrix(VECTOR_3t<T> delta)
	{
		Eigen::Transform<T, 3, Eigen::Affine> tform;
		tform = Eigen::Translation<T, 3>(delta(0), delta(1), delta(2));
		return tform.matrix().transpose();
	}

	/// <summary>
	/// create 4x4 right-multiply rotation matrix
	/// </summary>
	/// <param name="angle">rotation angle in rad</param>
	/// <param name="axis">the rotation axis</param>
	/// <returns>the 4x4 right-multiply rotation matrix</returns>
	template<typename T>
	inline MATRIX_4t<T> rotation_matrix(double angle, VECTOR_3t<T> axis)
	{
		Eigen::Transform<T, 3, Eigen::Affine> tform;
		tform = Eigen::AngleAxis<T>(angle, axis.normalized());
		return tform.matrix().transpose();
	}

	/// <summary>
	/// create 4x4 right-multiply rotation matrix
	/// </summary>
	/// <param name="angle">rotation angle in rad</param>
	/// <param name="axis">the rotation axis</param>
	/// <param name="pivot">the center of rotation</param>
	/// <returns>the 4x4 right-multiply rotation matrix</returns>
	template<typename T>
	inline MATRIX_4t<T> rotation_matrix(double angle, VECTOR_3t<T> axis, VECTOR_3t<T> pivot)
	{
		Eigen::Transform<T, 3, Eigen::Affine> tform;
		Eigen::Translation<T, 3> trans(pivot);
		Eigen::AngleAxis<T> rot(angle, axis.normalized());
		tform = trans * rot * trans.inverse();
		return tform.matrix().transpose();
	}

	inline fMATRIX_4 rotation_matrix_by_angles(double x_rad, double y_rad, double z_rad)
	{
		fVECTOR_3 axis_x(1, 0, 0);
		auto r_mat_x = rotation_matrix(x_rad, axis_x);
		
		fVECTOR_3 axis_y(0, 1, 0);
		auto r_mat_y = rotation_matrix(y_rad, axis_y);

		fVECTOR_3 axis_z(1, 0, 0);
		auto r_mat_z = rotation_matrix(z_rad, axis_z);
		
		return r_mat_x * r_mat_y * r_mat_z;
	}


	template<typename T>
	inline  MATRIX_4t<T>
		transformation_matrix(const VECTOR_3t<T>& trans_vec, const VECTOR_3t<T>& rot_angles_rad, const VECTOR_3t<T>& scale_vec)
	{
		auto r = rotation_matrix_by_angles(rot_angles_rad(0), rot_angles_rad(1), rot_angles_rad(2));
		auto s = scale_matrix(scale_vec);
		auto t = translation_matrix(trans_vec);
		return s * r * t;
	}

	/*find a similarity transformation that maps pts_1 to pts_2, which means
	pts_2 = pts_1.dot(R)*s + t.

	The method is described in :
	S.Umeyama, "Least-squares estimation of transformation parameters between two point patterns,"
	in IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 13, no. 4, pp. 376 - 380, Apr 1991.

	return transmat, 4x4 matrix for 3d points*/
	template<typename T>
	inline MATRIX_4t<T> _find_transform_similarity(const MATRIX_t<T>& pts_1, const MATRIX_t<T>& pts_2)
	{
		using fmatrix = MATRIX_t<T>;
		using fmatrix_4 = MATRIX_4t<T>;
		using fmatrix_3 = MATRIX_3t<T>;
		using fvector_3_row = Eigen::Matrix<T, 1, 3, Eigen::RowMajor>;

		auto ndim = pts_1.cols();
		if (ndim != 3)
			throw std::logic_error("Input pts must be 3 dim points!");

		fvector_3_row c1 = pts_1.colwise().mean();
		fmatrix p1 = pts_1.rowwise() - c1;
		double var1 = p1.array().pow(2).rowwise().sum().mean();

		fvector_3_row c2 = pts_2.colwise().mean();
		fmatrix p2 = pts_2.rowwise() - c2;
		double var2 = p2.array().pow(2).rowwise().sum().mean();

		fmatrix_3 sigmat = p2.transpose()*p1 / p1.rows();		

		//std::cout << sigmat << "\n\n";

		Eigen::JacobiSVD<fmatrix_3> svd(sigmat, Eigen::ComputeFullU | Eigen::ComputeFullV);
		fmatrix_3 U = svd.matrixU();
		fmatrix_3 V = svd.matrixV(); //V is not Vt, V is Vt transposed, different from python svd
		fmatrix_3 D = svd.singularValues().asDiagonal();

		fmatrix S = fmatrix_3::Identity();
		auto sigrank = svd.rank();
		if (sigrank == ndim)
		{
			//full rank
			if (sigmat.determinant() < 0)
				S(2, 2) = -1;
		}
		else if (sigrank == ndim - 1)
		{
			double dval = U.determinant() * V.determinant();
			if (dval < 0)
				S(2, 2) = -1;
		}
		else
			throw std::logic_error("rank is less than ndim-1, similarity transformation cannot be found");

		//create left - multiply transformation matrix
		fmatrix_3 rotmat = U * S * V.transpose();
		double scale = 1.0 / var1 * (D * S).diagonal().sum();
		fvector_3_row tranvec = c2.transpose() - scale * rotmat * c1.transpose();

		fmatrix_4 transmat = fmatrix_4::Identity();
		transmat.block(0, 0, 3, 3) = scale * rotmat;
		transmat.block(0, 3, 3, 1) = tranvec.transpose();
		return transmat.transpose();
	}

	/**
	* \brief find 3x3 right-multiply matrix that transforms points in camera coordinate to image coordinate.
	* the camera matrix has projective freedom (8 degrees of freedom).
	*
	* \param pts_camera 3d points in camera coordinate
	* \param pts_image_xy 2d points in image xy coordinate
	* \return the 3x3 camera matrix M such that pts*M projects the points to image coordinate
	*/
	template<typename T>
	MATRIX_t<T> find_camera_matrix_projective(const MATRIX_t<T>& pts_camera, const MATRIX_t<T>& pts_image_xy)
	{
		using MAT_TYPE = MATRIX_t<T>;
		using VEC_TYPE = VECTOR_t<T>;

		//the formulation uses left-multiply camera matrix, we transpose it in the output

		auto npts = pts_camera.rows();
		MAT_TYPE A(2 * npts, 8);
		A.fill(0);
		VEC_TYPE b(2 * npts);
		b.fill(0);
		for (int i = 0; i < npts; i++) {
			auto x = pts_camera(i, 0);
			auto y = pts_camera(i, 1);
			auto z = pts_camera(i, 2);
			auto u = pts_image_xy(i, 0);
			auto v = pts_image_xy(i, 1);

			{
				auto r = A.row(2 * i);
				r(0) = x;
				r(1) = y;
				r(2) = z;
				r(6) = -u * x;
				r(7) = -u * y;
				b(2 * i) = z * u;
			}

			{
				auto r = A.row(2 * i + 1);
				r(3) = x;
				r(4) = y;
				r(5) = z;
				r(6) = -v * x;
				r(7) = -v * y;
				b(2 * i + 1) = z * v;
			}
		}

		//solve
		MAT_TYPE ata = A.transpose() * A;
		VEC_TYPE atb = A.transpose() * b;
		VEC_TYPE sol = ata.llt().solve(atb);

		//output
		MATRIX_3t<T> camera_matrix = MATRIX_3t<T>::Identity();
		for (int i = 0; i < 8; i++)
			camera_matrix.data()[i] = sol.data()[i];
		MATRIX_3t<T> output = camera_matrix.transpose();
		return output;
	}

	/** \brief find a transformation that transforms pts_1 to pts_2 */
	template<typename T>
	MATRIX_t<T> find_transform_projective(const MATRIX_t<T>& pts_1, const MATRIX_t<T>& pts_2)
	{
		int ndim = pts_1.cols();
		int npts = pts_1.rows();

		MATRIX_t<T> pts_1_aug(npts, ndim + 1);
		pts_1_aug.block(0, 0, npts, ndim) = pts_1;
		pts_1_aug.col(ndim).array() = 1;

		MATRIX_t<T> pts_2_aug(npts, ndim + 1);
		pts_2_aug.block(0, 0, npts, ndim) = pts_2;
		pts_2_aug.col(ndim).array() = 1;

		MATRIX_t<T> ata = pts_1_aug.transpose() * pts_1_aug;
		MATRIX_t<T> atb = pts_1_aug.transpose() * pts_2_aug;
		MATRIX_t<T> tmat = ata.llt().solve(atb);

		return tmat;
	}

	template<typename T>
	inline MATRIX_t<T> find_transform_rotate_translate(const MATRIX_t<T>& pts_1, const MATRIX_t<T>& pts_2)
	{
		using MATRIX_TYPE = MATRIX_t<T>;
		using VECTOR_TYPE = VECTOR_t<T>;

		auto ndim = pts_1.cols();

		VECTOR_TYPE center_1 = pts_1.colwise().mean();
		auto p1 = (pts_1.rowwise() - center_1.transpose()).eval();

		VECTOR_TYPE center_2 = pts_2.colwise().mean();
		auto p2 = (pts_2.rowwise() - center_2.transpose()).eval();

		MATRIX_TYPE covmat = (p1.transpose() * p2).eval();
		auto svd = covmat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
		auto u = svd.matrixU().eval();
		auto v = svd.matrixV().eval();
		auto d = (v * u.transpose()).determinant();

		VECTOR_TYPE s = VECTOR_TYPE::Ones(ndim);
		s(ndim - 1) = d > 0 ? 1 : -1;
		auto rotmat = v * s.asDiagonal() * u.transpose();

		MATRIX_TYPE tmat_translate_to_origin = MATRIX_TYPE::Identity(ndim + 1, ndim + 1);
		tmat_translate_to_origin.leftCols(3).bottomRows(1) = -center_1.transpose();

		MATRIX_TYPE tmat_translate_to_pts_2 = MATRIX_TYPE::Identity(ndim + 1, ndim + 1);
		tmat_translate_to_pts_2.leftCols(3).bottomRows(1) = center_2.transpose();

		MATRIX_TYPE rotmat_44 = MATRIX_TYPE::Identity(ndim + 1, ndim + 1);
		rotmat_44.block(0, 0, ndim, ndim) = rotmat.transpose();	//right-mul format

		auto output = (tmat_translate_to_origin * rotmat_44 * tmat_translate_to_pts_2).eval();
		return output;
	}

	template<typename T>
	inline MATRIX_t<T> find_transform_similarity(const MATRIX_t<T>& pts_1, const MATRIX_t<T>& pts_2)
	{
		using fmatrix = MATRIX_t<T>;
		using fmatrix_4 = MATRIX_4t<T>;
		using fmatrix_3 = MATRIX_3t<T>;

		if (pts_1.cols() == 2)
		{
			int npts = (int)pts_1.rows();
			fmatrix p1(npts, 3);
			p1.setZero();
			p1.block(0, 0, npts, 2) = pts_1;

			fmatrix p2(npts, 3);
			p2.setZero();
			p2.block(0, 0, npts, 2) = pts_2;
			fmatrix_4 tmat = _find_transform_similarity(p1, p2);
			fmatrix_3 output = fmatrix_3::Identity();
			output.block(0, 0, 2, 2) = tmat.block(0, 0, 2, 2);
			output.block(2, 0, 1, 2) = tmat.block(3, 0, 1, 2);
			return output;
		}
		return _find_transform_similarity(pts_1, pts_2);
	}

	/// <summary>
	/// create a right-handed coordinate frame given z direction, and possibly x direction as well.
	/// </summary>
	template<typename T>
	inline MATRIX_3t<T> make_frame(const VECTOR_3t<T>& zdir, const VECTOR_3t<T>* xdir)
	{
		VECTOR_3t<T> dx;
		if (xdir)
			dx = *xdir;
		else
		{
			VECTOR_3t<T> tmp;
			int idxmax, idxmin;
			zdir.cwiseAbs().maxCoeff(&idxmax);
			zdir.cwiseAbs().minCoeff(&idxmin);

			tmp = zdir;
			tmp(idxmax) = zdir(idxmin);
			tmp(idxmin) = zdir(idxmax);
			dx = zdir.cross(tmp);
		}
		
		VECTOR_3t<T> dy = zdir.cross(dx);
		MATRIX_3t<T> frame;
		frame << dx, dy, zdir; //column_stack() because dx,dy,zdir are column vectors
		frame.transposeInPlace();
		frame.normalized();

		return frame;
	}

	inline void fit_circle_2(const fMATRIX& pts, fVECTOR_2* out_center, float_type* out_radius)
	{
		//offset by mean
		fVECTOR_2 center = pts.colwise().mean();
		fMATRIX _pts = pts.rowwise() - center.transpose();

		fVECTOR u = _pts.col(0);
		fVECTOR u2 = u.array().pow(2);

		fVECTOR v = _pts.col(1);
		fVECTOR v2 = v.array().pow(2);

		auto Suv = (u.array() * v.array()).sum();
		auto Suu = u2.sum();
		auto Svv = v2.sum();
		auto Suuv = (u2.array() * v.array()).sum();
		auto Suvv = (u.array() * v2.array()).sum();
		auto Suuu = u.array().pow(3).sum();
		auto Svvv = v.array().pow(3).sum();

		fMATRIX_2 A;
		A << Suu, Suv, Suv, Svv;

		fVECTOR_2 b;
		b << (Suuu + Suvv) / 2.0, (Svvv + Suuv) / 2.0;

		//solve
		fVECTOR sol = A.llt().solve(b);

		fVECTOR_2 circle_center = center + sol;
		float_type radius = (_pts.rowwise() - sol.transpose()).rowwise().norm().mean();

		if (out_center)
			*out_center = circle_center;
		if (out_radius)
			*out_radius = radius;
	}

	/// <summary>
	/// create laplacian matrix from a mesh
	/// </summary>
	/// <param name="output">the output laplacian matrix</param>
	/// <param name="faces">the faces of the mesh</param>
	/// <param name="n_vert">number of vertices</param>
	/// <param name="normalize">should we normalize each row of the laplacian so that the negative
	/// values sum to one?</param>
	/// <param name="flatten_coordinate">If false, the output laplacian matrix is NxN where N is the 
	/// number of vertices, which can be used to left-multiply with the Nx3 vertices. If true, the 
	/// vertex coordinates are flattened into [x,y,z,x,y,z...] or [x,x,..,y,y,..z,z], 
	/// and the output laplacian matrix is DNxDN, D=vertex dimension</param>
	/// <param name="n_vert_dim">number of vertex dimensions, applicable when flatten_coordinate=true</param>
	/// <param name="vertex_order">only applicable when flatten_coordinate=true, this determines
	/// the flattened coordinate order</param>
	template<typename T>
	void laplacian_from_mesh(SP_MATRIX_t<T>& output, const iMATRIX& faces, 
		size_t n_vert, bool normalize, bool flatten_coordinate,
		int n_vert_dim, VertexOrderInFormulation vertex_order)
	{
		if (flatten_coordinate)
			assert_throw(n_vert_dim > 0, "vertex dimension is not set");

		using TRIP = Eigen::Triplet<T>;
		auto n_face_dim = faces.cols();

		//construct the set of all edges
		std::set<std::pair<size_t,size_t>> edges;
		//std::unordered_set<std::pair<size_t, size_t>, boost::hash<std::pair<size_t, size_t>>> edges;
		for (size_t i = 0; i < faces.rows(); i++)
		{
			for (int k = 0; k < n_face_dim; k++)
			{
				auto u = faces(i, k);
				auto v = faces(i, (k + 1) % n_face_dim);
				edges.emplace(std::max(u, v), std::min(u, v));
			}
		}

		//count connection degrees
		std::vector<int> dgs(n_vert, 0);
		for (auto it = edges.begin(); it != edges.end(); ++it)
		{
			dgs[it->first]++;
			dgs[it->second]++;
		}

		//create matrix entries
		std::list<TRIP> trips;
		if (flatten_coordinate)
		{			
			//converting vertex index and vertex dimension to the row index of the matrix
			auto row_from_vindex_xxyyzz = [&](size_t idxv, int dim) {
				return idxv + dim * n_vert;
			};
			auto row_from_vindex_xyzxyz = [&](size_t idxv, int dim) {
				return idxv * n_vert_dim + dim;
			};

			using MAPFUNC = std::function<size_t(size_t, int)>;
			MAPFUNC mapfunc;
			if (vertex_order == VertexOrderInFormulation::XXYYZZ)
				mapfunc = row_from_vindex_xxyyzz;
			else if (vertex_order == VertexOrderInFormulation::XYZXYZ)
				mapfunc = row_from_vindex_xyzxyz;
			else
				assert_throw(false, "unsupported vertex order");

			//add diangonal
			for (size_t i = 0; i < n_vert; i++)
			{
				for (int d = 0; d < n_vert_dim; d++)
				{
					auto idxrow = mapfunc(i, d);
					trips.emplace_back(idxrow, idxrow, normalize ? 1.0 : dgs[i]);
				}
			}

			//add edge
			for (auto it = edges.begin(); it != edges.end(); ++it)
			{
				auto u = it->first;
				auto v = it->second;

				//for each dimension
				for (int d = 0; d < n_vert_dim; d++)
				{
					auto u_idxvar = mapfunc(u, d);
					auto v_idxvar = mapfunc(v, d);

					//add (u,v) to u row
					trips.emplace_back(u_idxvar, v_idxvar, normalize ? -1.0 / dgs[u] : -1.0);

					//add (v,u) to v row
					trips.emplace_back(v_idxvar, u_idxvar, normalize ? -1.0 / dgs[v] : -1.0);
				}
			}

			//create matrix
			output.resize(n_vert * n_vert_dim, n_vert * n_vert_dim);
			output.setFromTriplets(trips.begin(), trips.end());
		}
		else //coordinate not flattened
		{
			//add diagonal
			for (size_t i = 0; i < n_vert; i++)
			{
				trips.emplace_back(i, i, normalize ? 1.0 : dgs[i]);
			}

			//add edges
			for (auto it = edges.begin(); it != edges.end(); ++it)
			{
				auto u = it->first;
				auto v = it->second;
				trips.emplace_back(u, v, normalize ? -1.0 / dgs[u] : -1.0);
				trips.emplace_back(v, u, normalize ? -1.0 / dgs[v] : -1.0);
			}
			output.resize(n_vert, n_vert);
			output.setFromTriplets(trips.begin(), trips.end());
		}
	}
}