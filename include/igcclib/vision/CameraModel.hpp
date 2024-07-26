#pragma once

#include <igcclib/core/igcclib_eigen.hpp>
#include <igcclib/geometry/igcclib_geometry.hpp>

namespace _NS_UTILITY
{
	/** \brief a camera with extrinsic and intrinsc parameters.
	Typically, the camera looks at z+, with x+ right, y+ down, following the opencv convention. 
	Though it is all up to the intrinsic and extrinsic matrices. */
	class CameraModel {
	public:
		/** \brief right-mul projection matrix, will project points
		in camera coordinate to 2d locations.
		For orthographic projection, the 3rd column is [0,0,0].
		*/
		fMATRIX_3 m_projection_matrix = fMATRIX_3::Identity();

		/** \brief right-mul extrinsic matrix which maps world points
		to camera coordinate */
		fMATRIX_4 m_extrinsic_matrix = fMATRIX_4::Identity();

		/** \brief distortion coefficients, following opencv convention, see cv::projectPoints().
		The complete form is k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,tx,ty. It is possible that
		certain coefficients do not present, leaving the vector to have 4,5,8,12 or 14 elements.*/
		fVECTOR m_distortion;
	
		/** \brief image size, only valid when the camera produces an image */
		int m_width = 0;
		int m_height = 0;

	public:
		virtual ~CameraModel() {}

		virtual void copy_to(CameraModel* output) const;

		int get_image_width() const { return m_width; }
		int get_image_height() const { return m_height; }
		void set_image_size(int width, int height) { m_width = width; m_height = height; }

		/**
		* \brief project 3d points in camera coordinate to 2d coordinate
		*
		* \param pts input 3d points
		* \return the projected 2d points
		*/
		fMATRIX project_points(const fMATRIX& pts, bool with_distortion = true, bool preserve_z = false) const;

		/** \brief project a single 3d point to 2d */
		fVECTOR_2 project_points(const fVECTOR_3& p, bool with_distortion = true) const;

		/** \brief convert a point in world coordinate to camera coordinate */
		fVECTOR_3 convert_world_to_camera(const fVECTOR_3& p) const;

		/** \brief convert a point in camera coordinate to world coordinate */
		fVECTOR_3 convert_camera_to_world(const fVECTOR_3& p) const;

		/** \brief convert a projected point to 3d ray, without considering distortion */
		void ray_from_projected_points(
			const fVECTOR_2& p, fVECTOR_3* out_ray_p0, fVECTOR_3* out_ray_dir) const;

		/** \brief get fov in width, in degree */
		double get_fov_width_degree() const;

		/** \brief get fov in height, in degree */
		double get_fov_height_degree() const;

		/** \brief get the right-mul projection matrix, will project points
		in camera coordinate to 2d locations. The last column must be [0,0,0,1].
		For orthographic projection, the 3rd column is [0,0,0,1]. */
		const fMATRIX_3& get_projection_matrix() const {
			return m_projection_matrix;
		}

		/** \brief get the world position of the camera */
		fVECTOR_3 get_position() const {
			fVECTOR_3 output = m_extrinsic_matrix.inverse().row(3).leftCols(3).transpose();
			return output;
		}

		/** \brief get the viewing direction, which is z+ of the local axis of the camera */
		fVECTOR_3 get_view_direction() const {
			fVECTOR_3 output = m_extrinsic_matrix.inverse().row(2).leftCols(3).transpose().normalized();
			return output;
		}

		/** \brief set the projection matrix */
		void set_projection_matrix(const fMATRIX_3& mat) {
			m_projection_matrix = mat;
		}

		const fMATRIX_4& get_extrinsic_matrix() const {
			return m_extrinsic_matrix;
		}

		const fVECTOR& get_distortion_coefficient() const {
			return m_distortion;
		}

		void set_distortion_coefficient(const fVECTOR& distcoef);

		/** \brief set the extrinsic matrix, which is a right-mul matrix that 
		transforms world position to camera position */
		void set_extrinsic_matrix(const fMATRIX_4& tmat) {
			m_extrinsic_matrix = tmat;
		}

		bool is_orthographic() const {
			return m_projection_matrix.col(2).squaredNorm() < 1e-8;
		}

		template<typename Archive_t>
		void serialize(Archive_t& ar) {
			ar(m_extrinsic_matrix);
			ar(m_projection_matrix);
			ar(m_distortion);
		}

	public:
		static CameraModel OrthographicModel() {
			CameraModel model;
			model.m_projection_matrix.row(2).fill(0);
			return model;
		}
	
		/**
		* \brief construct projection matrix given the fov along image width
		*
		* \param fov_rad the fov in degree
		* \param width image width
		* \param height image height
		* \return fMATRIX_3 the right-mul projection matrix
		*/
		static fMATRIX_3 projection_matrix_by_fov_width(double fov_deg, double width, double height);

		/**
		* \brief construct projection matrix given the fov along image height
		*
		* \param fov_rad the fov in degree
		* \param width image width
		* \param height image height
		* \return fMATRIX_3 the right-mul projection matrix
		*/
		static fMATRIX_3 projection_matrix_by_fov_height(double fov_deg, double width, double height);

		/**
		* \brief construct an extrinsic matrix to make the camera look at a target, 
		such that the target is at the center of view, and the up_vector (in world space)
		is mapped to the up direction (pointing from image bottom to top) in the resulting image.
		The camera coordinate follows opencv convention, that is, look at z+, x+ right, y+ down.
		*
		* \param camera_position the camera position
		* \param target the target point
		* \param up_vector the up vector in world space
		*/
		static fMATRIX_4 extrinsic_matrix_by_look_at(
			const fVECTOR_3& camera_position, const fVECTOR_3& target, const fVECTOR_3& up_vector);
	};
}