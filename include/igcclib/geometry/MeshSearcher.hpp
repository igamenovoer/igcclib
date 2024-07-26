#pragma once
#include "TriangularMesh.hpp"
#include <igcclib/geometry/igcclib_cgal.hpp>

namespace _NS_UTILITY
{
	/// <summary>
	/// Search structures that operate on a mesh
	/// </summary>
	class MeshSearcher
	{
	private:
		//the mesh
		//const TriangularMesh* m_mesh;
		std::shared_ptr<TriangularMesh> m_mesh;

		//the aabb tree
		std::shared_ptr<FACETREE_3> m_aabb_tree;

		//triangle list for use with aabb tree
		std::shared_ptr<std::vector<TRI_3>> m_trilist;

	// management
	public:
		/// <summary>
		/// update all query structures from the mesh
		/// </summary>
		void update_query_structure();

		/// <summary>
		/// set the triangular mesh in the query structure
		/// </summary>
		/// <param name="mesh">the triangular mesh</param>
		/// <param name="update">should we update the query structure?</param>
		void set_mesh(const TriangularMesh& mesh, bool update = true,bool make_copy = false);

		/** \brief get the mesh which is used to create this searcher */
		const TriangularMesh* get_mesh() const {
			return m_mesh.get();
		}

		/** \brief get the mesh which is used to create this searcher */
		TriangularMesh* get_mesh() {
			return m_mesh.get();
		}

		/// <summary>
		/// get the list of all triangles of the input mesh
		/// </summary>
		/// <returns>the list of all triangles</returns>
		const std::vector<TRI_3>& get_triangle_list() const {
			return *m_trilist;	
		}

	//query
	public:
		/// <summary>
		/// for each query point, find the closest point on the mesh
		/// </summary>
		/// <param name="pts">the query points, nx3 matrix</param>
		/// <param name="out_nnpts">output nearest point for each query point</param>
		/// <param name="out_idxtri">the index of the triangle where the nearest point lies in</param>
		/// <param name="out_bc_pts">the barycentric coordinate of each nearest point</param>
		void find_closest_point(const fMATRIX& pts, fMATRIX* out_nnpts,
			iVECTOR* out_idxtri = 0, fMATRIX* out_bc_pts =0);

		/**
		* \brief find the closest point on the mesh to a given point.
		*
		* \param p the point in space whose closest point on the mesh will be found
		* \param out_closest_point the closest point on the mesh
		* \param out_idxtri the index of the triangle where the nearest point lies in 
		* \param out_barycentric the barycentric coordinate of the closest point
		*/
		void find_closest_point(const POINT_3& p, POINT_3* out_closest_point, 
			int_type* out_idxtri, POINT_3* out_barycentric = nullptr);

		/**
		* \brief find the closest point on the mesh to a given point.
		*
		* \param p the point in space whose closest point on the mesh will be found
		* \param out_closest_point the closest point on the mesh
		* \param out_idxtri the index of the triangle where the nearest point lies in
		* \param out_barycentric the barycentric coordinate of the closest point
		*/
		void find_closest_point(const fVECTOR_3& p, fVECTOR_3* out_closest_point,
			int_type* out_idxtri, fVECTOR_3* out_barycentric = nullptr);

		/// <summary>
		/// find ray intersection with the mesh, return the first hit points.
		/// </summary>
		/// <param name="p0">nx3, origins of the rays</param>
		/// <param name="dirs">nx3, directions of the rays</param>
		/// <param name="out_hitpts">nx3, output hit points of each row</param>
		/// <param name="out_idxtri">1xn, output indices of the triangles that contain the hit points. If a ray has no hit point, out_idxtri[i]=-1</param>
		/// <param name="out_bcpts">nx3, barycentric coordinates of the hitpts in their triangles</param>
		void intersect_with_ray_first(const fMATRIX& p0, const fMATRIX& dirs, 
			fMATRIX* out_hitpts, iVECTOR* out_idxtri =0, fMATRIX* out_bcpts =0);

		/// <summary>
		/// find single ray intersection with the mesh, return the first hit point.
		/// </summary>
		/// <param name="ray">the query ray</param>
		/// <param name="out_hitpoint">output hit point</param>
		/// <param name="out_idxtri">index of the triangle that contains the hit point. Equal to -1 if no hit point is found.</param>
		/// <param name="out_barycentric">the output barycentric coordinate</param>
		void intersect_with_ray_first(const RAY_3& ray, POINT_3* out_hitpoint, int_type* out_idxtri, POINT_3* out_barycentric = nullptr);

	public:
		/// <summary>
		/// create search structure from a mesh
		/// </summary>
		/// <param name="output">the output object</param>
		/// <param name="mesh">the triangular mesh</param>
		static void init_with_triangular_mesh(MeshSearcher& output, const TriangularMesh& mesh);

		//deprecated
		/// <summary>
		/// create search structure from a mesh
		/// </summary>
		/// <param name="mesh">the triangular mesh</param>
		/// <returns>the created search structure</returns>
		// static MeshSearcher init_with_triangular_mesh(const TriangularMesh& mesh);

		/// <summary>
		/// create search structure from vertex and face		
		/// </summary>
		/// <param name="output">the output object</param>
		/// <param name="vertices">nx3 vertices</param>
		/// <param name="faces">nx3 faces</param>
		static void init_with_vertex_face(MeshSearcher& output, const fMATRIX& vertices, const iMATRIX& faces);

	public:
		MeshSearcher()
		{
			m_trilist = std::make_shared<std::vector<TRI_3>>();
		}


		virtual ~MeshSearcher()
		{
		}
	};
};

