#pragma once

#include <igcclib/igcclib_master.hpp>
#include <igcclib/core/igcclib_eigen.hpp>
#include <igcclib/geometry/igcclib_cgal_eigen.hpp>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <boost/property_map/function_property_map.hpp>

namespace _NS_UTILITY
{
	/*!
	 * \class MeshPropertyReader
	 *
	 * \brief reader various mesh properties from CGAL surface mesh
	 *
	 */
	class MeshPropertyReader {
	public:
		using IndexList = std::vector<int_type>;

		/**
		* \brief initialize with faces only, with this initialization you can access functions related to mesh topology
		*
		* \param faces the face array
		* \param num_vertices	number of vertices, if set to -1, then we use faces.max()+1 as the number of vertices
		*/
		void init(int num_vertices, const iMATRIX& faces);

		/** \brief initialize with vertices and faces, the vertices can be 2d or 3d */
		void init(const fMATRIX& vertices, const iMATRIX& faces);

		/** \brief get all border edges, each element is (u,v) denoting an edge from vertex u to vertex v */
		std::vector<iVECTOR_2> get_border_edges() const;

		/** \brief get edges not belonging to the border */
		std::vector<iVECTOR_2> get_non_border_edges() const;

		/** \brief get all border vertices */
		IndexList get_border_vertices() const;

		/** \brief get vertices that are not on the border */
		IndexList get_non_border_vertices() const;

		/** \brief given a face, find all neighboring faces */
		IndexList get_neighbor_faces(int_type idxface) const;

		/** \brief given a vertex, find all neighboring vertices */
		IndexList get_neighbor_vertices(int_type idxv) const;

		/** \brief get a list of connected components, each of which consists of all the vertex indices in that component.
		Note that the connectivity is defined using faces, that is, two vertices are connected only if their faces have shared edge.*/
		std::vector<IndexList> get_connected_components_as_vertices() const;

		/** \brief get connected components containing any of the seed face and seed vertex, returned as a list of vertex index */
		std::vector<IndexList> get_connected_components_as_vertices(
			const IndexList* seed_faces,
			const IndexList* seed_vertices) const;

		/** \brief get a list of connected components, each consisting of all the face indices in that component */
		std::vector<IndexList> get_connected_components_as_faces() const;

		/** \brief get connected components containing any of the seed face or seed vertex, returned as a list of face index */
		std::vector<IndexList> get_connected_components_as_faces(
			const IndexList* seed_faces, 
			const IndexList* seed_vertices) const;

		size_t get_num_vertices() const { return m_mesh.number_of_vertices(); }
		size_t get_num_faces() const { return m_mesh.number_of_faces(); }

		/** \brief get the underlying CGAL mesh */
		const TRIMESH_3& _get_cgal_mesh() const { return m_mesh; }

	protected:
		int m_ndim = 3;
		TRIMESH_3 m_mesh;

		//keep the matrix representation of the mesh for convenience
		iMATRIX m_faces;
		fMATRIX m_vertices;
	};

	inline std::vector<MeshPropertyReader::IndexList> MeshPropertyReader::get_connected_components_as_vertices() const
	{
		auto idxface_conn = get_connected_components_as_faces();
		VECTOR_b vertex_included(get_num_vertices());
		vertex_included.fill(false);

		std::vector<IndexList> output(idxface_conn.size());
		VECTOR_b mask_vertex_in_component(get_num_vertices());
		for (size_t i = 0; i < idxface_conn.size(); i++)
		{
			auto& idxfaces = idxface_conn[i];

			mask_vertex_in_component.fill(false);
			for (auto idxf : idxfaces)
			{
				iVECTOR_3 f = m_faces.row(idxf);
				mask_vertex_in_component(f[0]) = true;
				mask_vertex_in_component(f[1]) = true;
				mask_vertex_in_component(f[2]) = true;
			}

			//get the vertices
			find_nonzeros(mask_vertex_in_component, &output[i]);

			//mask the selected vertices
			for (auto k : output[i])
				vertex_included(k) = true;
		}

		//vertices left out are isolated vertices
		for (Eigen::Index i = 0; i < vertex_included.size(); i++)
		{
			if (!vertex_included(i)) //found an isolated vertex
				output.push_back({ (int_type)i });
		}

		return output;
	}

	inline std::vector<MeshPropertyReader::IndexList>
		MeshPropertyReader::get_connected_components_as_vertices(
			const IndexList* seed_faces, 
			const IndexList* seed_vertices) const
	{
		std::vector<bool> mask_seedface(m_faces.rows(), false);
		std::vector<bool> mask_seedvertex(m_vertices.rows(), false);
		if (seed_faces)
			for (auto x : *seed_faces)
				mask_seedface[x] = true;

		if (seed_vertices)
			for (auto x : *seed_vertices)
				mask_seedvertex[x] = true;

		auto idxface_conn = get_connected_components_as_faces();
		VECTOR_b vertex_included(get_num_vertices());
		vertex_included.fill(false);

		std::vector<IndexList> output;
		VECTOR_b mask_vertex_in_component(get_num_vertices());
		for (size_t i = 0; i < idxface_conn.size(); i++)
		{
			auto& idxfaces = idxface_conn[i];

			mask_vertex_in_component.fill(false);
			for (auto idxf : idxfaces)
			{
				iVECTOR_3 f = m_faces.row(idxf);
				mask_vertex_in_component(f[0]) = true;
				mask_vertex_in_component(f[1]) = true;
				mask_vertex_in_component(f[2]) = true;
			}

			//get the vertices
			IndexList idxv_comp;
			find_nonzeros(mask_vertex_in_component, &idxv_comp);

			//accept this component?
			bool accept_component = false;
			for (auto x : idxfaces)
				accept_component |= mask_seedface[x];
			for (auto x : idxv_comp)
				accept_component |= mask_seedvertex[x];

			if (accept_component)
				output.push_back(idxv_comp);

			//mask the visited vertices, whether or not the component is selected as output
			for (auto k : idxv_comp)
				vertex_included(k) = true;
		}

		//vertices left out are isolated vertices
		for (Eigen::Index i = 0; i < vertex_included.size(); i++)
		{
			if (!vertex_included(i) && mask_seedvertex[i]) //found an isolated vertex and the vertex is selected in seed
				output.push_back({ (int_type)i });
		}

		return output;
	}

	inline std::vector<MeshPropertyReader::IndexList> 
		MeshPropertyReader::get_connected_components_as_faces() const
	{
		namespace pmp = CGAL::Polygon_mesh_processing;
		//create a property map for writing component index
		auto n_face = m_mesh.number_of_faces();
		std::vector<TRIMESH_3::faces_size_type> face2idxcomp(n_face, 0);

//		auto fmap_face2index = boost::make_function_property_map<TRIMESH_3::face_index, int>([](TRIMESH_3::face_index idx) {return (int)idx; });
//		auto face_comp_map = boost::make_iterator_property_map(face2idxcomp.begin(), fmap_face2index);

		auto face_comp_map = boost::make_function_property_map<TRIMESH_3::face_index, TRIMESH_3::faces_size_type&>(
			[&](TRIMESH_3::face_index idx)->TRIMESH_3::faces_size_type& {return face2idxcomp[(int)idx]; });
		
		int ncomp = CGAL::Polygon_mesh_processing::connected_components(m_mesh, face_comp_map);
		std::vector<IndexList> output(ncomp);
		for (size_t i = 0; i < face2idxcomp.size(); i++)
			output[face2idxcomp[i]].push_back((int_type)i);
		return output;
	}

	inline std::vector<MeshPropertyReader::IndexList>
		MeshPropertyReader::get_connected_components_as_faces(
			const IndexList* seed_faces, 
			const IndexList* seed_vertices) const
	{
		auto complist = get_connected_components_as_faces();
		std::vector<bool> mask_face(get_num_faces(), false);	//selected faces
		std::vector<bool> mask_vertex(get_num_vertices(), false);	//selected vertex
		std::vector<bool> mask_comp(complist.size(), false);	//selected component

		if (seed_faces)
		{
			for (auto x : *seed_faces)
				mask_face[x] = true;

			for (size_t i = 0; i < complist.size(); i++)
			{
				for (auto k : complist[i])
					if (mask_face[k])
					{
						mask_comp[i] = true;
						break;
					}
			}
		}

		if (seed_vertices)
		{
			for (auto x : *seed_vertices)
				mask_vertex[x] = true;

			for (size_t i = 0; i < complist.size(); i++)
			{
				for (auto k : complist[i])
				{
					iVECTOR_3 idxv = m_faces.row(k);
					bool hit = mask_vertex[idxv[0]] || mask_vertex[idxv[1]] || mask_vertex[idxv[2]];
					if (hit)
					{
						mask_comp[i] = true;
						break;
					}
				}
			}
		}

		decltype(complist) output;
		for (size_t i = 0; i < complist.size(); i++)
			if (mask_comp[i])
				output.push_back(complist[i]);
		return output;
	}

	inline void MeshPropertyReader::init(int num_vertices, const iMATRIX& faces)
	{
		if (num_vertices < 0)
			num_vertices = faces.maxCoeff() + 1;

		fMATRIX vertices(num_vertices, 3);
		vertices.setRandom();
		init(vertices, faces);

		m_vertices = vertices;
		m_faces = faces;
	}

	inline void MeshPropertyReader::init(const fMATRIX& vertices, const iMATRIX& faces)
	{
		fMATRIX v;
		if (vertices.cols() == 3)
			v = vertices;
		else if (vertices.cols() == 2)
		{
			v.resize(vertices.rows(), 3);
			v.leftCols(2) = vertices;
		}
		else {
			assert_throw(false, "vertices should be 2d or 3d");
		}

		//consistently orient all the faces
		//auto ptslist = to_point_list_3(v);
		//auto facelist = to_face_list(faces);
		//CGAL::Polygon_mesh_processing::orient_polygon_soup(ptslist, facelist);
		//make_trimesh_3(to_matrix(ptslist), to_matrix(facelist), m_mesh);

		make_trimesh_3(v, faces, m_mesh);

		m_vertices = vertices;
		m_faces = faces;
	}

	inline std::vector<iVECTOR_2> MeshPropertyReader::get_border_edges() const
	{
		std::vector<iVECTOR_2> edgelist;
		BOOST_FOREACH(TRIMESH_3::Edge_index e, m_mesh.edges())
		{
			bool ok = e!=m_mesh.null_edge() && m_mesh.is_border(e);
			if (ok)
			{
				int u = m_mesh.source(e.halfedge());
				int v = m_mesh.target(e.halfedge());
				edgelist.emplace_back(u, v);
			}
		}
		return edgelist;
	}

	inline std::vector<iVECTOR_2> MeshPropertyReader::get_non_border_edges() const
	{
		std::vector<iVECTOR_2> edgelist;
		BOOST_FOREACH(TRIMESH_3::Edge_index e, m_mesh.edges())
		{
			bool ok = e!=m_mesh.null_edge() && !m_mesh.is_border(e);
			if (ok)
			{
				int u = m_mesh.source(e.halfedge());
				int v = m_mesh.target(e.halfedge());
				edgelist.emplace_back(u, v);
			}
		}
		return edgelist;
	}

	inline MeshPropertyReader::IndexList MeshPropertyReader::get_border_vertices() const
	{
		IndexList idx_border_vertices;
		BOOST_FOREACH(TRIMESH_3::Vertex_index v, m_mesh.vertices())
		{
			bool ok = m_mesh.is_border(v, false);
			if (ok)
				idx_border_vertices.push_back(v);
		}
		
		return idx_border_vertices;
	}

	/** \brief get vertices that are not on the border */
	inline MeshPropertyReader::IndexList MeshPropertyReader::get_non_border_vertices() const
	{
		IndexList idx_border_vertices;
		BOOST_FOREACH(TRIMESH_3::Vertex_index v, m_mesh.vertices())
		{
			bool ok = v!=m_mesh.null_vertex() && !m_mesh.is_border(v, false);
			if (ok)
				idx_border_vertices.push_back(v);
		}

		return idx_border_vertices;
	}

	inline MeshPropertyReader::IndexList MeshPropertyReader::get_neighbor_faces(int_type idxface) const
	{
		IndexList idx_nb_faces;
		auto h = m_mesh.halfedge( TRIMESH_3::Face_index(idxface) );
		BOOST_FOREACH(TRIMESH_3::Face_index f, m_mesh.faces_around_face(h))
		{
			if(f != m_mesh.null_face())
				idx_nb_faces.push_back(f);
		}
		return idx_nb_faces;
	}

	inline MeshPropertyReader::IndexList MeshPropertyReader::get_neighbor_vertices(int_type idxv) const
	{
		IndexList idx_nb_vertices;
		auto h = m_mesh.halfedge(TRIMESH_3::Vertex_index(idxv));
		BOOST_FOREACH(TRIMESH_3::Vertex_index v, m_mesh.vertices_around_target(h))
		{
			if(v != m_mesh.null_vertex())
				idx_nb_vertices.push_back(v);
		}
		return idx_nb_vertices;
	}

}
