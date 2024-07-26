#pragma once
#include <igcclib/geometry/TriangularMesh.hpp>

namespace fbxsdk
{
	class FbxNode;
	class FbxMesh;
}

namespace _NS_UTILITY {

	void read_mesh_list_from_fbx_file(const std::string& filename, std::map<std::string, TriangularMesh>& out_mesh_list);

	fMATRIX_4 get_local_transformation(const fbxsdk::FbxNode* node);

	fMATRIX_4 get_global_transformation(fbxsdk::FbxNode* node);

	void LoadUVInofrmation(fbxsdk::FbxMesh* pMesh, std::vector<double>& texcoord_vec, std::vector<int>& uv_faces_vec);

	void read_mesh_from_fbx_node(fbxsdk::FbxNode* node, TriangularMesh& out_mesh);	

}
